/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2017 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "PIDController.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <HAL/HAL.h>

#include "Notifier.h"
#include "PIDOutput.h"
#include "PIDSource.h"

using namespace frc;

static const std::string kP = "p";
static const std::string kI = "i";
static const std::string kD = "d";
static const std::string kF = "f";
static const std::string kSetpoint = "setpoint";
static const std::string kEnabled = "enabled";

template <class T>
constexpr const T& clamp(const T& value, const T& low, const T& high) {
  return std::max(low, std::min(value, high));
}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
PIDController::PIDController(double Kp, double Ki, double Kd, PIDSource* source,
                             PIDOutput* output, double period)
    : PIDController(Kp, Ki, Kd, 0.0, source, output, period) {}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
PIDController::PIDController(double Kp, double Ki, double Kd, double Kf,
                             PIDSource* source, PIDOutput* output,
                             double period) {
  m_controlLoop = std::make_unique<Notifier>(&PIDController::Calculate, this);

  m_P = Kp;
  m_I = Ki;
  m_D = Kd;
  m_F = Kf;

  // Save original source
  m_origSource = std::shared_ptr<PIDSource>(source, NullDeleter<PIDSource>());

  // Create LinearDigitalFilter with original source as its source argument
  m_filter = LinearDigitalFilter::MovingAverage(m_origSource, 1);
  m_pidInput = &m_filter;

  m_pidOutput = output;
  m_period = period;

  m_controlLoop->StartPeriodic(m_period);
  m_setpointTimer.Start();

  static int instances = 0;
  instances++;
  HAL_Report(HALUsageReporting::kResourceType_PIDController, instances);
}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
PIDController::PIDController(double Kp, double Ki, double Kd, PIDSource& source,
                             PIDOutput& output, double period)
    : PIDController(Kp, Ki, Kd, 0.0, &source, &output, period) {}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
PIDController::PIDController(double Kp, double Ki, double Kd, double Kf,
                             PIDSource& source, PIDOutput& output,
                             double period)
    : PIDController(Kp, Ki, Kd, Kf, &source, &output, period) {}

PIDController::~PIDController() {
  // forcefully stopping the notifier so the callback can successfully run.
  m_controlLoop->Stop();
  RemoveListeners();
}

/**
 * Read the input, calculate the output accordingly, and write to the output.
 * This should only be called by the Notifier.
 */
void PIDController::Calculate() {
  bool enabled;
  PIDSource* pidInput;
  PIDOutput* pidOutput;

  {
    std::lock_guard<wpi::mutex> lock(m_mutex);
    pidInput = m_pidInput;
    pidOutput = m_pidOutput;
    enabled = m_enabled;
  }

  if (pidInput == nullptr) return;
  if (pidOutput == nullptr) return;

  if (enabled) {
    double feedForward = CalculateFeedForward();

    std::lock_guard<wpi::mutex> lock(m_mutex);
    double input = pidInput->PIDGet();
    double result;
    PIDOutput* pidOutput;

    m_error = GetContinuousError(m_setpoint - input);

    if (m_pidInput->GetPIDSourceType() == PIDSourceType::kRate) {
      if (m_P != 0) {
        m_totalError = clamp(m_totalError + m_error, m_minimumOutput / m_P,
                             m_maximumOutput / m_P);
      }

      m_result = m_D * m_error + m_P * m_totalError + feedForward;
    } else {
      if (m_I != 0) {
        m_totalError = clamp(m_totalError + m_error, m_minimumOutput / m_I,
                             m_maximumOutput / m_I);
      }

      m_result = m_P * m_error + m_I * m_totalError +
                 m_D * (m_error - m_prevError) + feedForward;
    }
    m_prevError = m_error;

    m_result = clamp(m_result, m_minimumOutput, m_maximumOutput);

    pidOutput = m_pidOutput;
    result = m_result;

    pidOutput->PIDWrite(result);
  }
}

/**
 * Calculate the feed forward term.
 *
 * Both of the provided feed forward calculations are velocity feed forwards.
 * If a different feed forward calculation is desired, the user can override
 * this function and provide his or her own. This function does no
 * synchronization because the PIDController class only calls it in synchronized
 * code, so be careful if calling it oneself.
 *
 * If a velocity PID controller is being used, the F term should be set to 1
 * over the maximum setpoint for the output. If a position PID controller is
 * being used, the F term should be set to 1 over the maximum speed for the
 * output measured in setpoint units per this controller's update period (see
 * the default period in this class's constructor).
 */
double PIDController::CalculateFeedForward() {
  if (m_pidInput->GetPIDSourceType() == PIDSourceType::kRate) {
    return m_F * GetSetpoint();
  } else {
    double temp = m_F * GetDeltaSetpoint();
    m_prevSetpoint = m_setpoint;
    m_setpointTimer.Reset();
    return temp;
  }
}

/**
 * Set the PID Controller gain parameters.
 *
 * Set the proportional, integral, and differential coefficients.
 *
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 */
void PIDController::SetPID(double p, double i, double d) {
  {
    std::lock_guard<wpi::mutex> lock(m_mutex);
    m_P = p;
    m_I = i;
    m_D = d;
  }

  if (m_pEntry) m_pEntry.SetDouble(m_P);
  if (m_iEntry) m_iEntry.SetDouble(m_I);
  if (m_dEntry) m_dEntry.SetDouble(m_D);
}

/**
 * Set the PID Controller gain parameters.
 *
 * Set the proportional, integral, and differential coefficients.
 *
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 * @param f Feed forward coefficient
 */
void PIDController::SetPID(double p, double i, double d, double f) {
  {
    std::lock_guard<wpi::mutex> lock(m_mutex);
    m_P = p;
    m_I = i;
    m_D = d;
    m_F = f;
  }

  if (m_pEntry) m_pEntry.SetDouble(m_P);
  if (m_iEntry) m_iEntry.SetDouble(m_I);
  if (m_dEntry) m_dEntry.SetDouble(m_D);
  if (m_fEntry) m_fEntry.SetDouble(m_F);
}

/**
 * Get the Proportional coefficient.
 *
 * @return proportional coefficient
 */
double PIDController::GetP() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_P;
}

/**
 * Get the Integral coefficient.
 *
 * @return integral coefficient
 */
double PIDController::GetI() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_I;
}

/**
 * Get the Differential coefficient.
 *
 * @return differential coefficient
 */
double PIDController::GetD() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_D;
}

/**
 * Get the Feed forward coefficient.
 *
 * @return Feed forward coefficient
 */
double PIDController::GetF() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_F;
}

/**
 * Return the current PID result.
 *
 * This is always centered on zero and constrained the the max and min outs.
 *
 * @return the latest calculated output
 */
double PIDController::Get() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_result;
}

/**
 * Set the PID controller to consider the input to be continuous,
 *
 * Rather then using the max and min in as constraints, it considers them to
 * be the same point and automatically calculates the shortest route to
 * the setpoint.
 *
 * @param continuous true turns on continuous, false turns off continuous
 */
void PIDController::SetContinuous(bool continuous) {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_continuous = continuous;
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the output
 */
void PIDController::SetInputRange(double minimumInput, double maximumInput) {
  {
    std::lock_guard<wpi::mutex> lock(m_mutex);
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
    m_inputRange = maximumInput - minimumInput;
  }

  SetSetpoint(m_setpoint);
}

/**
 * Sets the minimum and maximum values to write.
 *
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
void PIDController::SetOutputRange(double minimumOutput, double maximumOutput) {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_minimumOutput = minimumOutput;
  m_maximumOutput = maximumOutput;
}

/**
 * Set the setpoint for the PIDController.
 *
 * @param setpoint the desired setpoint
 */
void PIDController::SetSetpoint(double setpoint) {
  {
    std::lock_guard<wpi::mutex> lock(m_mutex);

    if (m_maximumInput > m_minimumInput) {
      if (setpoint > m_maximumInput)
        m_setpoint = m_maximumInput;
      else if (setpoint < m_minimumInput)
        m_setpoint = m_minimumInput;
      else
        m_setpoint = setpoint;
    } else {
      m_setpoint = setpoint;
    }
  }

  if (m_setpointEntry) m_setpointEntry.SetDouble(m_setpoint);
}

/**
 * Returns the current setpoint of the PIDController.
 *
 * @return the current setpoint
 */
double PIDController::GetSetpoint() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_setpoint;
}

/**
 * Returns the change in setpoint over time of the PIDController.
 *
 * @return the change in setpoint over time
 */
double PIDController::GetDeltaSetpoint() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return (m_setpoint - m_prevSetpoint) / m_setpointTimer.Get();
}

/**
 * Returns the current difference of the input from the setpoint.
 *
 * @return the current error
 */
double PIDController::GetError() const {
  double setpoint = GetSetpoint();
  {
    std::lock_guard<wpi::mutex> lock(m_mutex);
    return GetContinuousError(setpoint - m_pidInput->PIDGet());
  }
}

/**
 * Returns the current average of the error over the past few iterations.
 *
 * You can specify the number of iterations to average with SetToleranceBuffer()
 * (defaults to 1). This is the same value that is used for OnTarget().
 *
 * @return the average error
 */
double PIDController::GetAvgError() const { return GetError(); }

/**
 * Sets what type of input the PID controller will use.
 */
void PIDController::SetPIDSourceType(PIDSourceType pidSource) {
  m_pidInput->SetPIDSourceType(pidSource);
}
/**
 * Returns the type of input the PID controller is using.
 *
 * @return the PID controller input type
 */
PIDSourceType PIDController::GetPIDSourceType() const {
  return m_pidInput->GetPIDSourceType();
}

/*
 * Set the percentage error which is considered tolerable for use with
 * OnTarget.
 *
 * @param percentage error which is tolerable
 */
void PIDController::SetTolerance(double percent) {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_toleranceType = kPercentTolerance;
  m_tolerance = percent;
}

/*
 * Set the absolute error which is considered tolerable for use with
 * OnTarget.
 *
 * @param percentage error which is tolerable
 */
void PIDController::SetAbsoluteTolerance(double absTolerance) {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_toleranceType = kAbsoluteTolerance;
  m_tolerance = absTolerance;
}

/*
 * Set the percentage error which is considered tolerable for use with
 * OnTarget.
 *
 * @param percentage error which is tolerable
 */
void PIDController::SetPercentTolerance(double percent) {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_toleranceType = kPercentTolerance;
  m_tolerance = percent;
}

/*
 * Set the number of previous error samples to average for tolerancing. When
 * determining whether a mechanism is on target, the user may want to use a
 * rolling average of previous measurements instead of a precise position or
 * velocity. This is useful for noisy sensors which return a few erroneous
 * measurements when the mechanism is on target. However, the mechanism will
 * not register as on target for at least the specified bufLength cycles.
 *
 * @param bufLength Number of previous cycles to average. Defaults to 1.
 */
void PIDController::SetToleranceBuffer(int bufLength) {
  std::lock_guard<wpi::mutex> lock(m_mutex);

  // Create LinearDigitalFilter with original source as its source argument
  m_filter = LinearDigitalFilter::MovingAverage(m_origSource, bufLength);
  m_pidInput = &m_filter;
}

/*
 * Return true if the error is within the percentage of the total input range,
 * determined by SetTolerance. This asssumes that the maximum and minimum input
 * were set using SetInput.
 *
 * Currently this just reports on target as the actual value passes through the
 * setpoint. Ideally it should be based on being within the tolerance for some
 * period of time.
 *
 * This will return false until at least one input value has been computed.
 */
bool PIDController::OnTarget() const {
  double error = GetError();

  std::lock_guard<wpi::mutex> lock(m_mutex);
  switch (m_toleranceType) {
    case kPercentTolerance:
      return std::fabs(error) < m_tolerance / 100 * m_inputRange;
      break;
    case kAbsoluteTolerance:
      return std::fabs(error) < m_tolerance;
      break;
    case kNoTolerance:
      // TODO: this case needs an error
      return false;
  }
  return false;
}

/**
 * Begin running the PIDController.
 */
void PIDController::Enable() {
  {
    std::lock_guard<wpi::mutex> lock(m_mutex);
    m_enabled = true;
  }

  if (m_enabledEntry) m_enabledEntry.SetBoolean(true);
}

/**
 * Stop running the PIDController, this sets the output to zero before stopping.
 */
void PIDController::Disable() {
  {
    std::lock_guard<wpi::mutex> lock(m_mutex);
    m_pidOutput->PIDWrite(0);
    m_enabled = false;
  }

  if (m_enabledEntry) m_enabledEntry.SetBoolean(false);
}

/**
 * Return true if PIDController is enabled.
 */
bool PIDController::IsEnabled() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_enabled;
}

/**
 * Reset the previous error, the integral term, and disable the controller.
 */
void PIDController::Reset() {
  Disable();

  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_prevError = 0;
  m_totalError = 0;
  m_result = 0;
}

std::string PIDController::GetSmartDashboardType() const {
  return "PIDController";
}

void PIDController::InitTable(std::shared_ptr<nt::NetworkTable> subtable) {
  RemoveListeners();
  if (subtable) {
    m_pEntry = subtable->GetEntry(kP);
    m_pEntry.SetDouble(GetP());
    m_iEntry = subtable->GetEntry(kI);
    m_iEntry.SetDouble(GetI());
    m_dEntry = subtable->GetEntry(kD);
    m_dEntry.SetDouble(GetD());
    m_fEntry = subtable->GetEntry(kF);
    m_fEntry.SetDouble(GetF());
    m_setpointEntry = subtable->GetEntry(kSetpoint);
    m_setpointEntry.SetDouble(GetSetpoint());
    m_enabledEntry = subtable->GetEntry(kEnabled);
    m_enabledEntry.SetBoolean(IsEnabled());

    m_pListener = m_pEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          std::lock_guard<wpi::mutex> lock(m_mutex);
          m_P = event.value->GetDouble();
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_iListener = m_iEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          std::lock_guard<wpi::mutex> lock(m_mutex);
          m_I = event.value->GetDouble();
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_dListener = m_dEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          std::lock_guard<wpi::mutex> lock(m_mutex);
          m_D = event.value->GetDouble();
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_fListener = m_fEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          std::lock_guard<wpi::mutex> lock(m_mutex);
          m_F = event.value->GetDouble();
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_setpointListener = m_setpointEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          SetSetpoint(event.value->GetDouble());
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_enabledListener = m_enabledEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsBoolean()) return;
          if (event.value->GetBoolean()) {
            Enable();
          } else {
            Disable();
          }
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);
  }
}

/**
 * Wraps error around for continuous inputs. The original error is returned if
 * continuous mode is disabled. This is an unsynchronized function.
 *
 * @param error The current error of the PID controller.
 * @return Error for continuous inputs.
 */
double PIDController::GetContinuousError(double error) const {
  if (m_continuous) {
    error = std::fmod(error, m_inputRange);
    if (std::fabs(error) > m_inputRange / 2) {
      if (error > 0) {
        return error - m_inputRange;
      } else {
        return error + m_inputRange;
      }
    }
  }

  return error;
}

void PIDController::UpdateTable() {}

void PIDController::StartLiveWindowMode() { Disable(); }

void PIDController::StopLiveWindowMode() {}

void PIDController::RemoveListeners() {
  if (m_pListener != 0) {
    m_pEntry.RemoveListener(m_pListener);
    m_pListener = 0;
  }
  if (m_iListener != 0) {
    m_iEntry.RemoveListener(m_iListener);
    m_iListener = 0;
  }
  if (m_dListener != 0) {
    m_dEntry.RemoveListener(m_dListener);
    m_dListener = 0;
  }
  if (m_fListener != 0) {
    m_fEntry.RemoveListener(m_fListener);
    m_fListener = 0;
  }
  if (m_setpointListener != 0) {
    m_setpointEntry.RemoveListener(m_setpointListener);
    m_setpointListener = 0;
  }
  if (m_enabledListener != 0) {
    m_enabledEntry.RemoveListener(m_enabledListener);
    m_enabledListener = 0;
  }
}