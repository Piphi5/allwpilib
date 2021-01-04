// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/FeedbackAnalysis.h"

#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>

#include "sysid/analysis/FeedbackControllerPreset.h"

using namespace sysid;

std::tuple<Kp_t, Kd_t> CalculatePositionFeedbackGains(
    const FeedbackControllerPreset& preset, const LQRParameters& params,
    const FeedforwardGains& feedforwardGains) {
  // Get Ks, Kv, and Ka from the tuple.
  auto& [Ks, Kv, Ka] = feedforwardGains;

  // If acceleration requires no effort, velocity becomes an input for position
  // control. We choose an appropriate model in this case to avoid numerical
  // instabilities in the LQR.
  if (Ka > 1E-7_V / 1_mps_sq) {
    // Create a position system from our feedforward gains.
    auto system =
        frc::LinearSystemId::IdentifyPositionSystem<units::meter>(Kv, Ka);
    // Create an LQR with 2 states to control -- position and velocity.
    frc::LinearQuadraticRegulator<2, 1> controller{
        system,
        {params.qp.to<double>(), params.qv.to<double>()},
        {params.r.to<double>()},
        preset.period};
    // Compensate for any latency from sensor measurements, filtering, etc.
    controller.LatencyCompensate(system, preset.period,
                                 preset.positionMeasurementDelay);

    return {Kp_t(controller.K(0, 0)), Kd_t(controller.K(1, 0))};
  }

  // This is our special model to avoid instabilities in the LQR.
  auto system = frc::LinearSystem<1, 1, 1>(
      frc::MakeMatrix<1, 1>(0.0), frc::MakeMatrix<1, 1>(1.0),
      frc::MakeMatrix<1, 1>(1.0), frc::MakeMatrix<1, 1>(0.0));
  // Create an LQR with one state -- position.
  frc::LinearQuadraticRegulator<1, 1> controller{
      system, {params.qp.to<double>()}, {params.r.to<double>()}, preset.period};
  // Compensate for any latency from sensor measurements, filtering, etc.
  controller.LatencyCompensate(system, preset.period,
                               preset.positionMeasurementDelay);

  return {Kp_t(Kv.to<double>() * controller.K(0, 0)), Kd_t(0)};
}

std::tuple<Kp_t, Kd_t> CalculateVelocityFeedbackGains(
    const FeedbackControllerPreset& preset, const LQRParameters& params,
    const FeedforwardGains& feedforwardGains) {
  // Get Ks, Kv, and Ka from the tuple.
  auto& [Ks, Kv, Ka] = feedforwardGains;

  // If acceleration for velocity control requires no effort, the feedback
  // control gains approach zero. We special-case it here because numerical
  // instabilities arise in LQR otherwise.
  if (Ka < 1E-7_V / 1_mps_sq) {
    return {0_V / 1_m, 0_V / 1_mps};
  }

  // Create a velocity system from our feedforward gains.
  auto system =
      frc::LinearSystemId::IdentifyVelocitySystem<units::meter>(Kv, Ka);
  // Create an LQR controller with 1 state -- velocity.
  frc::LinearQuadraticRegulator<1, 1> controller{
      system, {params.qv.to<double>()}, {params.r.to<double>()}, preset.period};
  // Compensate for any latency from sensor measurements, filtering, etc.
  controller.LatencyCompensate(system, preset.period,
                               preset.velocityMeasurementDelay);

  return {Kp_t(controller.K(0, 0)), Kd_t(0)};
}