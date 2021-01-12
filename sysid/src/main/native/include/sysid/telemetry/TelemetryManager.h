// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <ntcore_cpp.h>
#include <units/time.h>
#include <units/voltage.h>
#include <wpi/SmallVector.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>

#include "sysid/telemetry/TelemetryLogger.h"

namespace sysid {
/**
 * Manages all telemetry for a round of tests and saves the data to a JSON.
 */
class TelemetryManager {
 public:
  /**
   * Constructs a telemetry manager with the given quasistatic ramp rate, step
   * voltage and NT instance. The caller must take care of the lifetime of the
   * two pointers passed to this constructor.
   */
  explicit TelemetryManager(double* quasistatic, double* step,
                            NT_Inst instance = nt::GetDefaultInstance())
      : m_inst(instance), m_quasistatic(quasistatic), m_step(step) {}

  /**
   * Begins the test with the given name and stores the data. The test is
   * automatically canceled when the robot is disabled, but can also be canceled
   * manually with CancelTest().
   */
  void BeginTest(wpi::StringRef name);

  /**
   * This periodically checks for the disabled state for the robot and
   * performs other checks. It must be called periodically by the user.
   */
  void Update();

  /**
   * Cancels the actively running test.
   */
  void CancelActiveTest();

  /**
   * Registers a callback to call when a test is canceled.
   *
   * @param callback A function that takes in two doubles -- the distances
   *                 traveled by the left and right encoders.
   */
  void RegisterCancellationCallback(
      std::function<void(double, double)> callback) const {
    m_callbacks.push_back(std::move(callback));
  }

  /**
   * Returns the JSON object that contains all of the collected data.
   */
  const wpi::json& GetJSON() const { return m_data; }

  /**
   * Saves all of the collected data to a JSON at the given path.
   */
  void SaveJSON(wpi::StringRef path);

  /**
   * Returns whether a test is currently running.
   */
  bool IsActive() const { return m_logger.operator bool(); }

  /**
   * Checks if a test has run or is currently running.
   */
  bool HasRunTest(wpi::StringRef test) const {
    return std::find(m_tests.begin(), m_tests.end(), test.str()) !=
           m_tests.end();
  }

 private:
  std::unique_ptr<TelemetryLogger> m_logger;
  std::string m_active;
  bool m_hasEnabled;

  wpi::SmallVector<std::string, 5> m_tests;
  mutable wpi::SmallVector<std::function<void(double, double)>, 2> m_callbacks;

  NT_Inst m_inst;
  wpi::json m_data;

  double* m_quasistatic;
  double* m_step;

  double m_speed = 0;
  units::second_t m_start;
};
}  // namespace sysid
