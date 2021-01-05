// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/time.h>
#include <units/voltage.h>

#include <algorithm>
#include <memory>
#include <string>

#include <ntcore_cpp.h>
#include <wpi/SmallVector.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>

#include "sysid/telemetry/TelemetryLogger.h"

namespace sysid {
/**
 * Represents the voltages for the system identification tests.
 */
struct VoltageParameters {
  decltype(1_V / 1_s) quasistatic;
  units::volt_t step;
  units::volt_t rotation;
};

/**
 * Manages all telemetry for a round of tests and saves the data to a JSON.
 */
class TelemetryManager {
 public:
  /**
   * Constructs a telemetry manager with the given instance.
   */
  explicit TelemetryManager(const VoltageParameters& params,
                            NT_Inst instance = nt::GetDefaultInstance())
      : m_inst(instance), m_params(params) {}

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

  NT_Inst m_inst;
  VoltageParameters m_params;
  wpi::json m_data;

  double m_speed = 0;
  units::second_t m_start;
};
}  // namespace sysid
