// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <string>

#include <ntcore_cpp.h>
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
   * Constructs a telemetry manager with the given instance.
   */
  explicit TelemetryManager(NT_Inst instance = nt::GetDefaultInstance())
      : m_inst(instance) {}

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

 private:
  std::unique_ptr<TelemetryLogger> m_logger;
  std::string m_active;
  bool m_hasEnabled;

  NT_Inst m_inst;
  wpi::json m_data;
};
}  // namespace sysid
