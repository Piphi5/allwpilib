// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <string>
#include <vector>

#include <glass/networktables/NetworkTablesHelper.h>
#include <ntcore_cpp.h>
#include <units/time.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>

namespace sysid {
/**
 * This class is reponsible for collecting data from the robot and storing it
 * inside a JSON.
 */
class TelemetryManager {
 public:
  /**
   * Represents settings for an instance of the TelemetryManager class. This
   * contains information about the quasistatic ramp rate for slow tests and the
   * step voltage for fast tests.
   *
   * The creator of this struct is responsible for managing the lifetime of the
   * pointers contained within the struct.
   */
  struct Settings {
    double* quasistaticRampRate;
    double* stepVoltage;
  };

  /**
   * Constructs an instance of the telemetry manager with the provided settings.
   *
   * @param settings The settings for this instance of the telemetry manager.
   */
  explicit TelemetryManager(Settings settings);

  /**
   * Begins a test with the given parameters.
   *
   * @param name The name of the test.
   */
  void BeginTest(wpi::StringRef name);

  /**
   * Ends the currently running test. If there is no test running, this is a
   * no-op.
   */
  void EndTest();

  /**
   * Updates the telemetry manager -- this adds a new autospeed entry and
   * collects newest data from the robot. This must be called periodically by
   * the user.
   */
  void Update();

  /**
   * Saves a JSON with the stored data at the given location.
   *
   * @param location The location to save the JSON at.
   */
  void SaveJSON(wpi::StringRef location);

  /**
   * Returns whether a test is currently running.
   *
   * @return Whether a test is currently running.
   */
  bool IsActive() const { return m_isRunningTest; }

  /**
   * Returns whether the specified test is running or has run.
   *
   * @param name The test to check.
   *
   * @return Whether the specified test is running or has run.
   */
  bool HasRunTest(wpi::StringRef name) const {
    return std::find(m_tests.cbegin(), m_tests.cend(), name) != m_tests.end();
  }

 private:
  /**
   * Stores information about a currently running test. This information
   * includes whether the robot will be traveling quickly (dynamic) or slowly
   * (quasistatic), the direction of movement, the start time of the test,
   * whether the robot is enabled, the current speed of the robot, and the
   * collected data.
   */
  struct TestParameters {
    bool fast;
    bool forward;
    double start;

    bool enabled = false;
    double speed = 0.0;

    std::vector<std::array<double, 10>> data{};
  };

  // Settings for this instance.
  Settings m_settings;

  // Test parameters for the currently running test.
  TestParameters m_params;
  bool m_isRunningTest = false;

  // A list of running or already run tests.
  std::vector<std::string> m_tests;

  // Stores the test data.
  wpi::json m_data;

  // NetworkTables instance and entries.
  glass::NetworkTablesHelper m_nt;
  NT_Entry m_autospeed;
  NT_Entry m_telemetry;
  NT_Entry m_fieldInfo;
};
}  // namespace sysid
