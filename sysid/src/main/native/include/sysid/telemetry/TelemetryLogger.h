// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <vector>

#include <glass/networktables/NetworkTablesHelper.h>
#include <ntcore_cpp.h>
#include <wpi/StringRef.h>

namespace sysid {
/**
 * This class is responsible for logging the mechanism characterization data
 * from NetworkTables.
 */
class TelemetryLogger {
 public:
  /**
   * Constructs an instance of the telemetry logger with the given NT_Inst.
   */
  explicit TelemetryLogger(
      std::function<double()> autospeed = [] { return 0.0; },
      NT_Inst instance = nt::GetDefaultInstance());

  /**
   * Check for new updates from NetworkTables and update the internal vector
   * with new data.
   */
  void Update();

  /**
   * Cancels the collection of data and returns the vector with the collected
   * data. Note that this clears the internal data vector.
   */
  std::vector<std::array<double, 10>> Cancel();

  /**
   * Returns whether the robot is enabled.
   */
  bool IsEnabled() const { return m_enabled; }

 private:
  // Helps with various NT functionality i.e. listeners, etc.
  glass::NetworkTablesHelper m_nt;
  std::function<double()> m_autospeedFunc;

  // Entries used to retrieve informaiton about the robot.
  NT_Entry m_fmsControl;
  NT_Entry m_telemetry;
  NT_Entry m_autospeed;

  // Stores the collected data and whether the robot is enabled.
  std::vector<std::array<double, 10>> m_data;
  bool m_enabled = false;
};
}  // namespace sysid
