// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <glass/networktables/NetworkTablesHelper.h>
#include <ntcore_cpp.h>
#include <wpi/StringRef.h>

#include "sysid/telemetry/TelemetryData.h"

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
  explicit TelemetryLogger(NT_Inst instance = nt::GetDefaultInstance());

  /**
   * Start the process of logging data.
   */
  void Start() { m_active = true; }

  /**
   * Check for new updates from NetworkTables and update the internal vector
   * with new data.
   */
  void Update();

  /**
   * Cancels the collection of data and returns the vector with the collected
   * data. Note that this clears the internal data vector.
   */
  std::vector<TelemetryData> Cancel();

 private:
  // Helps with various NT functionality i.e. listeners, etc.
  glass::NetworkTablesHelper m_nt;

  // Entries used to retrieve informaiton about the robot.
  NT_Entry m_fmsControl;
  NT_Entry m_telemetry;

  // Stores the collected data and whether the robot is enabled.
  std::vector<TelemetryData> m_data;
  bool m_enabled = false;

  // Whether the logger is active.
  bool m_active = false;
};
}  // namespace sysid
