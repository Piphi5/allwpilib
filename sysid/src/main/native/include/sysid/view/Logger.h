// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <string>

#include <glass/View.h>
#include <portable-file-dialogs.h>
#include <wpi/StringMap.h>

#include "sysid/telemetry/TelemetryManager.h"

namespace sysid {
/**
 * The logger GUI takes care of running the system idenfitication tests over
 * NetworkTables and logging the data. This data is then stored in a JSON file
 * which can be used for analysis.
 */
class Logger : public glass::View {
 public:
  Logger();

  void Display() override;
  void Hidden() override {}

 private:
  void SelectDataFolder();
  void CheckNTReset();

  VoltageParameters m_params{0.25_V / 1_s, 7_V, 4_V};

  std::unique_ptr<TelemetryManager> m_manager =
      std::make_unique<TelemetryManager>(m_params);

  std::unique_ptr<pfd::select_folder> m_selector;
  std::string m_jsonLocation;

  bool m_ntConnected = false;
  bool m_ntReset = true;

  int* m_team = nullptr;

  std::string m_opened;
  std::string m_exception;

  wpi::StringMap<std::string> m_tests;
};
}  // namespace sysid
