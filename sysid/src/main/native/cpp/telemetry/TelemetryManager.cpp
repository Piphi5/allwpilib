// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/telemetry/TelemetryManager.h"

#include <algorithm>
#include <system_error>

#include <wpi/raw_ostream.h>

using namespace sysid;

void TelemetryManager::BeginTest(wpi::StringRef path) {
  m_active = path;
  m_logger = std::make_unique<TelemetryLogger>(m_inst);
}

void TelemetryManager::Update() {
  // Don't do anything if we aren't running a test right now.
  if (!m_logger) {
    return;
  }

  m_logger->Update();

  // If we just enabled, store that value.
  if (!m_hasEnabled && m_logger->IsEnabled()) {
    m_hasEnabled = true;
  }

  // If the robot disabled after being enabled at one point during data
  // collection, cancel the test.
  if (m_hasEnabled && !m_logger->IsEnabled()) {
    CancelActiveTest();
  }
}

void TelemetryManager::CancelActiveTest() {
  if (m_logger) {
    // Retrieve the data from the logger and reset it.
    auto data = m_logger->Cancel();
    m_logger.reset();

    // Create our vector of raw data.
    std::vector<std::array<double, 10>> raw;
    raw.reserve(data.size());
    for (auto&& d : data) {
      raw.push_back(d.ToArray());
    }

    // Store the data in the JSON.
    m_data[m_active] = raw;

    // Reset the active test's name and enabled state.
    m_active = "";
    m_hasEnabled = false;
  }
}

void TelemetryManager::SaveJSON(wpi::StringRef path) {
  std::error_code ec;
  wpi::raw_fd_ostream os{path, ec};

  if (ec) {
    throw std::runtime_error("Cannot write to file: " + path.str());
  }

  os << m_data;
  os.flush();
  wpi::outs() << "Wrote JSON to: " << path << "\n";
}
