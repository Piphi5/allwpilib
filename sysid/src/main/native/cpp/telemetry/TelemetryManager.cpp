// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/telemetry/TelemetryManager.h"

#include <algorithm>
#include <system_error>

#include <wpi/raw_ostream.h>
#include <wpi/timestamp.h>

using namespace sysid;

void TelemetryManager::BeginTest(wpi::StringRef path) {
  m_tests.push_back(path);
  m_active = path;
  m_logger =
      std::make_unique<TelemetryLogger>([this] { return m_speed; }, m_inst);
}

void TelemetryManager::Update() {
  // Don't do anything if we aren't running a test right now.
  if (!m_logger) {
    return;
  }

  if (m_logger->IsEnabled()) {
    // Update the speed entry.
    if (wpi::StringRef(m_active).startswith("fast")) {
      m_speed = *m_step / 12.0;
    } else if (wpi::StringRef(m_active).startswith("slow")) {
      m_speed =
          (*m_quasistatic * (units::second_t(wpi::Now() * 1E-6) - m_start))
              .to<double>() /
          12.0;
    }
  } else {
    m_speed = 0.0;
  }

  m_logger->Update();

  // If we just enabled, store that value.
  if (!m_hasEnabled && m_logger->IsEnabled()) {
    m_hasEnabled = true;
    m_start = units::second_t(wpi::Now() * 1E-6);
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

    // Store the data in the JSON.
    m_data[m_active] = data;

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
