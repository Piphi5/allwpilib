// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/telemetry/TelemetryLogger.h"

#include <algorithm>

#include <wpi/raw_ostream.h>

using namespace sysid;

TelemetryLogger::TelemetryLogger(std::function<double()> autospeed,
                                 NT_Inst instance)
    : m_nt(instance),
      m_autospeedFunc(std::move(autospeed)),
      m_fmsControl(m_nt.GetEntry("/FMSInfo/FMSControlData")),
      m_telemetry(m_nt.GetEntry("/SmartDashboard/SysIdTelemetry")),
      m_autospeed(m_nt.GetEntry("/SmartDashboard/SysIdAutoSpeed")) {
  m_nt.AddListener(m_fmsControl);
  m_nt.AddListener(m_telemetry);
}

void TelemetryLogger::Update() {
  // Poll all the entries that we are listening to.
  for (auto&& event : m_nt.PollListener()) {
    // First check the enabled status from the FMS control word packet.
    if (event.entry == m_fmsControl && event.value && event.value->IsDouble()) {
      uint32_t controlWord = event.value->GetDouble();
      wpi::outs() << (controlWord & 0x01) << "\n";
      m_enabled = ((controlWord & 0x01) != 0) ? 1 : 0;
    }
    // Then, check our telemetry data if the robot is enabled (the data is
    // useless if we are disabled so there's no need to log it). Also set the
    // speed value.
    if (m_enabled) {
      if (event.entry == m_telemetry && event.value &&
          event.value->IsDoubleArray()) {
        auto data = event.value->GetDoubleArray();

        // Do some quick validation -- make sure the size is 10.
        if (data.size() != 10) {
          return;
        }

        // Convert to array and add to m_data.
        std::array<double, 10> d;
        std::copy_n(std::make_move_iterator(data.begin()), 10, d.begin());

        m_data.push_back(std::move(d));
      }
    }
  }

  // Update autospeed entry.
  if (m_enabled) {
    // Set the autospeed value.
    wpi::outs() << "Set autospeed to " << m_autospeedFunc() << "\n";
    nt::SetEntryValue(m_autospeed, nt::Value::MakeDouble(m_autospeedFunc()));
  }
}

std::vector<std::array<double, 10>> TelemetryLogger::Cancel() {
  return std::move(m_data);
}
