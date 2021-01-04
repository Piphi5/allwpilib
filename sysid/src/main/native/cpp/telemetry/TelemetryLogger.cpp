// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/telemetry/TelemetryLogger.h"

#include <wpi/raw_ostream.h>

using namespace sysid;

TelemetryLogger::TelemetryLogger(NT_Inst instance)
    : m_nt(instance),
      m_fmsControl(m_nt.GetEntry("/FMSInfo/FMSControlData")),
      m_telemetry(m_nt.GetEntry("/SmartDashboard/SysIdTelemetry")) {
  m_nt.AddListener(m_fmsControl);
  m_nt.AddListener(m_telemetry);
}

void TelemetryLogger::Update() {
  if (m_active) {
    // Poll all the entries that we are listening to.
    for (auto&& event : m_nt.PollListener()) {
      // First check the enabled status from the FMS control word packet.
      if (event.entry == m_fmsControl && event.value &&
          event.value->IsDouble()) {
        uint32_t controlWord = event.value->GetDouble();
        m_enabled = ((controlWord & 0x01) != 0) ? 1 : 0;
      }
      // Then, check our telemetry data if the robot is enabled (the data is
      // useless if we are disabled so there's no need to log it).
      if (m_enabled && event.entry == m_telemetry && event.value &&
          event.value->IsDoubleArray()) {
        auto data = event.value->GetDoubleArray();

        // Do some quick validation -- make sure the size is 10.
        if (data.size() != 10) {
          wpi::outs() << "The size of the received data was not 10.\n";
          return;
        }

        // Store the data.
        m_data.push_back(TelemetryData{
            units::second_t(data[0]), units::volt_t(data[1]), data[2],
            units::volt_t(data[3]), units::volt_t(data[4]),
            units::meter_t(data[5]), units::meter_t(data[6]),
            units::meters_per_second_t(data[7]),
            units::meters_per_second_t(data[8]), units::radian_t(data[9])});
      }
    }
  }
}

std::vector<TelemetryData> TelemetryLogger::Cancel() {
  m_active = false;
  return std::move(m_data);
}
