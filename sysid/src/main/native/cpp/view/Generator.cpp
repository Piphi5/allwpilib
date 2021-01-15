// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/Generator.h"

#include <glass/Context.h>
#include <imgui.h>
#include <imgui_stdlib.h>

#include "sysid/analysis/AnalysisType.h"

using namespace sysid;

Generator::Generator() {
  // Initialize persistent storage and assign pointers.
  auto& storage = glass::GetStorage();
  m_pTeam = storage.GetIntRef("Team");
  m_pUnitsPerRotation = storage.GetDoubleRef("Units Per Rotation", 1.0);
  m_pAnalysisType = storage.GetStringRef("Analysis Type", "Simple");
  m_pUnits = storage.GetStringRef("Units", "Meters");
}

void Generator::Display() {
  // Add team number input.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  ImGui::InputInt("Team", m_pTeam, 0, 0);

  // Add analysis type input.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
  ImGui::Combo("Analysis Type", &m_analysisIdx, kAnalysisTypes,
               IM_ARRAYSIZE(kAnalysisTypes));
  *m_pAnalysisType = kAnalysisTypes[m_analysisIdx];

  // Add section for motor and motor controller selection.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Motor / Motor Controller Selection");
  ImGui::Spacing();

  // Add motor port selection.
  bool drive = *m_pAnalysisType == "Drivetrain";
  for (size_t i = 0; i < m_portsCount; ++i) {
    // Ensure that our vector contains i+1 elements.
    if (m_primaryMotorPorts.size() == i) {
      m_primaryMotorPorts.emplace_back(i);
      m_secondaryMotorPorts.emplace_back(i);
    }

    // Add primary (left for drivetrain) motor ports.
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt(drive ? "L" : "Motor Port", &m_primaryMotorPorts[i], 0, 0);

    // Add secondary (right) motor ports.
    if (drive) {
      ImGui::SameLine(ImGui::GetFontSize() * 4);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
      ImGui::InputInt("R Motor Port", &m_primaryMotorPorts[i], 0, 0);
    }

    // Add buttons to add and remove ports.
    if (i == 0) {
      // +
      ImGui::SameLine(ImGui::GetFontSize() * 15);
      ImGui::Text("(+)");
      if (ImGui::IsItemClicked()) {
        ++m_portsCount;
      }

      // -
      if (m_portsCount > 1) {
        ImGui::SameLine(ImGui::GetFontSize() * 17);
        ImGui::Text("(-)");
        if (ImGui::IsItemClicked()) {
          --m_portsCount;
        }
      }
    }
  }

  // Add motor controller selection.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13);
  ImGui::Combo("Motor Controller", &m_motorControllerIdx, kMotorControllers,
               IM_ARRAYSIZE(kMotorControllers));

  // Add section for encoders.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Encoder Selection");
  ImGui::Spacing();

  // Add encoder selection.
  if (m_motorControllerIdx > 0) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13);
    ImGui::Combo("Encoder", &m_encoderIdx, kEncoders, IM_ARRAYSIZE(kEncoders));
  } else {
    m_encoderIdx = 2;
  }

  // Add encoder port selection if roboRIO is selected.
  if (m_encoderIdx > 1) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt("A##1", &m_primaryEncoderPorts[0], 0, 0);
    ImGui::SameLine(ImGui::GetFontSize() * 4);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt("B##1", &m_primaryEncoderPorts[1], 0, 0);

    // Add another row if we are running drive tests.
    if (drive) {
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
      ImGui::InputInt("A##2", &m_secondaryEncoderPorts[0], 0, 0);
      ImGui::SameLine(ImGui::GetFontSize() * 4);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
      ImGui::InputInt("B##2", &m_secondaryEncoderPorts[1], 0, 0);
    }
  }

  // Add CANCoder port selection.
  if (m_encoderIdx == 1 && m_motorControllerIdx > 0 &&
      m_motorControllerIdx < 4) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt(drive ? "L CANCoder Port" : "CANCoder Port",
                    &m_cancoderPorts[0], 0, 0);
    if (drive) {
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
      ImGui::InputInt("R CANCoder Port", &m_cancoderPorts[1], 0, 0);
    }
  }

  // Add gyro selection if selected is drivetrain.
  if (drive) {
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Text("Gyro");
    ImGui::Spacing();

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    ImGui::Combo("Gyro", &m_gyroIdx, kGyros, IM_ARRAYSIZE(kGyros));
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    ImGui::InputText("Gyro Parameter", &m_gyroCtor);
  }

  // Add section for other parameters.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Other Parameters");
  ImGui::Spacing();

  // Add units, units per rotation.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
  ImGui::InputText("Units", m_pUnits);
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
  ImGui::InputDouble("Units Per Rotation", m_pUnitsPerRotation, 0.0, 0.0,
                     "%.3f");

  // Add encoder resolution.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
  ImGui::InputDouble("Encoder EPR", &m_encoderEPR, 0.0, 0.0, "%.2f");
}
