// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/Analyzer.h"

#include <imgui.h>
#include <imgui_stdlib.h>
#include <wpi/raw_ostream.h>

using namespace sysid;

Analyzer::Analyzer() {
  // Fill the StringMap with preset values.
  m_presets["Default"] = presets::kDefault;
  m_presets["WPILib (2020-)"] = presets::kWPILibNew;
  m_presets["WPILib (Pre-2020)"] = presets::kWPILibOld;
  m_presets["CTRE (New)"] = presets::kCTRENew;
  m_presets["CTRE (Old)"] = presets::kCTREOld;
  m_presets["REV (Brushless)"] = presets::kREVBrushless;
  m_presets["REV (Brushed)"] = presets::kREVBrushed;
}

void Analyzer::Display() {
  // Get the current width of the window. This will be used to scale
  // our UI elements.
  float width = ImGui::GetContentRegionAvail().x;

  // Show the file location along with an option to choose.
  if (ImGui::Button("Select")) {
    m_selector = std::make_unique<pfd::open_file>("Select Data");
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(width - ImGui::CalcTextSize("Select").x);
  ImGui::InputText("##location", &m_location, ImGuiInputTextFlags_ReadOnly);

  // Allow the user to select which data set they want analyzed and add a reset
  // button.
  if (m_manager) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 15);
    if (ImGui::Combo("Dataset", &m_selected, AnalysisManager::kKeys,
                     IM_ARRAYSIZE(AnalysisManager::kKeys))) {
      m_gains = m_manager->SelectDataset(m_selected);
    }
    ImGui::SameLine(width - ImGui::CalcTextSize("Reset").x);
    if (ImGui::Button("Reset")) {
      m_manager.reset();
      m_location = "";
    }
  }

  // Function that displays a read-only value.
  auto ShowGain = [](const char* text, const double* data) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
    ImGui::InputDouble(text, const_cast<double*>(data), 0.0, 0.0, "%.3f",
                       ImGuiInputTextFlags_ReadOnly);
  };

  ImGui::Spacing();
  ImGui::Spacing();

  // Collapsing Headers are used for Feedforward and Feedback Analysis.
  if (ImGui::CollapsingHeader("Feedforward Analysis")) {
    // Depending on whether a file has been selected or not, display a generic
    // warning message or show the feedforward gains.
    if (!m_manager) {
      ImGui::Text("Please Select a JSON File");
    } else {
      ShowGain("Ks", &m_ff[0]);
      ShowGain("Kv", &m_ff[1]);
      ShowGain("Ka", &m_ff[2]);

      if (m_type == analysis::kElevator) {
        ShowGain("Kg", &m_ff[3]);
      } else if (m_type == analysis::kArm) {
        ShowGain("Kcos", &m_ff[3]);
      }

      ShowGain("r-squared", &m_rs);
    }
  }
  if (ImGui::CollapsingHeader("Feedback Analysis")) {
    // Depending on whether a file has been selected or not, display a generic
    // warning message or show the feedback gains.
    if (!m_manager) {
      ImGui::Text("Please Select a JSON File");
    } else {
      float cursorY = ImGui::GetCursorPosY();
      ShowGain("Kp", reinterpret_cast<const double*>(&m_Kp));
      ShowGain("Kd", reinterpret_cast<const double*>(&m_Kd));

      // Display LQR Parameters and Feedback Controller Preset
      ImGui::SetCursorPosY(cursorY);
      auto ShowParam = [this](const char* text, double* data,
                              float pos = 0.0f) {
        ImGui::SetCursorPosX(pos > 0.0f ? pos : ImGui::GetCursorPosX());
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
        if (ImGui::InputDouble(text, data, 0.0, 0.0, "%.2f")) {
          if (*data > 0.0) {
            m_gains = m_manager->Recalculate();
          }
        }
      };
      ShowParam("Allowable Position Error",
                reinterpret_cast<double*>(&m_params.qp), width * 0.33);
      ShowParam("Allowable Velocity Error",
                reinterpret_cast<double*>(&m_params.qv), width * 0.33);
      ShowParam("Allowable Control Effort",
                reinterpret_cast<double*>(&m_params.r), width * 0.33);
    }
  }

  // Periodic functions
  SelectFile();
}

void Analyzer::SelectFile() {
  // If the selector exists and is ready with a result, we can store it.
  if (m_selector && m_selector->ready()) {
    // Store the location of the file and reset the selector.
    m_location = m_selector->result()[0];
    m_selector.reset();

    // Create the analysis manager.
    m_manager =
        std::make_unique<AnalysisManager>(m_location, &m_preset, &m_params);
    m_gains = m_manager->SelectDataset(m_selected);
    m_type = m_manager->GetAnalysisType();
  }
}
