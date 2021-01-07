// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/Analyzer.h"

#include <implot.h>

#include <glass/Context.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_stdlib.h>
#include <wpi/FileSystem.h>
#include <wpi/raw_ostream.h>

using namespace sysid;

// Methods that return various ImPlotPoint values for plotting, given a
// const_iterator to the data.
ImPlotPoint GetVelocityVsVoltage(void* data, int idx) {
  auto& d = *static_cast<std::vector<PreparedData>*>(data);
  return ImPlotPoint(d[idx].voltage, d[idx].velocity);
}

ImPlotPoint GetAccelerationVsVoltage(void* data, int idx) {
  auto& d = *static_cast<std::vector<PreparedData>*>(data);
  return ImPlotPoint(d[idx].voltage, d[idx].acceleration);
}

ImPlotPoint GetVelocityVsTime(void* data, int idx) {
  auto& d = *static_cast<std::vector<PreparedData>*>(data);
  return ImPlotPoint(d[idx].timestamp, d[idx].velocity);
}

ImPlotPoint GetAccelerationVsTime(void* data, int idx) {
  auto& d = *static_cast<std::vector<PreparedData>*>(data);
  return ImPlotPoint(d[idx].timestamp, d[idx].acceleration);
}

Analyzer::Analyzer() {
  // Fill the StringMap with preset values.
  m_presets["Default"] = presets::kDefault;
  m_presets["WPILib (2020-)"] = presets::kWPILibNew;
  m_presets["WPILib (Pre-2020)"] = presets::kWPILibOld;
  m_presets["CTRE (New)"] = presets::kCTRENew;
  m_presets["CTRE (Old)"] = presets::kCTREOld;
  m_presets["REV (Brushless)"] = presets::kREVBrushless;
  m_presets["REV (Brushed)"] = presets::kREVBrushed;

  // Load the last file location from storage if it exists.
  m_location = glass::GetStorage().GetStringRef("AnalyzerJSONLocation");

  // Configure plot data.
  m_timeDomainData.push_back(
      PlotData{"Quasistatic Velocity vs. Time", GetVelocityVsTime,
               [this] { return &std::get<0>(m_manager->GetRawData()); }});
  m_timeDomainData.push_back(
      PlotData{"Quasistatic Acceleration vs. Time", GetAccelerationVsTime,
               [this] { return &std::get<0>(m_manager->GetRawData()); }});
  m_timeDomainData.push_back(
      PlotData{"Dynamic Velocity vs. Time", GetVelocityVsTime,
               [this] { return &std::get<1>(m_manager->GetRawData()); }});
  m_timeDomainData.push_back(
      PlotData{"Dynamic Acceleration vs. Time", GetAccelerationVsTime,
               [this] { return &std::get<1>(m_manager->GetRawData()); }});
}

void Analyzer::Display() {
  // Get the current width of the window. This will be used to scale
  // our UI elements.
  float width = ImGui::GetContentRegionAvail().x;

  // If this is the first call to Display() and there's a valid m_location, load
  // it.
  if (first) {
    if (!m_location->empty() && wpi::sys::fs::exists(*m_location)) {
      try {
        m_manager = std::make_unique<AnalysisManager>(
            *m_location, &m_preset, &m_loopType, &m_params, &m_selectedDataset);
        m_type = m_manager->GetAnalysisType();
        Calculate();
      } catch (const std::exception& e) {
        // If we run into an error here, let's just ignore it and make the user
        // explicitly select their file.
        *m_location = "";
      }
    } else {
      *m_location = "";
    }
    first = false;
  }

  // Show the file location along with an option to choose.
  if (ImGui::Button("Select")) {
    m_selector = std::make_unique<pfd::open_file>("Select Data");
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(width - ImGui::CalcTextSize("Select").x);
  ImGui::InputText("##location", m_location, ImGuiInputTextFlags_ReadOnly);

  // Allow the user to select which data set they want analyzed and add a reset
  // button. Also show the units and the units per rotation.
  if (m_manager) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 15);
    if (ImGui::Combo("Dataset", &m_selectedDataset, AnalysisManager::kKeys,
                     IM_ARRAYSIZE(AnalysisManager::kKeys))) {
      Calculate();
    }
    ImGui::SameLine(width - ImGui::CalcTextSize("Reset").x);
    if (ImGui::Button("Reset")) {
      m_manager.reset();
      *m_location = "";
    }
    ImGui::Spacing();
    ImGui::Text(
        "Units:              %s\n"
        "Units Per Rotation: %.4f",
        m_unit.c_str(), m_factor);
  }

  // Function that displays a read-only value.
  auto ShowGain = [](const char* text, double* data) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
    ImGui::InputDouble(text, data, 0.0, 0.0, "%.3f",
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
      float beginY = ImGui::GetCursorPosY();
      ShowGain("Ks", &m_ff[0]);
      ShowGain("Kv", &m_ff[1]);
      ShowGain("Ka", &m_ff[2]);

      if (m_type == analysis::kElevator) {
        ShowGain("Kg", &m_ff[3]);
      } else if (m_type == analysis::kArm) {
        ShowGain("Kcos", &m_ff[3]);
      }

      ShowGain("r-squared", &m_rs);
      float endY = ImGui::GetCursorPosY();

      // Come back to the starting y pos.
      ImGui::SetCursorPosY(beginY);

      auto ShowDiagnostics = [](const char* text) {
        ImGui::SetCursorPosX(ImGui::GetFontSize() * 15);
        if (ImGui::Button(text)) {
          ImGui::OpenPopup(text);
        }
      };

      ShowDiagnostics("Voltage-Domain Diagnostics");
      ShowDiagnostics("Time-Domain Diagnostics");

      auto size = ImGui::GetIO().DisplaySize;
      ImGui::SetNextWindowSize(ImVec2(size.x / 3, size.y * 0.8));

      if (ImGui::BeginPopupModal("Time-Domain Diagnostics")) {
        for (auto&& data : m_timeDomainData) {
          if (ImGui::CollapsingHeader(data.name)) {
            ImPlot::FitNextPlotAxes();
            ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 2);
            if (ImPlot::BeginPlot(data.name)) {
              ImPlot::PlotScatter("", data.getter, data.data(),
                                  data.data()->size());
              ImPlot::EndPlot();
            }
          }
        }
        if (ImGui::Button("Close")) {
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }

      ImGui::SetCursorPosY(endY);
    }
  }
  if (ImGui::CollapsingHeader("Feedback Analysis")) {
    // Depending on whether a file has been selected or not, display a generic
    // warning message or show the feedback gains.
    if (!m_manager) {
      ImGui::Text("Please Select a JSON File");
    } else {
      // Allow the user to select a feedback controller preset.
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
      if (ImGui::Combo("Gain Preset", &m_selectedPreset, kPresetNames,
                       IM_ARRAYSIZE(kPresetNames))) {
        m_preset = m_presets[kPresetNames[m_selectedPreset]];
        m_presetModified = false;
        Calculate();
      }

      if (m_presetModified) {
        ImGui::SameLine();
        ImGui::TextDisabled("(modified)");
      }

      float beginY = ImGui::GetCursorPosY();
      // Show our feedback controller preset values.
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      double value = m_preset.outputConversionFactor * 12;
      if (ImGui::InputDouble("Max Controller Output", &value, 0.0, 0.0,
                             "%.1f") &&
          value > 0) {
        m_preset.outputConversionFactor = value / 12.0;
        m_presetModified = true;
        Calculate();
      }

      auto ShowPresetValue = [this](const char* text, double* data,
                                    float cursorX = 0.0f) {
        if (cursorX > 0) {
          ImGui::SetCursorPosX(cursorX);
        }

        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
        if (ImGui::InputDouble(text, data, 0.0, 0.0, "%.4f") && *data > 0) {
          m_presetModified = true;
          Calculate();
        }
      };

      // Show controller period.
      ShowPresetValue("Controller Period (s)",
                      reinterpret_cast<double*>(&m_preset.period));

      // Show whether the controller gains are time-normalized.
      if (ImGui::Checkbox("Time-Normalized?", &m_preset.normalized)) {
        m_presetModified = true;
        Calculate();
      }
      float endY = ImGui::GetCursorPosY();

      // Show position/velocity measurement delay.
      ImGui::SetCursorPosY(beginY);
      ShowPresetValue(
          "Position Measurement Delay (s)",
          reinterpret_cast<double*>(&m_preset.positionMeasurementDelay),
          ImGui::GetFontSize() * 17);
      ShowPresetValue(
          "Velocity Measurement Delay (s)",
          reinterpret_cast<double*>(&m_preset.velocityMeasurementDelay),
          ImGui::GetFontSize() * 17);

      ImGui::SetCursorPosY(endY);

      ImGui::Separator();
      ImGui::Spacing();

      // Allow the user to select a loop type.
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
      if (ImGui::Combo("Loop Type", &m_selectedLoopType, kLoopTypes,
                       IM_ARRAYSIZE(kLoopTypes))) {
        m_loopType =
            static_cast<FeedbackControllerLoopType>(m_selectedLoopType);
        Calculate();
      }

      ImGui::Spacing();

      // Show Kp and Kd.
      beginY = ImGui::GetCursorPosY();
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      ImGui::InputDouble("Kp", &m_Kp, 0.0, 0.0, "%.3f",
                         ImGuiInputTextFlags_ReadOnly);

      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      ImGui::InputDouble("Kd", &m_Kd, 0.0, 0.0, "%.3f",
                         ImGuiInputTextFlags_ReadOnly);

      // Come back to the starting y pos.
      ImGui::SetCursorPosY(beginY);

      auto ShowLQRParam = [this](const char* text, double* data, float min,
                                 float max, float power = 2.0) {
        float val = *data;
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
        ImGui::SetCursorPosX(ImGui::GetFontSize() * 17);

        if (ImGui::SliderFloat(text, &val, min, max, "%.1f", power)) {
          *data = val;
          Calculate();
        }
      };

      if (m_selectedLoopType == 0) {
        ShowLQRParam("Max Position Error (units)", &m_params.qp, 0.05, 40.0);
      }

      ShowLQRParam("Max Velocity Error (units/s)", &m_params.qv, 0.05, 40.0);
      ShowLQRParam("Max Control Effort (V)", &m_params.r, 0.1, 12.0, 1.0);
    }
  }

  // Periodic functions
  try {
    SelectFile();
  } catch (const std::exception& e) {
    m_exception = e.what();
    ImGui::OpenPopup("Exception Caught!");
  }

  // Handle exceptions.
  if (ImGui::BeginPopupModal("Exception Caught!")) {
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s",
                       m_exception.c_str());
    ImGui::Text(
        "An exception at this stage usually means that the JSON data is "
        "malformed.");
    if (ImGui::Button("Close")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void Analyzer::SelectFile() {
  // If the selector exists and is ready with a result, we can store it.
  if (m_selector && m_selector->ready()) {
    // Store the location of the file and reset the selector.
    *m_location = m_selector->result()[0];
    m_selector.reset();

    // Create the analysis manager.
    m_manager = std::make_unique<AnalysisManager>(
        *m_location, &m_preset, &m_loopType, &m_params, &m_selectedDataset);
    m_type = m_manager->GetAnalysisType();
    Calculate();
  }
}

void Analyzer::Calculate() {
  auto gains = m_manager->Calculate();
  m_ff = std::get<0>(gains.ff);
  m_rs = std::get<1>(gains.ff);
  m_Kp = std::get<0>(gains.fb);
  m_Kd = std::get<1>(gains.fb);
  m_unit = m_manager->GetUnit();
  m_factor = m_manager->GetFactor();
}
