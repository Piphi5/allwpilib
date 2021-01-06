// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <glass/View.h>
#include <portable-file-dialogs.h>
#include <wpi/StringMap.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FeedbackAnalysis.h"
#include "sysid/analysis/FeedbackControllerPreset.h"

namespace sysid {
class Analyzer : public glass::View {
 public:
  static constexpr const char* kPresetNames[] = {
      "Default",    "WPILib (2020-)",  "WPILib (Pre-2020)", "CTRE (New)",
      "CTRE (Old)", "REV (Brushless)", "REV (Brushed)"};

  Analyzer();

  void Display() override;

 private:
  void SelectFile();

  // Everything related to feedback controller calculations.
  wpi::StringMap<FeedbackControllerPreset> m_presets;
  FeedbackControllerPreset m_preset = presets::kDefault;
  LQRParameters m_params{1_m, 1.5_mps, 7_V};
  int m_selected = 0;

  // Feedforward and feedback gains.
  AnalysisManager::Gains m_gains;

  // References to the feedforward gain vector and r-squared.
  const std::vector<double>& m_ff = std::get<0>(m_gains.ff);
  const double& m_rs = std::get<1>(m_gains.ff);

  // Reference to Kp and Kd.
  const Kp_t& m_Kp = std::get<0>(m_gains.fb);
  const Kd_t& m_Kd = std::get<1>(m_gains.fb);

  // Data analysis
  std::unique_ptr<AnalysisManager> m_manager;
  AnalysisType m_type;

  // File manipulation
  std::unique_ptr<pfd::open_file> m_selector;
  std::string m_location;
};
}  // namespace sysid
