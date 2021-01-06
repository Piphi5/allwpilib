// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <string>
#include <tuple>
#include <vector>

#include <wpi/SmallVector.h>
#include <wpi/StringMap.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>

#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FeedbackAnalysis.h"
#include "sysid/analysis/FeedbackControllerPreset.h"
#include "sysid/analysis/FeedforwardAnalysis.h"

namespace sysid {
/**
 * Represents each data point after it is cleaned and various parameters are
 * calculated.
 */
struct PreparedData {
  double timestamp;
  double voltage;
  double position;
  double velocity;
  double acceleration;
  double cos;
};

/**
 * Manages analysis of data. Each instance of this class represents a JSON file
 * that is read from storage.
 */
class AnalysisManager {
 public:
  static constexpr std::array<const char*, 4> kJsonDataKeys{
      "slow-forward", "slow-backward", "fast-forward", "fast-backward"};

  static constexpr const char* kKeys[] = {"Combined", "Forward", "Backward"};
  static constexpr double kMotionThreshold = 0.0508;

  // Struct for feedforward and feedback gains.
  struct Gains {
    std::tuple<std::vector<double>, double> ff;
    std::tuple<Kp_t, Kd_t> fb;
  };

  /**
   * Constructs an instance of the analysis manager with the given path and
   * pointer to the feedback controller preset and LQR parameters. The caller is
   * responsible for the ownership of the preset and params.
   */
  explicit AnalysisManager(wpi::StringRef path,
                           FeedbackControllerPreset* preset,
                           LQRParameters* params);

  /**
   * Selects a given dataset and recalculates the feedback and feedforward gains
   * based on the dataset. This returns the newly calculated gains for the
   * dataset.
   */
  const Gains& SelectDataset(wpi::StringRef dataset);
  const Gains& SelectDataset(int idx) { return SelectDataset(kKeys[idx]); }

  /**
   * Recalculates the gains with the newest preset and params.
   */
  const Gains& Recalculate();

  /**
   * Returns the analysis type of the current instance (read from the JSON).
   */
  const AnalysisType& GetAnalysisType() const { return m_type; }

  /**
   * Returns the currently calculated gains.
   */
  const Gains& GetGains() const { return m_gains; }

 private:
  // Represents one "set" of data. 0 is slow tests, 1 is fast tests.
  using Storage =
      std::tuple<std::vector<PreparedData>, std::vector<PreparedData>>;

  /**
   * Converts the raw data into "prepared data", after performing various
   * operations such as trimming, computing acceleration, etc.
   */
  void PrepareData();

  /**
   * Calculates feedforward gains for the given dataset.
   */
  void CalculateFeedforwardGains(wpi::StringRef dataset);

  /**
   * Calculates feedback gains for the given dataset.
   */
  void CalculateFeedbackGains(wpi::StringRef dataset);

  // JSON data and trimmed data.
  wpi::json m_data;
  wpi::StringMap<Storage> m_datasets;
  std::string m_dataset;

  // The analysis type and the units factor.
  AnalysisType m_type;
  double m_factor;

  // Preset and params for the feedback controller calculation.
  FeedbackControllerPreset* m_preset;
  LQRParameters* m_params;

  // Feedforward and feedback gains.
  Gains m_gains;
};
}  // namespace sysid
