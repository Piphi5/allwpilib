// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include <wpi/SmallVector.h>
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
  static constexpr double kMotionThreshold = 0.1;

  // Represents one "set" of data. 0 is slow tests, 1 is fast tests.
  using Storage =
      std::tuple<std::vector<PreparedData>, std::vector<PreparedData>>;

  // Struct for feedforward and feedback gains.
  struct Gains {
    std::tuple<std::vector<double>, double> ff;
    std::tuple<double, double> fb;
  };

  /**
   * Constructs an instance of the analysis manager with the given path and
   * parameters. The caller is responsible for maintaining ownership of the
   * pointers passed to this constructor.
   *
   * @param path    The path to the JSON containing the data.
   * @param preset  The feedback controller preset.
   * @param type    The loop type (i.e. position or velocity) for the feedback
   *                controller.
   * @param params  The LQR parameters for the feedback controller.
   * @param dataset The dataset (i.e. Combined, Forward, Backward) to run
   *                analysis on.
   */
  explicit AnalysisManager(wpi::StringRef path,
                           FeedbackControllerPreset* preset,
                           FeedbackControllerLoopType* type,
                           LQRParameters* params, int* dataset);

  /**
   * Calculates the gains with the newest data.
   */
  const Gains& Calculate();

  /**
   * Returns the analysis type of the current instance (read from the JSON).
   */
  const AnalysisType& GetAnalysisType() const { return m_type; }

  /**
   * Returns the units of analysis.
   */
  const std::string& GetUnit() const { return m_unit; }

  /**
   * Returns the factor (a.k.a units per rotation) for analysis.
   */
  double GetFactor() const { return m_factor; }

  /**
   * Returns a reference to the iterator of the currently selected datset.
   * Unfortunately, due to ImPlot internals, the reference cannot be const so
   * the user should be careful not to change any data.
   */
  Storage& GetRawData() { return m_datasets.at(kKeys[*m_dataset]); }

 private:
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
  std::map<std::string, Storage> m_datasets;
  int* m_dataset;

  // The analysis type and the units factor.
  AnalysisType m_type;
  double m_factor;
  std::string m_unit;

  // Preset and params for the feedback controller calculation.
  FeedbackControllerPreset* m_preset;
  FeedbackControllerLoopType* m_loopType;
  LQRParameters* m_params;

  // Feedforward and feedback gains.
  Gains m_gains;
};
}  // namespace sysid
