// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <tuple>
#include <vector>

#include <wpi/StringMap.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>

#include "sysid/analysis/AnalysisType.h"

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

  static constexpr double kMotionThreshold = 0.0508;

  /**
   * Constructs an instance of the analysis manager with the given path.
   */
  explicit AnalysisManager(wpi::StringRef path);

  /**
   * Returns the analysis type of the current instance (read from the JSON).
   */
  const AnalysisType& GetAnalysisType() const { return m_type; }

 private:
  // Represents one "set" of data. 0 is slow tests, 1 is fast tests.
  using Storage =
      std::tuple<std::vector<PreparedData>, std::vector<PreparedData>>;

  void PrepareData();

  wpi::json m_data;
  wpi::StringMap<Storage> m_dataset;

  AnalysisType m_type;
  double m_factor;
};
}  // namespace sysid
