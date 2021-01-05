// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <string_view>

namespace wpi {
class StringRef;
}  // namespace wpi

namespace sysid {
struct AnalysisType {
  /** The name of the analysis type */
  const char* name;

  /** The number of independent variables for feedforward analysis */
  size_t independentVariables;

  constexpr bool operator==(const AnalysisType& rhs) const {
    return std::string_view(name) == rhs.name;
  }

  constexpr bool operator!=(const AnalysisType& rhs) const {
    return !operator==(rhs);
  }
};

namespace analysis {
constexpr AnalysisType kDrivetrain{"Drivetrain", 3};
constexpr AnalysisType kElevator{"Elevator", 4};
constexpr AnalysisType kArm{"Arm", 4};
constexpr AnalysisType kSimple{"Simple", 3};

AnalysisType FromName(wpi::StringRef name);
}  // namespace analysis
}  // namespace sysid
