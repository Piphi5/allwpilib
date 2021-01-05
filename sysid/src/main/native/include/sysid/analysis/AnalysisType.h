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
  const char* name;

  constexpr bool operator==(const AnalysisType& rhs) const {
    return std::string_view(name) == rhs.name;
  }

  constexpr bool operator!=(const AnalysisType& rhs) const {
    return !operator==(rhs);
  }
};

namespace analysis {
constexpr AnalysisType kDrivetrain{"Drivetrain"};
constexpr AnalysisType kElevator{"Elevator"};
constexpr AnalysisType kArm{"Arm"};
constexpr AnalysisType kSimple{"Simple"};

AnalysisType FromName(wpi::StringRef name);
}  // namespace analysis
}  // namespace sysid
