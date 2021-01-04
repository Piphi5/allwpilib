// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <units/acceleration.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "gtest/gtest.h"
#include "sysid/analysis/FeedbackAnalysis.h"
#include "sysid/analysis/FeedbackControllerPreset.h"

TEST(FeedbackGainAnalysis, Velocity) {
  auto Ks = 1.01_V;
  auto Kv = 3.060_V / 1_mps;
  auto Ka = 0.327_V / 1_mps_sq;

  sysid::LQRParameters params{1_m, 1.5_mps, 7_V};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kDefault, params, {Ks, Kv, Ka});

  EXPECT_NEAR(Kp.to<double>(), 2.11, 0.05);
  EXPECT_NEAR(Kd.to<double>(), 0.00, 0.05);
}
