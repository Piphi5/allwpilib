// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/base.h>
#include <units/time.h>
#include <units/voltage.h>

namespace sysid {
/**
 * Represents a preset for a specific feedback controller. This includes info
 * about the max controller output, the controller period, whether the gains
 * are time-normalized, and whether there are measurement delays from sensors or
 * onboard filtering.
 */
struct FeedbackControllerPreset {
  /** The conversion factor between volts and the final controller output. */
  units::unit_t<units::inverse<units::volt>> outputConversionFactor;

  /** The period at which the controller runs. */
  units::second_t period;

  /** Whether the controller gains are time-normalized. */
  bool normalized;

  /** The measurement delay in the position measurements. */
  units::second_t positionMeasurementDelay;

  /** The measurement delay in the velocity measurements. */
  units::second_t velocityMeasurementDelay;
};

constexpr FeedbackControllerPreset kDefault{12 / 12_V, 20_ms, true, 0_s, 0_s};

constexpr FeedbackControllerPreset kWPILibNew{kDefault};
constexpr FeedbackControllerPreset kWPILibOld{1 / 12_V, 50_ms, false, 0_s, 0_s};

// https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#changing-velocity-measurement-parameters
// 100 ms sampling period + a moving average window size of 64 (i.e. a 64-tap
// FIR) = 100 / 2 ms + (64 - 1) / 2 ms = 81.5 ms.
constexpr FeedbackControllerPreset kCTRENew{1 / 12_V, 1_ms, true, 0_s, 81.5_ms};
constexpr FeedbackControllerPreset kCTREOld{1023 / 12_V, 1_ms, false, 0_s,
                                            81.5_ms};

// According to a Rev employee on the FRC Discord the window size is 40 so delay
// = (40-1)/2 ms = 19.5 ms.
constexpr FeedbackControllerPreset kREVBrushless{1 / 12_V, 1_ms, false, 0_s,
                                                 19.5_ms};

// https://www.revrobotics.com/content/sw/max/sw-docs/cpp/classrev_1_1_c_a_n_encoder.html#a7e6ce792bc0c0558fb944771df572e6a
// 64-tap FIR = (64 - 1) / 2 ms = 31.5 ms delay.
constexpr FeedbackControllerPreset kREVBrushed{1 / 12_V, 1_ms, false, 0_s,
                                               31.5_ms};

}  // namespace sysid
