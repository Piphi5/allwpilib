// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/acceleration.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <tuple>

namespace sysid {
using Kp_t = decltype(1_V / 1_m);
using Kd_t = decltype(1_V / 1_mps);

using Ks_t = decltype(1_V);
using Kv_t = decltype(1_V / 1_mps);
using Ka_t = decltype(1_V / 1_mps_sq);

using FeedforwardGains = std::tuple<Ks_t, Kv_t, Ka_t>;

struct FeedbackControllerPreset;

/**
 * Represents parameters used to calculate optimal feedback gains using a
 * linear-quadratic regulator (LQR).
 */
struct LQRParameters {
  /** The maximum allowable deviation in position. */
  units::meter_t qp;

  /** The maximum allowable deviation in velocity. */
  units::meters_per_second_t qv;

  /** The maximum allowable control effort */
  units::volt_t r;
};

/**
 * Calculates position feedback gains for the given controller preset, LQR
 * controller gain parameters and feedforward gains.
 *
 * @param preset           The feedback controller preset.
 * @param params           The parameters for calculating optimal feedback
 *                         gains.
 * @param feedforwardGains The feedforward gains for the system.
 */
std::tuple<Kp_t, Kd_t> CalculatePositionFeedbackGains(
    const FeedbackControllerPreset& preset, const LQRParameters& params,
    const FeedforwardGains& feedforwardGains);

/**
 * Calculates velocity feedback gains for the given controller preset, LQR
 * controller gain parameters and feedforward gains.
 *
 * @param preset           The feedback controller preset.
 * @param params           The parameters for calculating optimal feedback
 *                         gains.
 * @param feedforwardGains The feedforward gains for the system.
 */
std::tuple<Kp_t, Kd_t> CalculateVelocityFeedbackGains(
    const FeedbackControllerPreset& preset, const LQRParameters& params,
    const FeedforwardGains& feedforwardGains);
}  // namespace sysid
