// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

namespace sysid {
/**
 * Represents the struct that is sent over the network by the robot program.
 * Note that this data format is comaptible with the old frc-characterization
 * tool.
 */
struct TelemetryData {
  /** Time when this data was captured. */
  units::second_t timestamp;

  /** The battery voltage. */
  units::volt_t voltage;

  /** The speed command sent to the motors (between -1 and 1). */
  double speed;

  /** The voltage sent to the primary motor (left for drivetrains). */
  units::volt_t pVolts;

  /** The voltage sent to the secondary motor (right for drivetrains). */
  units::volt_t sVolts;

  /** The position of the primary encoder (left for drivetrains). */
  units::meter_t pPosition;

  /** The position of the secondary encoder (right for drivetrains). */
  units::meter_t sPosition;

  /** The velocity of the primary encoder (left for drivetrains). */
  units::meters_per_second_t pVelocity;

  /** The velocity of the secondary encoder (right for drivetrains). */
  units::meters_per_second_t sVelocity;

  /** The gyro angle (only applicable for drivetrain tests) */
  units::radian_t angle;

  std::array<double, 10> ToArray() const {
    return {
        timestamp.to<double>(), voltage.to<double>(),   speed,
        pVolts.to<double>(),    sVolts.to<double>(),    pPosition.to<double>(),
        sPosition.to<double>(), pVelocity.to<double>(), sVelocity.to<double>(),
        angle.to<double>()};
  }
};
}  // namespace sysid
