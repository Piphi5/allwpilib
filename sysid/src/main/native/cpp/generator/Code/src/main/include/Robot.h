// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <array>
#include<functional>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "networktables/NetworkTableEntry.h"
#include <frc/livewindow/LiveWindow.h>
#include <frc/Joystick.h>
#include <units/time.h>

class Robot : public frc::TimedRobot {
 public:
  Robot() {
   TimedRobot{5_ms};
   frc::LiveWindow::GetInstance()->DisableAllTelemetry();
  }
  void RobotInit() override
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

  enum Sides {
    LEFT,
    RIGHT,
    FOLLOWER
  };

 private:
  frc::SendableChooser<std::string> m_chooser;
  nt::NetworkTableEntry autoSpeedEntry;
  nt::NetworkTableEntry telemetryEntry;
  nt::NetworkTableEntry rotateEntry;
  std::array<double, 10> data;
  std::string m_autoSelected;
  std::function<double()> leftEncoderPosition;
  std::function<double()> leftEncoderVelocity;
  std::function<double()> rightEncoderPosition;
  std::function<double()> rightEncoderVelocity;
  

};
