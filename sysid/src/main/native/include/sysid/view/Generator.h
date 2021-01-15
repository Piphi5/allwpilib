// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <glass/View.h>
#include <wpi/SmallVector.h>

namespace sysid {
class Generator : public glass::View {
 public:
  Generator();
  void Display() override;

  static constexpr const char* kAnalysisTypes[] = {"Drivetrain", "Elevator",
                                                   "Arm", "Simple"};

  static constexpr const char* kMotorControllers[] = {"PWM",
                                                      "TalonSRX",
                                                      "VictorSPX",
                                                      "TalonFX",
                                                      "SPARK MAX (Brushless)",
                                                      "SPARK MAX (Brushed)"};

  static constexpr const char* kEncoders[] = {
      "Built-In", "CANCoder / Alternate", "roboRIO"};

  static constexpr const char* kGyros[] = {"Analog", "ADXRS450", "NavX",
                                           "Pigeon"};

 private:
  // Persistent storage pointers.
  int* m_pTeam;
  double* m_pUnitsPerRotation;
  std::string* m_pAnalysisType;
  std::string* m_pUnits;

  // Indices for combo boxes.
  int m_analysisIdx = 3;
  int m_motorControllerIdx = 1;
  int m_encoderIdx = 2;
  int m_gyroIdx = 2;

  // Vectors for motor ports.
  wpi::SmallVector<int, 3> m_primaryMotorPorts;
  wpi::SmallVector<int, 3> m_secondaryMotorPorts;
  size_t m_portsCount = 1;

  // Vectors for encoders.
  wpi::SmallVector<int, 2> m_primaryEncoderPorts{0, 1};
  wpi::SmallVector<int, 2> m_secondaryEncoderPorts{2, 3};
  wpi::SmallVector<int, 2> m_cancoderPorts{20, 21};

  // Encoder resolution.
  double m_encoderEPR = 4096.0;

  // Gyro constructor parameter.
  std::string m_gyroCtor = "SPI.Port.kMXP";
};  // namespace sysid
}  // namespace sysid
