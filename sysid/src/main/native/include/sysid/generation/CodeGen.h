// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <wpi/StringRef.h>

namespace sysid {
/**
 * Stores necessary data for building a project.
 * For non-drive projects, the Left Motor fields will store the 
 * necessary data.
 */
struct ProjectData {
  std::vector<int> left_motor_ports;
  std::vector<int> right_motor_ports;
  std::vector<wpi::StringRef> left_motor_types;
  std::vector<wpi::StringRef> right_motor_types;
  double epr;
  double gearing;
  int left_encoder_ports[2];
  int right_encoder_ports[2];
  bool left_encoder_inverted = false;
  bool right_encoder_inverted = false;
  wpi::StringRef gyro_type = "";
  wpi::StringRef gyro_port = "";
};

/**
 * Manages the code for generating a robot project
 */
class CodeGenerator {
 public:
  /**
   * Constructs an instance of the Code Generator with the 
   * provided parameters to construct a robot project
   * 
   * @param data The necessary project parameters to generate
   * the desired robot project.
   */
  explicit CodeGenerator(ProjectData data);
  
  /**
   * Generates a robot project with the specified project data and save path.
   * It will also generate a config file for future use.
   */
  void GenerateProject(wpi::StringRef savePath);

 private:
  // The necessary parameters to generate a project
  ProjectData m_data;

  /**
   * Generates a Config file based off of the stored project data.
   * Saves it to the specified path location.
   */
  void MakeConfig(wpi::StringRef savePath);


};
}