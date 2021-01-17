// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <fstream>
#include <functional>

namespace sysid {
/**
 * Stores necessary data for building a project.
 * For non-drive projects, the Left Motor fields will store the 
 * necessary data.
 */
struct ProjectData {
  std::vector<int> left_motor_ports;
  std::vector<int> right_motor_ports;
  std::vector<std::string> left_motor_types;
  std::vector<std::string> right_motor_types;
  std::vector<bool> left_motor_inverted;
  std::vector<bool> right_motor_inverted;
  int epr;
  double gearing;
  int left_encoder_ports[2];
  int right_encoder_ports[2];
  std::string encoder_type;
  bool left_encoder_inverted = false;
  bool right_encoder_inverted = false;
  std::string gyro_type = "";
  std::string gyro_port = "";
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
   * @param templatePath The path to the template
 
   */
  CodeGenerator(ProjectData data, const std::string& templatePath);
  
  /**
   * Generates a robot code with the specified project data and save path.
   * It will also generate a config file for future use.
   * @param savePath The path to the output file
   */
  void GenerateCode(const std::string& savePath);

 private:
  typedef void (CodeGenerator::*Command)(void);
  // The necessary parameters to generate a project
  ProjectData m_data;
  // The template to generate the code off of
  std::ifstream m_templateFile;
  // The file to output the generated code
  std::ofstream m_outFile;
  // Maps template commands to functions
  std::unordered_map<std::string, Command> m_commandMap;
  
  /**
   * Generates a Config file based off of the stored project data.
   * Saves it to the specified path location.
   * @param savePath The path to the output directory
   */
  void MakeConfig(const std::string& savePath);

  /**
   * Handles the logic of setting the EPR value.
   * Mainly serves to handle Spark Max's special EPR case.
   */
  void SetConstants();

  /**
   * Differentiates between a double sided project (drivetrain)
   * and a single sided project (simple-motor, elevator, arm).
   */
  void SetSides();

  /**
   * Sets up encoders for a specific motor.
   */
  void SetupEncoders();

  /**
   * Initializes and creates multiple motors
   */
  void CreateMotors();

  /**
   * Create a motor initialization method that sets up all the motor features.
   * @param motor motor controller to be setup
   */
  void SetupMotor(const std::string& motor);

  /**
   * Sets up the motors for manual control during the Teleop method. 
   */
  void ManualControl();

  /**
   * Initializes all the motors and sets up all the underlying logic
   * in the robotInit method.
   */
  void InitializeMotors();

  /**
   * Sets up the suppliers for the left and right encoders
   * @param side determines if it is the left or right side
   * @param encoder_inverted determines if the encoder is inverted or not
   * @param encoder_ports an array storing encoder ports
   */
  void SetupSuppliers(const std::string& side, bool encoder_inverted, int encoder_ports[2]);

  /**
   * Initializes leader motors in the robot init method.
   * This is a general method that can be used to setup the 
   * left and right sides of the robot project.
   * @param side determines if it is the left or right side
   * @param motors the motor vector that should be used
   * @param ports the ports vector that should be used
   * @param inverted the motor inverted vector that should be used
   */
  void InitializeLeaders(const std::string& side, std::vector<std::string> motors, std::vector<int> ports, std::vector<bool> inverted);

  /**
   * Initializes follower motors in the robot init method.
   * This is a general method to then be used to setup the left and right 
   * sides of the project.
   * @param side determines if it is the left side or right side
   * @param motors the motor vector that should be used
   * @param ports the ports vector that should be used
   * @param inverted the motor inverted vector that should be used
   */
  void InitializeFollowers(const std::string& side, std::vector<std::string> motors, std::vector<int> ports, std::vector<bool> inverted);

  /**
   * Sets up the Gyro for the drivetrain tests in the
   * robot init method.
   */
  void InitializeGyros();

  /**
   * Enables motors in the template to run at a certain speed.
   * Mainly serves to differentiate between moving a double sided project
   * (drivetrain) and a single sided project.
   */
  void EnableMotors();

  /**
   * Disables motors in the template.
   * Mainly serves to differentiate between moving a double sided project
   * (drivetrain) and a single sided project.
   */
  void DisableMotors();
};
}