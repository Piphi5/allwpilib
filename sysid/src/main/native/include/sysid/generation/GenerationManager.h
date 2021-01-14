// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/StringRef.h>

#include "sysid/generation/CodeGen.h"


namespace sysid {
/**
 * This class is responsible for generating
 * and deploying robot projects that will 
 * collect data for system identification.
 */
class GenerationManager {
 public:
  /**
   * Constructs an instance of the GenerationManager.
   */ 
  explicit GenerationManager();

  /**
   * Reads a config data file and loads it
   * as the internally stored data.
   * 
   * @param configPath The location of the config file.
   */
  void ReadConfig(wpi::StringRef configPath);

  /**
   * Generates a project into the specified directory.
   * 
   * @param savePath The location of the robot project.
   */
  void GenerateProject(wpi::StringRef savePath);

  /**
   * Deploys a robot project based on the specified directory.
   * 
   * @param projectPath The location of the project.
   */
  void DeployProject(wpi::StringRef projectPath);
 private:
  // Stored parameters to generate a project 
  ProjectData m_storedData;
};
}