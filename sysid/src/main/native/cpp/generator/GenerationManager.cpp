// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/generator/GenerationManager.h"

#include <cstdio>
#include <array>
#include <string>
#include <system_error>
#include <iostream>

using namespace sysid;

void GenerationManager::BuildProject(const std::string& savePath) {
  
  std::string result = "";
  std::array<char, 128> buffer{};
  std::string command = GetGradleCommand("build");

  auto pipe = popen(
      std::string("cd " + savePath + ";" + "chmod +x gradlew;" + command).c_str(),
      "r");
  if (!pipe) throw std::runtime_error("The command failed to execute.");

  while (!feof(pipe)) {
    if (std::fgets(buffer.data(), 128, pipe) != nullptr) {
      //result += buffer.data();
      std::cout << buffer.data();
    }
  }

  auto status = pclose(pipe);

  if (status == -1) throw std::runtime_error("Build Failed.");

  //std::cout<< result << std::endl;

}

std::string GenerationManager::GetGradleCommand(const std::string& action) {
  std::string command = "./gradlew " + action + " ";
  std::string jdk = std::getenv("HOME");
  jdk += ((jdk.back() == fs::path::preferred_separator ? "" : "/")) +
         std::string("wpilib/2021/jdk");

  if (fs::exists(fs::status(jdk))) {
    command += "-Dorg.gradle.java.home=" + jdk + " 2>&1";

  }
  
  
  return command;
}

// fs::path GenerationManager::GetProjectPath(const std::string& savePath) const {
//   return savePath + (savePath.back() == fs::path::preferred_separator
//                       ? m_name
//                       : fs::path::preferred_separator + m_name);

// }