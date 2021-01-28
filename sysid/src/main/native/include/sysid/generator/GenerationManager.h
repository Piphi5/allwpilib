// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#if defined(__GNUG__) && !defined(__clang__) && __GNUC__ < 8
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

#include <cstdio>
#include <string>


namespace sysid {
// TODO: Refactor files so that there's a method that handles pipe for deploy and build
class GenerationManager {
 public:
  void BuildProject(const std::string& savePath);
  void GenerateProject(const std::string& savePath);
 private:
  fs::path GetProjectPath(std::string path) const;
  std::string GetGradleCommand(const std::string& command);
  
};

}