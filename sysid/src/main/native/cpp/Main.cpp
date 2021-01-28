// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef RUNNING_SYSID_TESTS

#include <memory>

// testing imports (REMOVE LATER)
#include <filesystem>
#include <iostream>


#include <glass/Context.h>
#include <glass/Window.h>
#include <glass/WindowManager.h>
#include <wpigui.h>

#include "sysid/view/Analyzer.h"
#include "sysid/view/Generator.h"
#include "sysid/view/Logger.h"
#include "sysid/generator/CodeGen.h"
#include "sysid/generator/GenerationManager.h"

namespace gui = wpi::gui;
namespace fs = std::filesystem;

static std::unique_ptr<glass::WindowManager> gWindowManager;

glass::Window* gLoggerWindow;
glass::Window* gAnalyzerWindow;
glass::Window* gGeneratorWindow;

const char* GetWPILibVersion();

#ifdef _WIN32
int __stdcall WinMain(void* hInstance, void* hPrevInstance, char* pCmdLine,
                      int nCmdShow) {
#else
int main() {
#endif
  // Create the wpigui (along with Dear ImGui) and Glass contexts.
  // gui::CreateContext();
  // glass::CreateContext();

  // // Initialize window manager and add views.
  // gWindowManager = std::make_unique<glass::WindowManager>("SysId");
  // gWindowManager->GlobalInit();

  // gLoggerWindow =
  //     gWindowManager->AddWindow("Logger", std::make_unique<sysid::Logger>());

  // gAnalyzerWindow = gWindowManager->AddWindow(
  //     "Analyzer", std::make_unique<sysid::Analyzer>());

  // gGeneratorWindow = gWindowManager->AddWindow(
  //     "Generator", std::make_unique<sysid::Generator>());

  // // Configure save file.
  // gui::ConfigurePlatformSaveFile("sysid.ini");

  // // Add menu bar.
  // gui::AddLateExecute([] {
  //   ImGui::BeginMainMenuBar();
  //   gui::EmitViewMenu();

  //   if (ImGui::BeginMenu("Widgets")) {
  //     gWindowManager->DisplayMenu();
  //     ImGui::EndMenu();
  //   }

  //   bool about = false;
  //   if (ImGui::BeginMenu("Info")) {
  //     if (ImGui::MenuItem("About")) {
  //       about = true;
  //     }
  //     ImGui::EndMenu();
  //   }

  //   ImGui::EndMainMenuBar();

  //   if (about) {
  //     ImGui::OpenPopup("About");
  //     about = false;
  //   }
  //   if (ImGui::BeginPopupModal("About")) {
  //     ImGui::Text("SysId: System Identification for Robot Mechanisms");
  //     ImGui::Separator();
  //     ImGui::Text("v%s", GetWPILibVersion());
  //     if (ImGui::Button("Close")) {
  //       ImGui::CloseCurrentPopup();
  //     }
  //     ImGui::EndPopup();
  //   }
  // });

  // gui::Initialize("System Identification", 1280, 720);
  // gui::Main();

  // glass::DestroyContext();
  // gui::DestroyContext();
    sysid::ProjectData data = {
    {0, 1},
    {2, 3},
    {"SparkMax", "SparkMax"},
    {"SparkMax", "SparkMax"},
    {false, false},
    {false, false},
    {1},
    {1},
    {},
    {},
    {"Built-In"},
    {true},
    {true},
    {"Pigeon"},
    {"0"}
  };

  std::string curr_path {fs::current_path()};
  std::cout << "Current path is " << curr_path  << '\n';
  std::string input{curr_path + "/sysid/test.txt"};
  std::string test{curr_path + "/sysid/output.txt"};
  sysid::CodeGenerator generator = sysid::CodeGenerator(data, input);
  sysid::GenerationManager gen_manager = sysid::GenerationManager();
  
  //generator.GenerateCode(test);
  gen_manager.BuildProject(curr_path + "/sysid/src/main/native/cpp/generator/Code");
}

#endif  // RUNNING_SYSID_TESTS
