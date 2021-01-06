// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef RUNNING_SYSID_TESTS

#include <memory>

#include <glass/Context.h>
#include <glass/Window.h>
#include <glass/WindowManager.h>
#include <wpigui.h>

#include "sysid/view/Analyzer.h"
#include "sysid/view/Logger.h"

namespace gui = wpi::gui;

static std::unique_ptr<glass::WindowManager> gWindowManager;

glass::Window* gLoggerWindow;
glass::Window* gAnalyzerWindow;

#ifdef _WIN32
int __stdcall WinMain(void* hInstance, void* hPrevInstance, char* pCmdLine,
                      int nCmdShow) {
#else
int main() {
#endif
  // Create the wpigui (along with Dear ImGui) and Glass contexts.
  gui::CreateContext();
  glass::CreateContext();

  // Initialize window manager and add views.
  gWindowManager = std::make_unique<glass::WindowManager>("SysId");
  gWindowManager->GlobalInit();

  gLoggerWindow =
      gWindowManager->AddWindow("Logger", std::make_unique<sysid::Logger>());

  gAnalyzerWindow = gWindowManager->AddWindow(
      "Analyzer", std::make_unique<sysid::Analyzer>());

  // Configure save file.
  gui::ConfigurePlatformSaveFile("sysid.ini");

  // Add menu bar.
  gui::AddLateExecute([] {
    ImGui::BeginMainMenuBar();
    gui::EmitViewMenu();

    if (ImGui::BeginMenu("Widgets")) {
      gWindowManager->DisplayMenu();
      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  });

  gui::Initialize("System Identification", 1280, 720);
  gui::Main();

  glass::DestroyContext();
  gui::DestroyContext();
}

#endif  // RUNNING_SYSID_TESTS
