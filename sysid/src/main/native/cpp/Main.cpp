// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef RUNNING_SYSID_TESTS

#include <iostream>

#include <glass/Context.h>
#include <wpigui.h>

namespace gui = wpi::gui;

#ifdef _WIN32
int __stdcall WinMain(void* hInstance, void* hPrevInstance, char* pCmdLine,
                      int nCmdShow) {
#else
int main() {
#endif
  // Create the wpigui (along with Dear ImGui) and Glass contexts.
  gui::CreateContext();
  glass::CreateContext();

  // Configure save file.
  gui::ConfigurePlatformSaveFile("sysid.ini");

  // Add menu bar.
  gui::AddLateExecute([] {
    ImGui::BeginMainMenuBar();
    gui::EmitViewMenu();
    ImGui::EndMainMenuBar();
  });

  gui::Initialize("System Identification", 1280, 720);
  gui::Main();

  glass::DestroyContext();
  gui::DestroyContext();
}

#endif  // RUNNING_SYSID_TESTS
