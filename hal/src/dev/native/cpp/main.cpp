// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "hal/HAL.h"

int main() {
  std::cout << "Hello World" << std::endl;
  std::cout << HAL_GetRuntimeType() << std::endl;
}
