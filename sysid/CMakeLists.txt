project(sysid)

include(AddTest)
include(CompileWarnings)
include(LinkMacOSGUI)

file(GLOB_RECURSE sysid_srcs src/main/native/cpp/*.cpp)
file(GLOB_RECURSE sysid_hdrs src/main/native/include/*.h)

add_executable(sysid ${sysid_srcs} ${sysid_hdrs})
target_include_directories(sysid PUBLIC src/main/native/include)

wpilib_target_warnings(sysid)
wpilib_link_macos_gui(sysid)
target_link_libraries(sysid libglassnt libglass ntcore wpigui wpimath wpiutil imgui)

if (WITH_TESTS)
  wpilib_add_test(sysid src/test/native/cpp)
  target_sources(sysid_test PRIVATE ${sysid_srcs})
  target_include_directories(sysid_test PRIVATE src/main/native/include)
  target_compile_definitions(sysid_test PUBLIC RUNNING_SYSID_TESTS)
  target_link_libraries(sysid_test libglassnt libglass ntcore wpigui wpimath wpiutil imgui gmock_main)
  wpilib_link_macos_gui(sysid_test)
endif()
