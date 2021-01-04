project(sysid)

include(CompileWarnings)
include(LinkMacOSGUI)

file(GLOB_RECURSE sysid_srcs src/app/native/cpp/*.cpp)
file(GLOB_RECURSE sysid_hdrs src/app/native/include/*.h)

add_executable(sysid ${sysid_srcs} ${sysid_hdrs})
target_include_directories(sysid PUBLIC src/app/native/include)

wpilib_target_warnings(sysid)
wpilib_link_macos_gui(sysid)
target_link_libraries(sysid libglass ntcore wpigui wpimath wpiutil imgui)
