// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

#include <wpi/StringRef.h>

#include "gtest/gtest.h"
#include "sysid/analysis/AnalysisManager.h"
#include "sysid/telemetry/TelemetryManager.h"

#if defined(__GNUG__) && !defined(__clang__) && __GNUC__ < 8
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

#include <ntcore_cpp.h>
#include <glass/networktables/NetworkTablesHelper.h>

using namespace sysid;
using namespace std::chrono_literals;

std::string getCodePath() {
  std::string wpilib_name = "allwpilib";

  // find build path
  std::string curr_path{fs::current_path()};
  size_t wpilib_dir = curr_path.find(wpilib_name);
  wpilib_dir += (wpilib_name.length());
  std::string build_path = curr_path.substr(0, wpilib_dir) +
                           "'/sysid/src/test/native/cpp/integration/Project'";
  return build_path;
}

void RunTest(TelemetryManager& m_manager, NT_Inst m_client, NT_Entry m_enable,
             wpi::StringRef test) {
  nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(true));
  m_manager.BeginTest(test);

  nt::Flush(m_client);
  std::this_thread::sleep_for(.01s);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    m_manager.Update();
    // std::this_thread::sleep_for(.1s);
  }

  nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(false));
  nt::Flush(m_client);
  m_manager.EndTest();
  std::this_thread::sleep_for(1s);
}

// Logger Constants
double m_quasistatic = .25;
double m_step = 5.0;

// Analyzer Constants
FeedbackControllerPreset m_preset = presets::kDefault;
FeedbackControllerLoopType m_loopType = FeedbackControllerLoopType::kVelocity;
LQRParameters m_params{0.05f, 0.05f, 0.1f};
double m_threshold = 0.2;
int m_window = 8;
int m_selectedDataset = 0;

class FullTest : public ::testing::Test {
 public:
  FullTest()
      : m_client(nt::CreateInstance()),
        m_nt(m_client),
        m_kill(m_nt.GetEntry("/SmartDashboard/SysIdKill")),
        m_enable(m_nt.GetEntry("/SmartDashboard/SysIdRun")) {
    std::string command = "./gradlew simulatejava ";
    std::string jdk = std::getenv("HOME");
    jdk += ((jdk.back() == fs::path::preferred_separator ? "" : "/")) +
           std::string("wpilib/2021/jdk");

    if (fs::exists(fs::status(jdk))) {
      command += "-Dorg.gradle.java.home=" + jdk + " 2>&1";
    }

    std::string savePath = getCodePath();

    std::system(
        std::string("cd " + savePath + ";" + "chmod +x gradlew;" + command)
            .c_str());

    nt::StartClient(m_client, "localhost", NT_DEFAULT_PORT);

    void* temp_data;

    NT_AddLogger(
        m_client, &temp_data,
        [](void* data, const struct NT_LogMessage* msg) {}, 0, 255);

    while (connections == 0) {
      NT_GetConnections(m_client, &connections);
    }
    // std::this_thread::sleep_for(5s);

    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(false));
    nt::Flush(m_client);
  }

  ~FullTest() {
    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(true));
    // nt::Flush(m_client);

    while (connections != 0) {
      NT_GetConnections(m_client, &connections);
    }
  }

  NT_Inst m_client;
  glass::NetworkTablesHelper m_nt;

  NT_Entry m_kill;
  NT_Entry m_enable;
  size_t connections{0};
};

TEST_F(FullTest, IntegrationTestDrive) {
  TelemetryManager m_manager{
      TelemetryManager::Settings{&m_quasistatic, &m_step}, m_client};

  // std::this_thread::sleep_for(20s);

  RunTest(m_manager, m_client, m_enable, "slow-forward");
  RunTest(m_manager, m_client, m_enable, "slow-backward");
  RunTest(m_manager, m_client, m_enable, "fast-forward");
  RunTest(m_manager, m_client, m_enable, "fast-backward");
  RunTest(m_manager, m_client, m_enable, "trackwidth");

  std::string save_path{fs::current_path()};

  std::string out_path{m_manager.SaveJSON(save_path)};

  // wpi::StringRef json_path {m_manager.SaveJSON(save_path)};
  // std::cout << save_path << " " << out_path << std::endl;

  AnalysisManager m_analyzer{
      out_path,
      AnalysisManager::Settings{&m_preset, &m_loopType, &m_params, &m_threshold,
                                &m_window, &m_selectedDataset}};

  auto gains = m_analyzer.Calculate();
  auto feedforward = std::get<0>(gains.ff);
  // std::cout << feedforward[2] << "\n";

  EXPECT_NEAR(feedforward[1], 1.98, 5E-2);
  EXPECT_NEAR(feedforward[2], 0.2, 5E-2);
  EXPECT_NEAR(feedforward[3], .762, 1E-3);
}

// TEST_F (FullTest, IntegrationTestArm) {
//   ASSERT_EQ(0, 0);
// }

// TEST_F (FullTest, IntegrationTestSimpleMotor) {
//   ASSERT_EQ(0, 0);
// }

// TEST_F (FullTest, IntegrationTestElevator) {
//   ASSERT_EQ(0, 0);
// }
