// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <ntcore_cpp.h>
#include <wpi/raw_ostream.h>

#include "gtest/gtest.h"
#include "sysid/telemetry/TelemetryData.h"
#include "sysid/telemetry/TelemetryManager.h"

class TelemetryManagerTest : public ::testing::Test {
 public:
  TelemetryManagerTest()
      : server_inst(nt::CreateInstance()), client_inst(nt::CreateInstance()) {
    nt::SetNetworkIdentity(server_inst, "server");
    nt::SetNetworkIdentity(client_inst, "client");
  }

  ~TelemetryManagerTest() override {
    nt::DestroyInstance(server_inst);
    nt::DestroyInstance(client_inst);
  }

  void Connect();

 protected:
  NT_Inst server_inst;
  NT_Inst client_inst;
};

void TelemetryManagerTest::Connect() {
  nt::StartServer(server_inst, "telemetrylogger.ini", "127.0.0.1", 10000);
  nt::StartClient(client_inst, "127.0.0.1", 10000);

  // Use connection listener to ensure we've connected.
  NT_ConnectionListenerPoller poller =
      nt::CreateConnectionListenerPoller(server_inst);
  nt::AddPolledConnectionListener(poller, false);
  bool timed_out = false;
  if (nt::PollConnectionListener(poller, 1.0, &timed_out).empty()) {
    FAIL() << "client didn't connect to server";
  }
}

TEST_F(TelemetryManagerTest, Data) {
  Connect();
  if (HasFatalFailure()) {
    return;
  }

  // Create the manager.
  sysid::TelemetryManager manager{{0.25_V / 1_s, 7_V, 4_V}, client_inst};

  // Begin the "slow-forward" test.
  manager.BeginTest("slow-forward");

  for (double i = 0.0; i < 10.0; i += 1.0) {
    nt::SetEntryValue(nt::GetEntry(server_inst, "/FMSInfo/FMSControlData"),
                      nt::Value::MakeDouble(1));

    nt::SetEntryValue(
        nt::GetEntry(server_inst, "/SmartDashboard/SysIdTelemetry"),
        nt::Value::MakeDoubleArray({i, i, i, i, i, i, i, i, i, i}));

    nt::Flush(server_inst);
    std::this_thread::sleep_for(std::chrono::milliseconds(6));
    manager.Update();
  }

  // Disable the robot.
  nt::SetEntryValue(nt::GetEntry(server_inst, "/FMSInfo/FMSControlData"),
                    nt::Value::MakeDouble(0));
  nt::Flush(server_inst);
  std::this_thread::sleep_for(std::chrono::milliseconds(6));
  manager.Update();

  const auto& data = manager.GetJSON();
  EXPECT_FALSE(data.dump().empty());
}
