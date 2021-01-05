// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <ntcore_cpp.h>
#include <wpi/raw_ostream.h>

#include "gtest/gtest.h"
#include "sysid/telemetry/TelemetryData.h"
#include "sysid/telemetry/TelemetryLogger.h"

class TelemetryLoggerTest : public ::testing::Test {
 public:
  TelemetryLoggerTest()
      : server_inst(nt::CreateInstance()), client_inst(nt::CreateInstance()) {
    nt::SetNetworkIdentity(server_inst, "server");
    nt::SetNetworkIdentity(client_inst, "client");
  }

  ~TelemetryLoggerTest() override {
    nt::DestroyInstance(server_inst);
    nt::DestroyInstance(client_inst);
  }

  void Connect();

 protected:
  NT_Inst server_inst;
  NT_Inst client_inst;
};

void TelemetryLoggerTest::Connect() {
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

TEST_F(TelemetryLoggerTest, Data) {
  Connect();
  if (HasFatalFailure()) {
    return;
  }

  // Create the logger and start it.
  sysid::TelemetryLogger logger{[] { return 0.0; }, client_inst};

  for (double i = 0.0; i < 10.0; i += 1.0) {
    nt::SetEntryValue(nt::GetEntry(server_inst, "/FMSInfo/FMSControlData"),
                      nt::Value::MakeDouble(1));

    nt::SetEntryValue(
        nt::GetEntry(server_inst, "/SmartDashboard/SysIdTelemetry"),
        nt::Value::MakeDoubleArray({i, i, i, i, i, i, i, i, i, i}));

    nt::Flush(server_inst);
    std::this_thread::sleep_for(std::chrono::milliseconds(6));
    logger.Update();
  }

  auto data = logger.Cancel();
  EXPECT_EQ(data.size(), 10u);
}

TEST_F(TelemetryLoggerTest, MalformedData) {
  Connect();
  if (HasFatalFailure()) {
    return;
  }

  // Create the logger and start it.
  sysid::TelemetryLogger logger{[] { return 0.0; }, client_inst};

  for (double i = 0.0; i < 10.0; i += 1.0) {
    nt::SetEntryValue(nt::GetEntry(server_inst, "/FMSInfo/FMSControlData"),
                      nt::Value::MakeDouble(1));

    nt::SetEntryValue(
        nt::GetEntry(server_inst, "/SmartDashboard/SysIdTelemetry"),
        nt::Value::MakeDoubleArray({i, i, i, i, i, i, i, i, i}));

    nt::Flush(server_inst);
    std::this_thread::sleep_for(std::chrono::milliseconds(6));
    logger.Update();
  }

  auto data = logger.Cancel();
  EXPECT_EQ(data.size(), 0u);
}
