#pragma once

namespace position_cmd_to_px4_bridge
{

enum class BridgeState
{
  IDLE,
  WAIT_FASTLIVO,
  WAIT_PX4,
  STREAM_SETPOINT,
  ENTER_OFFBOARD,
  ARM,
  ACTIVE,
  FAILSAFE
};

struct BridgeInputs
{
  bool enabled{false};
  bool fastlivo_fresh{false};
  bool px4_status_fresh{false};
  bool command_fresh{false};
  bool px4_offboard{false};
  bool px4_armed{false};
};

struct BridgeOutputs
{
  BridgeState state{BridgeState::IDLE};
  bool stream_setpoint{false};
  bool request_offboard{false};
  bool request_arm{false};
  bool request_land{false};
};

class BridgeStateMachine
{
public:
  explicit BridgeStateMachine(int warmup_cycles, bool auto_request_offboard_and_arm = true);

  BridgeOutputs tick(const BridgeInputs & inputs);

private:
  int warmup_cycles_{10};
  bool auto_request_offboard_and_arm_{true};
  int stream_cycles_{0};
  BridgeState state_{BridgeState::IDLE};
};

}  // namespace position_cmd_to_px4_bridge
