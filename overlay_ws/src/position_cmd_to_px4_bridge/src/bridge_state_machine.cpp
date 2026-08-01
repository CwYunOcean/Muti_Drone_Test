#include "position_cmd_to_px4_bridge/bridge_state_machine.hpp"

#include <algorithm>

namespace position_cmd_to_px4_bridge
{

BridgeStateMachine::BridgeStateMachine(int warmup_cycles, bool auto_request_offboard_and_arm)
: warmup_cycles_(std::max(0, warmup_cycles)),
  auto_request_offboard_and_arm_(auto_request_offboard_and_arm)
{
}

BridgeOutputs BridgeStateMachine::tick(const BridgeInputs & inputs)
{
  BridgeOutputs outputs{};
  outputs.state = state_;

  if (!inputs.enabled) {
    state_ = BridgeState::IDLE;
    stream_cycles_ = 0;
    outputs.state = state_;
    return outputs;
  }

  if (state_ == BridgeState::ACTIVE && !inputs.fastlivo_fresh) {
    state_ = BridgeState::FAILSAFE;
    outputs.state = state_;
    outputs.request_land = true;
    return outputs;
  }

  switch (state_) {
    case BridgeState::IDLE:
      state_ = BridgeState::WAIT_FASTLIVO;
      break;
    case BridgeState::WAIT_FASTLIVO:
      if (inputs.fastlivo_fresh) {
        state_ = BridgeState::WAIT_PX4;
      }
      break;
    case BridgeState::WAIT_PX4:
      if (!inputs.fastlivo_fresh) {
        state_ = BridgeState::WAIT_FASTLIVO;
      } else if (inputs.px4_status_fresh && inputs.command_fresh) {
        state_ = BridgeState::STREAM_SETPOINT;
        stream_cycles_ = 0;
      }
      break;
    case BridgeState::STREAM_SETPOINT:
      if (!inputs.fastlivo_fresh) {
        state_ = BridgeState::WAIT_FASTLIVO;
        stream_cycles_ = 0;
      } else {
        outputs.stream_setpoint = true;
        ++stream_cycles_;
        if (stream_cycles_ >= warmup_cycles_) {
          if (auto_request_offboard_and_arm_) {
            state_ = BridgeState::ACTIVE;
            outputs.request_offboard = true;
            outputs.request_arm = true;
          } else if (inputs.px4_offboard && inputs.px4_armed) {
            state_ = BridgeState::ACTIVE;
          }
        }
      }
      break;
    case BridgeState::ENTER_OFFBOARD:
      if (!inputs.fastlivo_fresh) {
        state_ = BridgeState::WAIT_FASTLIVO;
        stream_cycles_ = 0;
      } else {
        outputs.stream_setpoint = true;
        if (inputs.px4_offboard) {
          state_ = BridgeState::ARM;
          outputs.request_arm = true;
        }
      }
      break;
    case BridgeState::ARM:
      if (!inputs.fastlivo_fresh) {
        state_ = BridgeState::WAIT_FASTLIVO;
        stream_cycles_ = 0;
      } else {
        outputs.stream_setpoint = true;
        if (inputs.px4_armed) {
          state_ = BridgeState::ACTIVE;
        }
      }
      break;
    case BridgeState::ACTIVE:
      outputs.stream_setpoint = true;
      if (!inputs.command_fresh) {
        state_ = BridgeState::FAILSAFE;
        outputs.request_land = true;
      }
      break;
    case BridgeState::FAILSAFE:
      outputs.request_land = true;
      break;
  }

  outputs.state = state_;
  return outputs;
}

}  // namespace position_cmd_to_px4_bridge
