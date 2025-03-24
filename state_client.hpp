#ifndef KBD_SM_STATE_CLIENT_HPP
#define KBD_SM_STATE_CLIENT_HPP
#include "c_sdk.h"
#include "struct_common.h"

namespace kbd {
namespace sm {

class StateClient final {
 public:
  StateClient() : last_state_(BUT_S), is_read_env_state_machine_(false), is_open_(false) {}

  ~StateClient() = default;

  bool parking_init() {
    if (!is_open_state_machine_()) {
      return false;
    }
    auto state = get_state_();
    if (state == PARKING_S && last_state_ != state) {
      last_state_ = state;
      return true;
    }
    if (last_state_ != state) {
      last_state_ = state;
    }
    return false;
  }

  bool driving_init() {
    if (!is_open_state_machine_()) {
      return false;
    }
    auto state = get_state_();
    if (state == DRIVING_S && last_state_ != state) {
      last_state_ = state;
      return true;
    }

    if (last_state_ != state) {
      last_state_ = state;
    }
    return false;
  }

  bool parking_stop() {
    if (!is_open_state_machine_()) {
      return false;
    }
    auto state = get_state_();
    return state != PARKING_S;
  }

  bool driving_stop() {
    if (!is_open_state_machine_()) {
      return false;
    }
    auto state = get_state_();
    return state != DRIVING_S;
  }

 private:
  D_P_STATE_MACHINE get_state_() {
    S2S_MCore_Bridge_GetSigStateMachine_Output(&state_machine_);
    return state_machine_.stateMachine;
  }

  bool is_open_state_machine_() {
    if (!is_read_env_state_machine_) {
      is_read_env_state_machine_ = true;
      auto state = C_cSystem_GetEnvVariable("ADAS_STATE_MACHINE_SWITCH");
      if (nullptr == state) {
        state = "ON";
      }

      // LOGW("ADAS_STATE_MACHINE_SWITCH: %s", state);
      if (state == "ON") {
        is_open_ = true;
      } else {
        is_open_ = false;
      }
    }
    return is_open_;
  }

 private:
  StateMachine_Output state_machine_;
  D_P_STATE_MACHINE last_state_;
  bool is_read_env_state_machine_;
  bool is_open_;
};

}  // namespace sm
}  // namespace kbd

#endif  // KBD_SM_STATE_CLIENT_HPP