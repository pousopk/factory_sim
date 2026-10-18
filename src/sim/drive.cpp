#include "sim/drive.hpp"
#include <algorithm>
#include <cmath>

namespace fsim {

namespace {

bool has(std::uint16_t word, std::uint16_t bit) {
    return (word & bit) != 0;
}

} // namespace

const char* stateName(Drive::State state) {
    switch (state) {
    case Drive::State::SwitchingOnInhibited:
        return "switching on inhibited";
    case Drive::State::ReadyToSwitchOn:
        return "ready to switch on";
    case Drive::State::SwitchedOn:
        return "switched on";
    case Drive::State::OperationEnabled:
        return "operation enabled";
    case Drive::State::Fault:
        return "fault";
    }
    return "?";
}

void Drive::command(std::uint16_t controlWord, std::int16_t setpoint) {
    const bool acknowledge = has(controlWord, stw1::kAcknowledge) && !has(control_, stw1::kAcknowledge);
    if (!has(controlWord, stw1::kControlByPlc)) {
        // The PLC isn't in control: the telegram means nothing. A drive that
        // was running can't just carry on with nobody in charge.
        if (state_ == State::OperationEnabled) {
            trip("F07220: control by PLC missing (STW1.10 went off while running)");
        }
        control_ = controlWord; // still remembered, for the next acknowledge edge
        return;
    }
    control_ = controlWord;
    setpoint_ = setpoint;
    if (state_ == State::Fault) {
        if (acknowledge) {
            fault_.clear();
            state_ = State::SwitchingOnInhibited;
        } else {
            return;
        }
    }
    step();
}

// The state machine, followed as far as the control word allows: 16#047E
// then 16#047F goes all the way from "inhibited" to "operation enabled".
void Drive::step() {
    const bool off2 = !has(control_, stw1::kNoCoastStop);
    const bool off3 = !has(control_, stw1::kNoQuickStop);
    const bool on = has(control_, stw1::kOn);
    const bool enable = has(control_, stw1::kEnableOperation);
    for (int guard = 0; guard < 5; ++guard) {
        const auto before = state_;
        // OFF2 and OFF3 win in any state: the motor coasts, or stops on the
        // quick-stop ramp, and the drive goes back to "inhibited".
        if (off2 && state_ != State::SwitchingOnInhibited) {
            state_ = State::SwitchingOnInhibited;
            stopping_ = false;
        } else if (off3 && state_ != State::SwitchingOnInhibited) {
            if (state_ == State::OperationEnabled && std::abs(speed_) > 0.0) {
                stopping_ = true;
                quick_ = true;
            } else {
                state_ = State::SwitchingOnInhibited;
                stopping_ = false;
            }
        } else {
            switch (state_) {
            case State::SwitchingOnInhibited:
                if (!on && !off2 && !off3) {
                    state_ = State::ReadyToSwitchOn; // OFF1 first: an "on" left over from before doesn't start it
                }
                break;
            case State::ReadyToSwitchOn:
                if (on) {
                    state_ = State::SwitchedOn;
                }
                break;
            case State::SwitchedOn:
                if (!on) {
                    state_ = State::ReadyToSwitchOn;
                } else if (enable) {
                    state_ = State::OperationEnabled;
                }
                break;
            case State::OperationEnabled:
                if (!on) {
                    stopping_ = true; // OFF1: down the normal ramp, then off
                    quick_ = false;
                } else if (!enable) {
                    state_ = State::SwitchedOn; // pulses off: the motor coasts
                } else {
                    stopping_ = false;
                }
                break;
            case State::Fault:
                break;
            }
        }
        if (state_ == before) {
            break;
        }
    }
}

bool Drive::pulsesOn() const {
    return state_ == State::OperationEnabled;
}

double Drive::target() const {
    if (stopping_ || !has(control_, stw1::kSetpointEnable)) {
        return 0.0;
    }
    return static_cast<double>(setpoint_) / kFullScale;
}

void Drive::update(double seconds, double acceleration, double quickStop, double coast) {
    if (!pulsesOn()) {
        // No torque: the belt and its load slow down by themselves.
        const double change = coast * seconds;
        speed_ = std::abs(speed_) <= change ? 0.0 : speed_ - std::copysign(change, speed_);
        return;
    }
    double goal = target();
    double rate = stopping_ && quick_ ? quickStop : acceleration;
    if (!has(control_, stw1::kRampEnable)) {
        goal = 0.0; // the ramp's output held at 0: braking as fast as allowed
        rate = quickStop;
    } else if (!has(control_, stw1::kRampStart) && !stopping_) {
        goal = speed_; // the ramp frozen where it is
    }
    const double change = rate * seconds;
    speed_ += std::clamp(goal - speed_, -change, change);
    if (stopping_ && speed_ == 0.0) {
        // Down to standstill: OFF1 switches off, OFF3 inhibits.
        stopping_ = false;
        state_ = quick_ ? State::SwitchingOnInhibited : State::ReadyToSwitchOn;
        quick_ = false;
    }
}

void Drive::trip(std::string cause) {
    state_ = State::Fault;
    stopping_ = false;
    fault_ = std::move(cause);
}

std::int16_t Drive::actualSpeed() const {
    const double scaled = std::round(speed_ * kFullScale);
    return static_cast<std::int16_t>(std::clamp(scaled, -32768.0, 32767.0));
}

std::uint16_t Drive::statusWord() const {
    std::uint16_t word = zsw1::kControlRequested | zsw1::kNoLimit | zsw1::kNoMotorOvertemperature;
    switch (state_) {
    case State::OperationEnabled:
        word |= zsw1::kOperationEnabled | zsw1::kBrakeOpen;
        [[fallthrough]];
    case State::SwitchedOn:
        word |= zsw1::kReadyToOperate;
        [[fallthrough]];
    case State::ReadyToSwitchOn:
        word |= zsw1::kReadyToSwitchOn;
        break;
    case State::SwitchingOnInhibited:
        word |= zsw1::kSwitchingOnInhibited;
        break;
    case State::Fault:
        word |= zsw1::kFault;
        break;
    }
    if (has(control_, stw1::kNoCoastStop)) {
        word |= zsw1::kNoCoastStop;
    }
    if (has(control_, stw1::kNoQuickStop)) {
        word |= zsw1::kNoQuickStop;
    }
    if (state_ != State::Fault) {
        word |= zsw1::kNoOverload;
    }
    if (speed_ >= 0.0) {
        word |= zsw1::kForward;
    }
    if (pulsesOn() && std::abs(speed_ - target()) <= 0.02) {
        word |= zsw1::kAtSpeed;
    }
    return word;
}

} // namespace fsim
