#pragma once
#include <cstdint>
#include <string>

// A variable-speed drive (VFD) the way a Siemens PLC drives a SINAMICS:
// PROFIdrive standard telegram 1, two words each way.
//
//   PLC -> drive   STW1     control word    16#047E: ready, 16#047F: run
//                  NSOLL_A  speed setpoint  16#4000 = 100 % of the reference speed
//   drive -> PLC   ZSW1     status word     ready, running, fault, at speed, ...
//                  NIST_A   actual speed    same scale
//
// Inside, the PROFIdrive state machine decides whether the motor may run:
//
//   switching on inhibited --STW1.0=0--> ready to switch on --STW1.0=1--> switched on
//        ^                                      ^                            | STW1.3=1
//        |  OFF2 (coast) or OFF3 (quick stop)    | OFF1: ramp down to 0       v
//        +----------------------------------------+---------------- operation enabled
//   fault: from anywhere; the motor coasts; acknowledge with a rising STW1.7
//
// and a ramp-function generator takes the speed to the setpoint at the
// drive's acceleration. Speeds here are fractions of the reference speed
// (1.0 = 100 % = 16#4000); the World turns them into metres per second.
namespace fsim {

// STW1, the control word: the bits a PLC program sets.
namespace stw1 {
constexpr std::uint16_t kOn = 1U << 0;              // OFF1 when 0: ramp down and switch off
constexpr std::uint16_t kNoCoastStop = 1U << 1;     // OFF2 when 0: pulses off at once, the motor coasts
constexpr std::uint16_t kNoQuickStop = 1U << 2;     // OFF3 when 0: stop on the quick-stop ramp
constexpr std::uint16_t kEnableOperation = 1U << 3; // pulses enabled
constexpr std::uint16_t kRampEnable = 1U << 4;      // 0: the ramp output is forced to 0
constexpr std::uint16_t kRampStart = 1U << 5;       // 0: the ramp output is frozen
constexpr std::uint16_t kSetpointEnable = 1U << 6;  // 0: the setpoint is taken as 0
constexpr std::uint16_t kAcknowledge = 1U << 7;     // a rising edge acknowledges a fault
constexpr std::uint16_t kControlByPlc = 1U << 10;   // 0: the drive ignores the telegram
constexpr std::uint16_t kReady = 0x047E;            // the usual "switch on" word
constexpr std::uint16_t kRun = 0x047F;              // and "run"
} // namespace stw1

// ZSW1, the status word: what the drive tells the PLC.
namespace zsw1 {
constexpr std::uint16_t kReadyToSwitchOn = 1U << 0;
constexpr std::uint16_t kReadyToOperate = 1U << 1;
constexpr std::uint16_t kOperationEnabled = 1U << 2;
constexpr std::uint16_t kFault = 1U << 3;
constexpr std::uint16_t kNoCoastStop = 1U << 4;
constexpr std::uint16_t kNoQuickStop = 1U << 5;
constexpr std::uint16_t kSwitchingOnInhibited = 1U << 6;
constexpr std::uint16_t kAtSpeed = 1U << 8;          // actual speed within 2 % of the setpoint
constexpr std::uint16_t kControlRequested = 1U << 9; // "the PLC should control me": always set
constexpr std::uint16_t kNoLimit = 1U << 11;         // no current or torque limit reached
constexpr std::uint16_t kBrakeOpen = 1U << 12;       // with the pulses
constexpr std::uint16_t kNoMotorOvertemperature = 1U << 13;
constexpr std::uint16_t kForward = 1U << 14; // the motor turns forwards (or stands)
constexpr std::uint16_t kNoOverload = 1U << 15;
} // namespace zsw1

constexpr std::int16_t kFullScale = 0x4000; // 100 % in NSOLL_A and NIST_A

struct DriveSpec {
    bool fitted = false;         // the conveyor has a drive on the bus (else: a run output, or nothing)
    float referenceSpeed = 1.0F; // m/s at 100 % (16#4000): the drive's reference speed
    float quickStop = 5.0F;      // m/s², the OFF3 ramp
    float coast = 1.5F;          // m/s², how fast a loaded belt slows down with the motor off
    // The telegram's four words.
    std::string controlOutput;  // %QW: STW1
    std::string setpointOutput; // %QW: NSOLL_A
    std::string statusInput;    // %IW: ZSW1
    std::string actualInput;    // %IW: NIST_A

    bool operator==(const DriveSpec&) const = default;
};

class Drive {
public:
    enum class State { SwitchingOnInhibited, ReadyToSwitchOn, SwitchedOn, OperationEnabled, Fault };

    // What the PLC sent at a scan. A drive whose control word lacks
    // "control by PLC" ignores it (and faults if it was running: the PLC let go).
    void command(std::uint16_t controlWord, std::int16_t setpoint);

    // One step of `seconds`: the ramp moves the speed towards its target at
    // `acceleration`; OFF3 uses `quickStop`; with the motor off, the belt
    // coasts down at `coast` (all in fractions of the reference speed per second).
    void update(double seconds, double acceleration, double quickStop, double coast);

    // A fault, as the drive would raise it (overload, a blocked motor...).
    // Acknowledged by a rising edge of STW1.7.
    void trip(std::string cause);

    State state() const { return state_; }
    double speed() const { return speed_; } // fraction of the reference speed, negative backwards
    std::uint16_t statusWord() const;
    std::int16_t actualSpeed() const; // NIST_A
    const std::string& fault() const { return fault_; }
    std::uint16_t controlWord() const { return control_; }
    std::int16_t setpoint() const { return setpoint_; }

private:
    void step();           // follow the control word through the state machine
    double target() const; // what the ramp heads for now
    bool pulsesOn() const; // the motor is driven (not coasting)

    State state_ = State::SwitchingOnInhibited;
    std::uint16_t control_ = 0;
    std::int16_t setpoint_ = 0;
    double speed_ = 0.0;
    bool stopping_ = false; // OFF1 or OFF3: ramping down before switching off
    bool quick_ = false;    // the ramp down is OFF3's
    std::string fault_;
};

const char* stateName(Drive::State state);

} // namespace fsim
