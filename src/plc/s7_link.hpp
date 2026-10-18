#pragma once
#include "sim/control.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

// A live PLC controlling the simulated line over S7 (Siemens' protocol):
// soft_plc started with --s7, or a real S7 (rack 0, slot 1).
//
// Every scan, the simulator writes its input image to the PLC's inputs (%I,
// the PE area) and reads the PLC's outputs (%Q, the PA area) into its output
// image. The PLC runs on its own clock, so this only makes sense with the
// simulation in real time (time scale 1): unlike the lockstep link, the two
// cycles aren't aligned, and the timing jitters as it would on a real line.
//
// If the connection fails or drops, the outputs read as all off (the safe
// state), and it tries to connect again once a second.
namespace fsim {

class S7Link final : public ControlProgram {
public:
    // Exchanges the first `bytes` of each area (soft_plc on its own has 16;
    // a real S7 has far more).
    S7Link(std::string host, std::uint16_t port, std::size_t bytes = 16);
    ~S7Link() override;
    S7Link(const S7Link&) = delete;
    S7Link& operator=(const S7Link&) = delete;

    void scan(IoImage& io, double time) override;
    std::string name() const override;

    struct Status {
        bool connected = false;
        std::string error; // the last failure, if any
        std::uint64_t exchanges = 0;
        double lastRoundTripMicroseconds = 0.0; // write inputs + read outputs
    };
    Status status() const { return status_; }

private:
    bool connect();
    void fail(int code);

    std::string host_;
    std::uint16_t port_;
    std::size_t bytes_;
    std::uintptr_t client_ = 0; // Snap7's client handle (kept out of this header)
    std::chrono::steady_clock::time_point lastAttempt_{};
    Status status_;
};

} // namespace fsim
