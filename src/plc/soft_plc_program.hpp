#pragma once
#include "sim/control.hpp"
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <tl/expected.hpp>
#include <vector>

// soft_plc, controlling the simulated line in lockstep.
//
// Each scan of the World is one cycle of soft_plc: the I/O image's input
// bytes are copied into soft_plc's process image, the ST program runs once
// with the *simulated* time as its clock, and the output bytes are copied
// back. So a TON in the program times simulated seconds: the line can run
// at 0.1x or 10x real time, or be single-stepped, and the program keeps up
// exactly, with the same result every run.
//
// It can also listen for monitoring connections (soft_plc's protocol), so
// ladder_editor can go online with the program running inside the
// simulator: power flow, values, forcing, and Download (an online change).
// Requests are answered between cycles, as in soft_plc itself.
namespace fsim {

class SoftPlcProgram final : public ControlProgram {
public:
    // Compiles `source` (Structured Text, e.g. ladder_editor's Export ST).
    // With a `monitorPort`, it accepts monitoring connections on it.
    static tl::expected<std::unique_ptr<SoftPlcProgram>, std::string> fromSource(std::string_view source,
                                                                                 std::uint16_t monitorPort = 0);
    static tl::expected<std::unique_ptr<SoftPlcProgram>, std::string> fromFile(const std::filesystem::path& file,
                                                                               std::uint16_t monitorPort = 0);
    ~SoftPlcProgram() override;

    void scan(IoImage& io, double time) override;
    std::string name() const override { return "soft_plc"; }

    struct Status {
        bool running = true; // false: STOP after a runtime error
        std::string error;   // why it stopped
        std::uint64_t cycles = 0;
        double lastCycleMicroseconds = 0.0; // real time the last cycle took
        std::uint16_t monitorPort = 0;      // 0: not listening
    };
    Status status() const;

    // Every variable of the program, with its value now, as soft_plc writes it.
    struct Variable {
        std::string path;
        std::string type;
        std::string value;
    };
    std::vector<Variable> variables() const;

private:
    struct State;
    explicit SoftPlcProgram(std::unique_ptr<State> state);
    std::unique_ptr<State> state_; // soft_plc's objects, kept out of this header
};

} // namespace fsim
