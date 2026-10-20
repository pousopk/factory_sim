#include "plc/soft_plc_program.hpp"
#include "core/io_driver.hpp"
#include "core/scan_cycle.hpp"
#include "monitor/monitor_server.hpp"
#include "monitor/monitor_service.hpp"
#include "st/analyzer.hpp"
#include "st/diagnostic.hpp"
#include "st/st_program.hpp"
#include "st/value_text.hpp"
#include <algorithm>
#include <chrono>
#include <fmt/format.h>
#include <fstream>
#include <sstream>
#include <system_error>

namespace fsim {

namespace {

// soft_plc's I/O driver for the lockstep: its inputs come from the
// simulator's I/O image, its outputs go back to it. (A real soft_plc has a
// driver for hardware here; that's the seam the simulator plugs into.)
class LockstepIo final : public softplc::IoDriver {
public:
    IoImage* io = nullptr; // the image of the scan in progress

    void readInputs(softplc::ProcessImage& image) override {
        auto inputs = image.bytes(softplc::Area::Input);
        std::copy_n(io->inputs().begin(), std::min(inputs.size(), io->inputs().size()), inputs.begin());
    }
    void writeOutputs(const softplc::ProcessImage& image) override {
        const auto outputs = image.bytes(softplc::Area::Output);
        std::copy_n(outputs.begin(), std::min(outputs.size(), io->outputs().size()), io->outputs().begin());
    }
};

// Rounded, not truncated (duration_cast): 8.19 s as a double is
// 8.1899999999999995, which cut down to whole nanoseconds is 1 ns short. A
// 150 ms timer started at 8.04 s then sees 149.999999 ms at 8.19 s, and
// ends a scan late.
softplc::TimePoint simulatedTime(double seconds) {
    return softplc::TimePoint{} + std::chrono::round<softplc::Duration>(std::chrono::duration<double>(seconds));
}

} // namespace

struct SoftPlcProgram::State {
    std::string source;
    softplc::ProcessImage image{IoImage::kBytes, IoImage::kBytes, 256};
    LockstepIo io;
    std::unique_ptr<softplc::st::StProgram> program;
    std::unique_ptr<softplc::ScanCycle> cycle;
    std::unique_ptr<softplc::monitor::MonitorServer> server;
    std::unique_ptr<softplc::monitor::MonitorService> service;
};

tl::expected<std::unique_ptr<SoftPlcProgram>, std::string> SoftPlcProgram::fromSource(std::string_view source,
                                                                                      std::uint16_t monitorPort) {
    auto compiled = softplc::st::compileProgram(source);
    if (!compiled) {
        return tl::unexpected(softplc::st::formatDiagnostic(compiled.error(), source));
    }
    auto state = std::make_unique<State>();
    state->source = std::string(source);
    for (const auto& [number, size] : softplc::st::dataBlocksUsedBy(*compiled)) {
        state->image.addDataBlock(number, size);
    }
    // On a runtime error the program goes to STOP (outputs off) and stays
    // there, as a PLC does; the status says why.
    state->program = std::make_unique<softplc::st::StProgram>(std::move(*compiled));
    state->cycle =
        std::make_unique<softplc::ScanCycle>(state->image, state->io, *state->program, std::chrono::milliseconds(10));
    if (monitorPort != 0) {
        try {
            state->server = std::make_unique<softplc::monitor::MonitorServer>(monitorPort);
        } catch (const std::system_error& error) {
            return tl::unexpected(fmt::format("can't listen for monitoring on port {}: {}", monitorPort, error.what()));
        }
        state->service = std::make_unique<softplc::monitor::MonitorService>(*state->program, state->image,
                                                                            *state->cycle, state->source);
    }
    return std::unique_ptr<SoftPlcProgram>(new SoftPlcProgram(std::move(state)));
}

tl::expected<std::unique_ptr<SoftPlcProgram>, std::string> SoftPlcProgram::fromFile(const std::filesystem::path& file,
                                                                                    std::uint16_t monitorPort) {
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        return tl::unexpected("can't read " + file.string());
    }
    std::stringstream text;
    text << in.rdbuf();
    auto program = fromSource(text.str(), monitorPort);
    if (!program) {
        return tl::unexpected(file.filename().string() + ": " + program.error());
    }
    return program;
}

SoftPlcProgram::SoftPlcProgram(std::unique_ptr<State> state) : state_(std::move(state)) {}

SoftPlcProgram::~SoftPlcProgram() = default;

void SoftPlcProgram::scan(IoImage& io, double time) {
    state_->io.io = &io;
    state_->cycle->runOnce(simulatedTime(time)); // inputs in, one cycle, outputs out
    state_->io.io = nullptr;
    // Between cycles: answer monitoring requests (values, forcing, LOAD).
    if (state_->server) {
        state_->server->processPending([this](std::string_view request) { return state_->service->handle(request); });
    }
}

SoftPlcProgram::Status SoftPlcProgram::status() const {
    Status status;
    status.running = !state_->program->stopped();
    if (const auto& error = state_->program->stopError()) {
        status.error =
            fmt::format("line {}, column {}: {}", error->location.line, error->location.column, error->message);
    }
    status.cycles = state_->cycle->stats().cycles;
    status.lastCycleMicroseconds =
        std::chrono::duration<double, std::micro>(state_->cycle->stats().lastExecution).count();
    status.monitorPort = state_->server ? state_->server->port() : 0;
    return status;
}

std::vector<SoftPlcProgram::Variable> SoftPlcProgram::variables() const {
    std::vector<Variable> result;
    const auto& interpreter = state_->program->interpreter();
    for (const auto& variable : interpreter.variables()) {
        const auto value = interpreter.valueOf(variable.path, state_->image);
        result.push_back(Variable{variable.path, softplc::st::dataTypeName(variable.type),
                                  value ? softplc::st::formatValue(*value, variable.type) : "?"});
    }
    return result;
}

} // namespace fsim
