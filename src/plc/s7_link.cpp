#include "plc/s7_link.hpp"
#include <algorithm>
#include <fmt/format.h>
#include <snap7.h>

namespace fsim {

namespace {

S7Object handle(std::uintptr_t client) {
    return static_cast<S7Object>(client);
}

constexpr int kRack = 0;
constexpr int kSlot = 1; // a soft_plc, or an S7-300/400's CPU; an S7-1200/1500 also answers on slot 1

} // namespace

S7Link::S7Link(std::string host, std::uint16_t port, std::size_t bytes)
    : host_(std::move(host)), port_(port), bytes_(std::min(bytes, IoImage::kBytes)) {
    client_ = static_cast<std::uintptr_t>(Cli_Create());
}

S7Link::~S7Link() {
    auto client = handle(client_);
    Cli_Disconnect(client);
    Cli_Destroy(&client);
}

std::string S7Link::name() const {
    return fmt::format("S7 PLC at {}:{}", host_, port_);
}

bool S7Link::connect() {
    const auto now = std::chrono::steady_clock::now();
    if (now - lastAttempt_ < std::chrono::seconds(1)) {
        return false; // not again so soon: connecting blocks, and a dead PLC would freeze every scan
    }
    lastAttempt_ = now;
    auto client = handle(client_);
    std::uint16_t port = port_;
    Cli_SetParam(client, p_u16_RemotePort, &port);
    if (const int code = Cli_ConnectTo(client, host_.c_str(), kRack, kSlot); code != 0) {
        fail(code);
        return false;
    }
    status_.connected = true;
    status_.error.clear();
    return true;
}

void S7Link::fail(int code) {
    char text[256] = {};
    Cli_ErrorText(code, text, sizeof text);
    status_.error = text;
    status_.connected = false;
    Cli_Disconnect(handle(client_));
}

void S7Link::scan(IoImage& io, double /*time*/) {
    if (!status_.connected && !connect()) {
        io.outputs().fill(0); // no PLC: everything off
        return;
    }
    const auto start = std::chrono::steady_clock::now();
    auto client = handle(client_);
    if (const int code = Cli_EBWrite(client, 0, static_cast<int>(bytes_), io.inputs().data()); code != 0) {
        fail(code);
        io.outputs().fill(0);
        return;
    }
    if (const int code = Cli_ABRead(client, 0, static_cast<int>(bytes_), io.outputs().data()); code != 0) {
        fail(code);
        io.outputs().fill(0);
        return;
    }
    ++status_.exchanges;
    status_.lastRoundTripMicroseconds =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
}

} // namespace fsim
