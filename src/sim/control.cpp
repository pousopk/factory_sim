#include "sim/control.hpp"

namespace fsim {

BuiltInSorting::BuiltInSorting(std::vector<SorterSpec> sorters) {
    for (auto& spec : sorters) {
        sorters_.push_back(Sorter{std::move(spec), false, false, {}, 0});
    }
}

// The same logic a PLC program would have, scan by scan:
//
//   NOK sensor's rising edge   ->  remember "fire at count + distance"
//   count has reached the first remembered value  ->  valve on
//   valve on and pusher extended                  ->  valve off
//
// The encoder count is a 32-bit counter that wraps around, so "has it
// reached" is decided on the difference, read as signed: it works across
// the wrap, as long as a box is never more than 2^31 pulses away.
void BuiltInSorting::scan(IoImage& io, double /*time*/) {
    for (auto& sorter : sorters_) {
        const auto nokAddress = parseAddress(sorter.spec.nokInput);
        const auto countAddress = parseAddress(sorter.spec.encoderInput);
        const auto valveAddress = parseAddress(sorter.spec.valveOutput);
        const auto extendedAddress = parseAddress(sorter.spec.extendedInput);
        if (!nokAddress || !countAddress || !valveAddress || !extendedAddress) {
            continue; // not set up: does nothing, as a PLC program with no I/O would
        }
        const bool nok = io.bit(*nokAddress);
        const auto count = io.value(*countAddress);
        if (nok && !sorter.lastNok) {
            sorter.firingCounts.push_back(count + static_cast<std::uint32_t>(sorter.spec.distance));
        }
        sorter.lastNok = nok;
        if (!sorter.firingCounts.empty() && static_cast<std::int32_t>(count - sorter.firingCounts.front()) >= 0) {
            sorter.firingCounts.pop_front();
            sorter.valve = true;
            ++sorter.fired;
        }
        if (sorter.valve && io.bit(*extendedAddress)) {
            sorter.valve = false;
        }
        io.setBit(*valveAddress, sorter.valve);
    }
}

std::vector<BuiltInSorting::Status> BuiltInSorting::status() const {
    std::vector<Status> result;
    for (const auto& sorter : sorters_) {
        result.push_back(Status{sorter.fired, sorter.firingCounts.size()});
    }
    return result;
}

} // namespace fsim
