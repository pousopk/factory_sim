#include "analysis/analysis.hpp"
#include <algorithm>
#include <cmath>
#include <fmt/format.h>

namespace fsim {

std::vector<Trace> traces(const World& world) {
    std::vector<Trace> result;
    for (const auto& conveyor : world.conveyors()) {
        result.push_back(Trace{conveyor.spec.name + " motor", conveyor.spec.runOutput, &conveyor.motor});
    }
    for (const auto& photocell : world.photocells()) {
        result.push_back(Trace{photocell.spec.name + (photocell.spec.onlyNok ? " inspection" : " photocell"),
                               photocell.spec.input, &photocell.output});
    }
    const auto actuator = [&](const auto& a) {
        result.push_back(Trace{a.spec.name + " valve", a.spec.valveOutput, &a.stroke.valve()});
        result.push_back(Trace{a.spec.name + " extended", a.spec.extendedInput, &a.stroke.extended()});
        result.push_back(Trace{a.spec.name + " retracted", a.spec.retractedInput, &a.stroke.retracted()});
    };
    std::for_each(world.pushers().begin(), world.pushers().end(), actuator);
    std::for_each(world.diverters().begin(), world.diverters().end(), actuator);
    std::for_each(world.stoppers().begin(), world.stoppers().end(), actuator);
    for (const auto& camera : world.cameras()) {
        result.push_back(Trace{camera.spec.name + " trigger", camera.spec.triggerOutput, &camera.trigger});
        result.push_back(Trace{camera.spec.name + " busy", "", &camera.busy}); // its ready input is the inverse
    }
    for (std::size_t i = 0; i < world.buttons().size(); ++i) {
        result.push_back(
            Trace{world.buttons()[i].name + " button", world.buttons()[i].address, &world.buttonSignal(i)});
    }
    for (std::size_t i = 0; i < world.lamps().size(); ++i) {
        result.push_back(Trace{world.lamps()[i].name + " lamp", world.lamps()[i].address, &world.lampSignal(i)});
    }
    return result;
}

Steps steps(const Signal& signal, double from, double to) {
    Steps result;
    bool value = signal.valueAt(from);
    result.times.push_back(from);
    result.values.push_back(value ? 1.0 : 0.0);
    for (const auto& edge : signal.edges()) {
        if (edge.time <= from || edge.time > to || edge.value == value) {
            continue;
        }
        // A vertical step: the old value and the new one, at the same time.
        result.times.push_back(edge.time);
        result.values.push_back(value ? 1.0 : 0.0);
        value = edge.value;
        result.times.push_back(edge.time);
        result.values.push_back(value ? 1.0 : 0.0);
    }
    result.times.push_back(to);
    result.values.push_back(value ? 1.0 : 0.0);
    return result;
}

std::vector<Event> eventLog(const std::vector<Trace>& traces) {
    std::vector<Event> events;
    for (const auto& trace : traces) {
        for (const auto& edge : trace.signal->edges()) {
            events.push_back(Event{edge.time, trace.name, trace.address, edge.value});
        }
    }
    // Stable: switches at the same instant keep the traces' order.
    std::stable_sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.time < b.time; });
    return events;
}

std::string eventsCsv(const std::vector<Event>& events) {
    std::string csv = "time_s,signal,address,value\n";
    for (const auto& event : events) {
        // Names are the user's: quote them (doubling any quote inside) in
        // case they hold a comma.
        std::string name = event.signal;
        for (std::size_t at = name.find('"'); at != std::string::npos; at = name.find('"', at + 2)) {
            name.insert(at, 1, '"');
        }
        csv += fmt::format("{:.6f},\"{}\",{},{}\n", event.time, name, event.address, event.value ? 1 : 0);
    }
    return csv;
}

Statistics statistics(std::span<const double> values) {
    Statistics s;
    s.count = values.size();
    if (values.empty()) {
        return s;
    }
    double sum = 0.0;
    s.min = values.front();
    s.max = values.front();
    for (const double v : values) {
        sum += v;
        s.min = std::min(s.min, v);
        s.max = std::max(s.max, v);
    }
    s.mean = sum / static_cast<double>(values.size());
    double squares = 0.0;
    for (const double v : values) {
        squares += (v - s.mean) * (v - s.mean);
    }
    s.deviation = std::sqrt(squares / static_cast<double>(values.size()));
    return s;
}

std::vector<double> positionErrors(const Photocell& photocell) {
    std::vector<double> errors;
    for (const auto& trigger : photocell.triggers) {
        errors.push_back(trigger.positionError * 1000.0);
    }
    return errors;
}

std::vector<double> positionErrorsAtScan(const Photocell& photocell, double scanSeconds) {
    std::vector<double> errors;
    for (const auto& trigger : photocell.triggers) {
        // The first scan starting at or after the output time.
        const double seen = std::ceil(trigger.outputTime / scanSeconds - 1e-9) * scanSeconds;
        errors.push_back((trigger.positionError + trigger.speed * (seen - trigger.outputTime)) * 1000.0);
    }
    return errors;
}

} // namespace fsim
