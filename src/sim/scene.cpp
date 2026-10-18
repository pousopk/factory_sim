#include "sim/scene.hpp"
#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <numbers>
#include <sstream>

namespace fsim {

namespace {

using json = nlohmann::json;

constexpr int kFormat = 1;
constexpr float kQuarterTurn = std::numbers::pi_v<float> / 2.0F;

// The first free name of the form C1, C2, ... (or B1, B2, ...).
template <class Items, class NameOf> std::string freeName(const Items& items, const char* prefix, NameOf nameOf) {
    for (int n = 1;; ++n) {
        const auto name = fmt::format("{}{}", prefix, n);
        if (std::none_of(items.begin(), items.end(), [&](const auto& item) { return nameOf(item) == name; })) {
            return name;
        }
    }
}

// Indices that point into a list where element `removed` is going away:
// those after it move down by one.
void shiftDown(std::size_t& index, std::size_t removed) {
    if (index > removed) {
        --index;
    }
}

// Everything in `items` that stood on conveyor `removed` goes; the rest
// refers to its conveyor by an index that may shift.
template <class Items> void dropFrom(Items& items, std::size_t removed) {
    std::erase_if(items, [&](const auto& item) { return item.conveyor == removed; });
    for (auto& item : items) {
        shiftDown(item.conveyor, removed);
    }
}

template <class Items> void eraseAt(Items& items, std::size_t index) {
    if (index < items.size()) {
        items.erase(items.begin() + static_cast<std::ptrdiff_t>(index));
    }
}

// What of an actuator is its place (the layout), with its name and timing
// taken out, so two of them compare equal when only those differ.
template <class Spec> Spec placementOf(Spec spec) {
    spec.name.clear();
    if constexpr (requires { spec.stroke; }) {
        spec.stroke = StrokeSpec{};
    }
    return spec;
}

template <class Specs> bool samePlaces(const Specs& a, const Specs& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](const auto& x, const auto& y) {
               return placementOf(x) == placementOf(y);
           });
}

} // namespace

Scene defaultScene() {
    Scene scene;
    PlacedConveyor conveyor;
    conveyor.spec.name = "C1";
    conveyor.spec.length = 3.0F;
    conveyor.spec.targetSpeed = 0.5F;
    scene.conveyors.push_back(conveyor);
    SpawnerSpec spawner;
    spawner.interval = 1.2;
    scene.spawners.push_back(spawner);
    PhotocellSpec photocell;
    photocell.name = "B1";
    photocell.along = 1.5F;
    photocell.input = "%IX0.0";
    scene.photocells.push_back(photocell);
    scene.conveyors[0].spec.encoderInput = "%ID8";
    return scene;
}

Scene sortingDemo() {
    Scene scene;
    PlacedConveyor conveyor;
    conveyor.spec.name = "C1";
    conveyor.spec.length = 6.0F;
    conveyor.spec.targetSpeed = 0.5F;
    conveyor.spec.encoderInput = "%ID8";
    scene.conveyors.push_back(conveyor);
    SpawnerSpec spawner;
    spawner.interval = 1.0;
    spawner.intervalJitter = 0.2;
    spawner.nokFraction = 0.3F;
    scene.spawners.push_back(spawner);
    PhotocellSpec inspection;
    inspection.name = "BQ";
    inspection.along = 1.5F;
    inspection.onlyNok = true;
    inspection.input = "%IX0.0";
    scene.photocells.push_back(inspection);
    PusherSpec pusher;
    pusher.name = "P1";
    pusher.along = 3.5F;
    pusher.valveOutput = "%QX0.0";
    pusher.extendedInput = "%IX0.1";
    pusher.retractedInput = "%IX0.2";
    scene.pushers.push_back(pusher);
    addBin(scene, 0); // across from the pusher
    scene.sorters.push_back(sorterFor(scene, 0, 0));
    scene.lamps.push_back(LampSpec{"Reject", "%QX0.0"});
    scene.control = Control::BuiltInSorting;
    return scene;
}

bool sameControl(const Scene& a, const Scene& b) {
    return a.control == b.control && a.sorters == b.sorters && a.plcProgram == b.plcProgram &&
           a.monitorPort == b.monitorPort && a.s7Host == b.s7Host && a.s7Port == b.s7Port;
}

Scene conveyorDemo(std::string program) {
    Scene scene;
    PlacedConveyor conveyor;
    conveyor.spec.name = "C1";
    conveyor.spec.length = 4.0F;
    conveyor.spec.targetSpeed = 0.5F;
    conveyor.spec.runOutput = "%QX0.0"; // the program's motor output
    conveyor.spec.encoderInput = "%ID8";
    scene.conveyors.push_back(conveyor);
    SpawnerSpec spawner;
    spawner.interval = 2.5;
    scene.spawners.push_back(spawner);
    PhotocellSpec atStart;
    atStart.name = "boxAtStart";
    atStart.along = 0.6F;
    atStart.input = "%IX0.3";
    scene.photocells.push_back(atStart);
    PhotocellSpec atEnd;
    atEnd.name = "boxAtEnd";
    atEnd.along = 3.7F;
    atEnd.input = "%IX0.4";
    scene.photocells.push_back(atEnd);
    scene.buttons = {
        ButtonSpec{"Start", "%IX0.0", false, false},
        ButtonSpec{"Stop", "%IX0.1", true, false},  // wired normally closed
        ButtonSpec{"E-stop", "%IX0.2", true, true}, // normally closed, and it stays pressed
        ButtonSpec{"Reset", "%IX0.5", false, false},
    };
    scene.lamps = {
        LampSpec{"Motor", "%QX0.0"},
        LampSpec{"Jam", "%QX0.1"},
        LampSpec{"Batch full", "%QX0.2"},
        LampSpec{"Too fast", "%QX0.3"},
    };
    scene.control = Control::SoftPlc;
    scene.plcProgram = std::move(program);
    return scene;
}

std::unique_ptr<ControlProgram> makeProgram(const Scene& scene) {
    if (scene.control == Control::BuiltInSorting) {
        return std::make_unique<BuiltInSorting>(scene.sorters);
    }
    return nullptr;
}

namespace {

// Every address the scene's devices, buttons and lamps use.
std::vector<IoAddress> usedAddresses(const Scene& scene) {
    std::vector<IoAddress> used;
    const auto add = [&](const std::string& text) {
        if (const auto address = parseAddress(text)) {
            used.push_back(*address);
        }
    };
    for (const auto& c : scene.conveyors) {
        add(c.spec.runOutput);
        add(c.spec.encoderInput);
    }
    for (const auto& p : scene.photocells) {
        add(p.input);
    }
    const auto actuator = [&](const auto& a) {
        add(a.valveOutput);
        add(a.extendedInput);
        add(a.retractedInput);
    };
    std::for_each(scene.pushers.begin(), scene.pushers.end(), actuator);
    std::for_each(scene.diverters.begin(), scene.diverters.end(), actuator);
    std::for_each(scene.stoppers.begin(), scene.stoppers.end(), actuator);
    for (const auto& b : scene.buttons) {
        add(b.address);
    }
    for (const auto& l : scene.lamps) {
        add(l.address);
    }
    return used;
}

} // namespace

std::string freeAddress(const Scene& scene, IoAddress::Area area, IoAddress::Size size) {
    const auto used = usedAddresses(scene);
    const auto taken = [&](std::size_t byte, std::size_t width, std::optional<unsigned> bit) {
        return std::any_of(used.begin(), used.end(), [&](const IoAddress& u) {
            if (u.area != area) {
                return false;
            }
            const std::size_t uWidth = u.size == IoAddress::Size::Bit ? 1 : u.size == IoAddress::Size::Word ? 2 : 4;
            const bool overlap = u.byte < byte + width && byte < u.byte + uWidth;
            if (!overlap) {
                return false;
            }
            // Two bits in the same byte only clash if they're the same bit.
            return !(bit && u.size == IoAddress::Size::Bit && u.bit != *bit);
        });
    };
    if (size == IoAddress::Size::Bit) {
        for (std::size_t byte = 0; byte < 8; ++byte) {
            for (unsigned bit = 0; bit < 8; ++bit) {
                if (!taken(byte, 1, bit)) {
                    return formatAddress(IoAddress{area, size, byte, bit});
                }
            }
        }
        return {};
    }
    const std::size_t width = size == IoAddress::Size::Word ? 2 : 4;
    for (std::size_t byte = 8; byte + width <= IoImage::kBytes; byte += width) {
        if (!taken(byte, width, std::nullopt)) {
            return formatAddress(IoAddress{area, size, byte, 0});
        }
    }
    return {};
}

// Where to fire, in encoder pulses after the sensor sees a box. The sensor
// switches when the box's front edge reaches it. The paddle should meet the
// box's middle when it's halfway across the belt, which is the dead time
// plus half the stroke after firing; the box moves on meanwhile, so it fires
// that much earlier:
//
//   distance = (pusher - sensor) + box length / 2 - speed x (dead time + stroke / 2)
//
// The time part assumes the belt's set speed: change the speed and the
// sorter needs a new distance (real sorters compensate the same way).
SorterSpec sorterFor(const Scene& scene, std::size_t photocell, std::size_t pusher) {
    const auto& sensor = scene.photocells.at(photocell);
    const auto& paddle = scene.pushers.at(pusher);
    const auto& conveyor = scene.conveyors.at(paddle.conveyor).spec;
    float boxLength = 0.2F;
    for (const auto& spawner : scene.spawners) {
        if (spawner.conveyor == paddle.conveyor) {
            boxLength = spawner.size.x;
        }
    }
    const double lead =
        conveyor.targetSpeed * (paddle.stroke.deadTime + paddle.stroke.extendTime / 2.0); // m travelled while it moves
    const double metres = (paddle.along - sensor.along) + boxLength / 2.0 - lead;
    SorterSpec sorter;
    sorter.name = "Sorter " + paddle.name;
    sorter.nokInput = sensor.input;
    sorter.encoderInput = scene.conveyors.at(sensor.conveyor).spec.encoderInput;
    sorter.valveOutput = paddle.valveOutput;
    sorter.extendedInput = paddle.extendedInput;
    sorter.distance = std::max<std::int64_t>(0, std::llround(metres * conveyor.encoder.pulsesPerMetre()));
    return sorter;
}

std::vector<ConveyorSpec> placeConveyors(const Scene& scene) {
    std::vector<ConveyorSpec> placed;
    placed.reserve(scene.conveyors.size());
    for (std::size_t i = 0; i < scene.conveyors.size(); ++i) {
        const auto& conveyor = scene.conveyors[i];
        auto spec = conveyor.spec;
        if (conveyor.follows && *conveyor.follows < i) {
            const auto& before = placed[*conveyor.follows];
            const auto direction = directionOf(before.heading);
            const auto end = before.start + direction * before.length;
            switch (conveyor.turn) {
            case Turn::Straight:
                spec.heading = before.heading;
                spec.start = end + direction * kTransferGap;
                break;
            case Turn::Left:
            case Turn::Right: {
                // A right-angle transfer: the next belt runs across the end
                // of this one, its centre line just past the end, starting
                // far enough back to catch the whole box.
                spec.heading = before.heading + (conveyor.turn == Turn::Left ? kQuarterTurn : -kQuarterTurn);
                const auto across = directionOf(spec.heading);
                spec.start = end + direction * (kTransferGap + spec.width / 2.0F) - across * (before.width / 2.0F);
                break;
            }
            }
        }
        placed.push_back(spec);
    }
    return placed;
}

bool sameLayout(const Scene& a, const Scene& b) {
    if (a.conveyors.size() != b.conveyors.size() || !samePlaces(a.pushers, b.pushers) ||
        !samePlaces(a.diverters, b.diverters) || !samePlaces(a.stoppers, b.stoppers) || !samePlaces(a.bins, b.bins)) {
        return false;
    }
    const auto pa = placeConveyors(a);
    const auto pb = placeConveyors(b);
    for (std::size_t i = 0; i < pa.size(); ++i) {
        if (pa[i].start != pb[i].start || pa[i].heading != pb[i].heading || pa[i].length != pb[i].length ||
            pa[i].width != pb[i].width || pa[i].beltHeight != pb[i].beltHeight || pa[i].guideLeft != pb[i].guideLeft ||
            pa[i].guideRight != pb[i].guideRight) {
            return false;
        }
    }
    return true;
}

std::unique_ptr<World> buildWorld(const Scene& scene, const ProgramMaker& make) {
    auto world = std::make_unique<World>();
    for (const auto& spec : placeConveyors(scene)) {
        world->addConveyor(spec);
    }
    for (const auto& spawner : scene.spawners) {
        world->addSpawner(spawner);
    }
    for (const auto& photocell : scene.photocells) {
        world->addPhotocell(photocell);
    }
    // (Anything on a conveyor that doesn't exist is left out: the loader
    // rejects such files, so it only happens to a scene built by hand.)
    const auto exists = [&](std::size_t conveyor) { return conveyor < world->conveyors().size(); };
    for (const auto& pusher : scene.pushers) {
        if (exists(pusher.conveyor))
            world->addPusher(pusher);
    }
    for (const auto& diverter : scene.diverters) {
        if (exists(diverter.conveyor))
            world->addDiverter(diverter);
    }
    for (const auto& stopper : scene.stoppers) {
        if (exists(stopper.conveyor))
            world->addStopper(stopper);
    }
    for (const auto& bin : scene.bins) {
        if (exists(bin.conveyor))
            world->addBin(bin);
    }
    for (const auto& bin : scene.bins) {
        if (exists(bin.conveyor))
            world->addBin(bin);
    }
    world->setPanel(scene.buttons, scene.lamps);
    world->setProgram(make(scene));
    return world;
}

void applyLive(World& world, const Scene& scene) {
    const auto placed = placeConveyors(scene);
    for (std::size_t i = 0; i < placed.size() && i < world.conveyors().size(); ++i) {
        world.conveyors()[i].spec = placed[i];
    }
    world.setPhotocells(scene.photocells);
    world.setSpawners(scene.spawners);
    world.setStrokes(scene.pushers, scene.diverters, scene.stoppers);
    world.setPanel(scene.buttons, scene.lamps);
}

// ---------------------------------------------------------------- Editing

std::size_t addConveyorAfter(Scene& scene, std::size_t conveyor, Turn turn) {
    PlacedConveyor next;
    next.spec = scene.conveyors.at(conveyor).spec;
    next.spec.name = freeName(scene.conveyors, "C", [](const PlacedConveyor& c) { return c.spec.name; });
    next.follows = conveyor;
    next.turn = turn;
    // A box coming off the conveyor before lands across a turned belt: a
    // guide on the far side stops it flying off and turns it.
    next.spec.guideLeft = turn == Turn::Right;
    next.spec.guideRight = turn == Turn::Left;
    // And it sits a little lower, so a box only drops onto it once it has
    // really left the conveyor before; at the same height, the turned belt
    // would catch its front edge and drag it sideways off the first one.
    if (turn != Turn::Straight) {
        next.spec.beltHeight -= 0.02F;
    }
    next.spec.runOutput.clear(); // its own I/O, not the one it was copied from
    next.spec.encoderInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::DoubleWord);
    scene.conveyors.push_back(next);
    return scene.conveyors.size() - 1;
}

std::size_t addConveyor(Scene& scene) {
    // Beside everything there is: 1 m further along +Z than the furthest part.
    float furthest = 0.0F;
    bool any = false;
    for (const auto& spec : placeConveyors(scene)) {
        const auto end = spec.start + directionOf(spec.heading) * spec.length;
        const float reach = std::max(spec.start.z, end.z) + spec.width / 2.0F;
        furthest = any ? std::max(furthest, reach) : reach;
        any = true;
    }
    PlacedConveyor conveyor;
    conveyor.spec.name = freeName(scene.conveyors, "C", [](const PlacedConveyor& c) { return c.spec.name; });
    conveyor.spec.start = {0.0F, 0.0F, any ? furthest + 1.0F + conveyor.spec.width / 2.0F : 0.0F};
    conveyor.spec.encoderInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::DoubleWord);
    scene.conveyors.push_back(conveyor);
    return scene.conveyors.size() - 1;
}

void removeConveyor(Scene& scene, std::size_t conveyor) {
    if (conveyor >= scene.conveyors.size()) {
        return;
    }
    // Conveyors that followed it keep their place, on their own now.
    const auto placed = placeConveyors(scene);
    for (std::size_t i = 0; i < scene.conveyors.size(); ++i) {
        auto& c = scene.conveyors[i];
        if (c.follows == conveyor) {
            c.spec.start = placed[i].start;
            c.spec.heading = placed[i].heading;
            c.follows.reset();
            c.turn = Turn::Straight;
        } else if (c.follows) {
            shiftDown(*c.follows, conveyor);
        }
    }
    scene.conveyors.erase(scene.conveyors.begin() + static_cast<std::ptrdiff_t>(conveyor));
    // What stood on it goes with it.
    dropFrom(scene.photocells, conveyor);
    dropFrom(scene.spawners, conveyor);
    dropFrom(scene.pushers, conveyor);
    dropFrom(scene.diverters, conveyor);
    dropFrom(scene.stoppers, conveyor);
    dropFrom(scene.bins, conveyor);
}

std::size_t addPusher(Scene& scene, std::size_t conveyor) {
    PusherSpec pusher;
    pusher.name = freeName(scene.pushers, "P", [](const PusherSpec& p) { return p.name; });
    pusher.conveyor = conveyor;
    pusher.along = scene.conveyors.at(conveyor).spec.length * 2.0F / 3.0F;
    // Wired to the next free output for its valve, and inputs for its sensors.
    pusher.valveOutput = freeAddress(scene, IoAddress::Area::Output, IoAddress::Size::Bit);
    pusher.extendedInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    scene.pushers.push_back(pusher); // (in the scene now, so the next free input is another one)
    scene.pushers.back().retractedInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    return scene.pushers.size() - 1;
}

std::size_t addDiverter(Scene& scene, std::size_t conveyor) {
    DiverterSpec diverter;
    diverter.name = freeName(scene.diverters, "D", [](const DiverterSpec& d) { return d.name; });
    diverter.conveyor = conveyor;
    diverter.along = scene.conveyors.at(conveyor).spec.length / 3.0F;
    // Wired to the next free output for its valve, and inputs for its sensors.
    diverter.valveOutput = freeAddress(scene, IoAddress::Area::Output, IoAddress::Size::Bit);
    diverter.extendedInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    scene.diverters.push_back(diverter); // (in the scene now, so the next free input is another one)
    scene.diverters.back().retractedInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    return scene.diverters.size() - 1;
}

std::size_t addStopper(Scene& scene, std::size_t conveyor) {
    StopperSpec stopper;
    stopper.name = freeName(scene.stoppers, "S", [](const StopperSpec& s) { return s.name; });
    stopper.conveyor = conveyor;
    stopper.along = scene.conveyors.at(conveyor).spec.length - 0.3F;
    // Wired to the next free output for its valve, and inputs for its sensors.
    stopper.valveOutput = freeAddress(scene, IoAddress::Area::Output, IoAddress::Size::Bit);
    stopper.extendedInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    scene.stoppers.push_back(stopper); // (in the scene now, so the next free input is another one)
    scene.stoppers.back().retractedInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    return scene.stoppers.size() - 1;
}

std::size_t addBin(Scene& scene, std::size_t conveyor) {
    BinSpec bin;
    bin.name = freeName(scene.bins, "Bin", [](const BinSpec& b) { return b.name; });
    bin.conveyor = conveyor;
    bin.along = scene.conveyors.at(conveyor).spec.length / 2.0F;
    // Opposite the last pusher on this conveyor, where it pushes boxes to.
    for (const auto& pusher : scene.pushers) {
        if (pusher.conveyor == conveyor) {
            bin.along = pusher.along;
            bin.side = pusher.side == Side::Left ? Side::Right : Side::Left;
        }
    }
    scene.bins.push_back(bin);
    return scene.bins.size() - 1;
}

void removePusher(Scene& scene, std::size_t index) {
    eraseAt(scene.pushers, index);
}
void removeDiverter(Scene& scene, std::size_t index) {
    eraseAt(scene.diverters, index);
}
void removeStopper(Scene& scene, std::size_t index) {
    eraseAt(scene.stoppers, index);
}
void removeBin(Scene& scene, std::size_t index) {
    eraseAt(scene.bins, index);
}

std::size_t addPhotocell(Scene& scene, std::size_t conveyor) {
    PhotocellSpec photocell;
    photocell.name = freeName(scene.photocells, "B", [](const PhotocellSpec& p) { return p.name; });
    photocell.conveyor = conveyor;
    photocell.along = scene.conveyors.at(conveyor).spec.length / 2.0F;
    photocell.input = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    scene.photocells.push_back(photocell);
    return scene.photocells.size() - 1;
}

void removePhotocell(Scene& scene, std::size_t photocell) {
    if (photocell < scene.photocells.size()) {
        scene.photocells.erase(scene.photocells.begin() + static_cast<std::ptrdiff_t>(photocell));
    }
}

std::size_t addSpawner(Scene& scene, std::size_t conveyor) {
    SpawnerSpec spawner;
    spawner.conveyor = conveyor;
    scene.spawners.push_back(spawner);
    return scene.spawners.size() - 1;
}

void removeSpawner(Scene& scene, std::size_t spawner) {
    if (spawner < scene.spawners.size()) {
        scene.spawners.erase(scene.spawners.begin() + static_cast<std::ptrdiff_t>(spawner));
    }
}

// ---------------------------------------------------------------- Files

namespace {

json vec3(glm::vec3 v) {
    return json::array({v.x, v.y, v.z});
}

glm::vec3 vec3From(const json& j, glm::vec3 fallback) {
    if (!j.is_array() || j.size() != 3) {
        return fallback;
    }
    return {j[0].get<float>(), j[1].get<float>(), j[2].get<float>()};
}

json strokeJson(const StrokeSpec& s) {
    return {{"deadTime", s.deadTime}, {"extendTime", s.extendTime}, {"retractTime", s.retractTime}};
}

StrokeSpec strokeFrom(const json& j, StrokeSpec s) {
    if (j.is_object()) {
        s.deadTime = j.value("deadTime", s.deadTime);
        s.extendTime = j.value("extendTime", s.extendTime);
        s.retractTime = j.value("retractTime", s.retractTime);
    }
    return s;
}

// An actuator's I/O wiring, in the file and back.
template <class Spec> void ioToJson(json& j, const Spec& s) {
    j["valveOutput"] = s.valveOutput;
    j["extendedInput"] = s.extendedInput;
    j["retractedInput"] = s.retractedInput;
}
template <class Spec> void ioFromJson(const json& j, Spec& s) {
    s.valveOutput = j.value("valveOutput", std::string());
    s.extendedInput = j.value("extendedInput", std::string());
    s.retractedInput = j.value("retractedInput", std::string());
}

const char* controlName(Control control) {
    switch (control) {
    case Control::BuiltInSorting:
        return "built-in sorting";
    case Control::SoftPlc:
        return "soft_plc";
    case Control::S7Plc:
        return "S7 PLC";
    case Control::Manual:
        break;
    }
    return "manual";
}

Control controlFrom(const std::string& name) {
    for (const auto control : {Control::BuiltInSorting, Control::SoftPlc, Control::S7Plc}) {
        if (name == controlName(control)) {
            return control;
        }
    }
    return Control::Manual;
}

const char* sideName(Side side) {
    return side == Side::Left ? "left" : "right";
}
Side sideFrom(const json& j, Side fallback) {
    if (!j.is_string()) {
        return fallback;
    }
    return j.get<std::string>() == "left" ? Side::Left : Side::Right;
}

const char* turnName(Turn turn) {
    switch (turn) {
    case Turn::Left:
        return "left";
    case Turn::Right:
        return "right";
    case Turn::Straight:
        break;
    }
    return "straight";
}

} // namespace

std::string sceneToJson(const Scene& scene) {
    json data = {{"format", kFormat},         {"conveyors", json::array()}, {"photocells", json::array()},
                 {"spawners", json::array()}, {"pushers", json::array()},   {"diverters", json::array()},
                 {"stoppers", json::array()}, {"bins", json::array()}};
    for (const auto& c : scene.conveyors) {
        json conveyor = {{"name", c.spec.name},
                         {"length", c.spec.length},
                         {"width", c.spec.width},
                         {"beltHeight", c.spec.beltHeight},
                         {"targetSpeed", c.spec.targetSpeed},
                         {"acceleration", c.spec.acceleration},
                         {"guideLeft", c.spec.guideLeft},
                         {"guideRight", c.spec.guideRight},
                         {"runOutput", c.spec.runOutput},
                         {"encoderInput", c.spec.encoderInput},
                         {"encoder",
                          {{"pulsesPerRevolution", c.spec.encoder.pulsesPerRevolution},
                           {"rollerDiameter", c.spec.encoder.rollerDiameter}}}};
        if (c.follows) {
            conveyor["follows"] = *c.follows;
            conveyor["turn"] = turnName(c.turn);
        } else {
            conveyor["start"] = vec3(c.spec.start);
            conveyor["heading"] = c.spec.heading; // radians, as in the code: converting would lose the last bit
        }
        data["conveyors"].push_back(conveyor);
    }
    for (const auto& p : scene.photocells) {
        data["photocells"].push_back({{"name", p.name},
                                      {"conveyor", p.conveyor},
                                      {"along", p.along},
                                      {"height", p.height},
                                      {"beamDiameter", p.beamDiameter},
                                      {"onFraction", p.onFraction},
                                      {"offFraction", p.offFraction},
                                      {"responseTime", p.responseTime},
                                      {"input", p.input},
                                      {"onlyNok", p.onlyNok}});
    }
    for (const auto& s : scene.spawners) {
        data["spawners"].push_back({{"conveyor", s.conveyor},
                                    {"interval", s.interval},
                                    {"intervalJitter", s.intervalJitter},
                                    {"size", vec3(s.size)},
                                    {"offset", s.offset},
                                    {"enabled", s.enabled},
                                    {"nokFraction", s.nokFraction}});
    }
    for (const auto& p : scene.pushers) {
        json entry = {{"name", p.name},           {"conveyor", p.conveyor},         {"along", p.along},
                      {"side", sideName(p.side)}, {"paddleLength", p.paddleLength}, {"stroke", strokeJson(p.stroke)}};
        ioToJson(entry, p);
        data["pushers"].push_back(entry);
    }
    for (const auto& d : scene.diverters) {
        json entry = {{"name", d.name},           {"conveyor", d.conveyor}, {"along", d.along},
                      {"side", sideName(d.side)}, {"angle", d.angle},       {"stroke", strokeJson(d.stroke)}};
        ioToJson(entry, d);
        data["diverters"].push_back(entry);
    }
    for (const auto& s : scene.stoppers) {
        json entry = {{"name", s.name},
                      {"conveyor", s.conveyor},
                      {"along", s.along},
                      {"height", s.height},
                      {"stroke", strokeJson(s.stroke)}};
        ioToJson(entry, s);
        data["stoppers"].push_back(entry);
    }
    for (const auto& b : scene.bins) {
        data["bins"].push_back({{"name", b.name},
                                {"conveyor", b.conveyor},
                                {"along", b.along},
                                {"side", sideName(b.side)},
                                {"size", vec3(b.size)}});
    }
    data["buttons"] = json::array();
    for (const auto& b : scene.buttons) {
        data["buttons"].push_back(
            {{"name", b.name}, {"address", b.address}, {"normallyClosed", b.normallyClosed}, {"latching", b.latching}});
    }
    data["lamps"] = json::array();
    for (const auto& l : scene.lamps) {
        data["lamps"].push_back({{"name", l.name}, {"address", l.address}});
    }
    data["sorters"] = json::array();
    for (const auto& s : scene.sorters) {
        data["sorters"].push_back({{"name", s.name},
                                   {"nokInput", s.nokInput},
                                   {"encoderInput", s.encoderInput},
                                   {"valveOutput", s.valveOutput},
                                   {"extendedInput", s.extendedInput},
                                   {"distance", s.distance}});
    }
    data["control"] = controlName(scene.control);
    data["plcProgram"] = scene.plcProgram;
    data["monitorPort"] = scene.monitorPort;
    data["s7Host"] = scene.s7Host;
    data["s7Port"] = scene.s7Port;
    return data.dump(2) + "\n";
}

tl::expected<Scene, std::string> sceneFromJson(std::string_view text) {
    try {
        const auto data = json::parse(text);
        if (data.value("format", 0) != kFormat) {
            return tl::unexpected(fmt::format("unknown scene format {} (this version reads format {})",
                                              data.value("format", 0), kFormat));
        }
        Scene scene;
        // Missing fields get their defaults, so older files keep loading.
        const PlacedConveyor defaults;
        for (const auto& c : data.value("conveyors", json::array())) {
            PlacedConveyor conveyor;
            auto& spec = conveyor.spec;
            spec.name = c.value("name", defaults.spec.name);
            spec.length = c.value("length", defaults.spec.length);
            spec.width = c.value("width", defaults.spec.width);
            spec.beltHeight = c.value("beltHeight", defaults.spec.beltHeight);
            spec.targetSpeed = c.value("targetSpeed", defaults.spec.targetSpeed);
            spec.acceleration = c.value("acceleration", defaults.spec.acceleration);
            spec.guideLeft = c.value("guideLeft", false);
            spec.guideRight = c.value("guideRight", false);
            spec.runOutput = c.value("runOutput", std::string());
            spec.encoderInput = c.value("encoderInput", std::string());
            const auto encoder = c.value("encoder", json::object());
            spec.encoder.pulsesPerRevolution = encoder.value("pulsesPerRevolution", spec.encoder.pulsesPerRevolution);
            spec.encoder.rollerDiameter = encoder.value("rollerDiameter", spec.encoder.rollerDiameter);
            if (c.contains("follows")) {
                const auto follows = c["follows"].get<std::size_t>();
                if (follows >= scene.conveyors.size()) {
                    return tl::unexpected(fmt::format("conveyor {} follows conveyor {}, which doesn't come before it",
                                                      spec.name, follows));
                }
                conveyor.follows = follows;
                const auto turn = c.value("turn", std::string("straight"));
                conveyor.turn = turn == "left" ? Turn::Left : turn == "right" ? Turn::Right : Turn::Straight;
            } else {
                spec.start = vec3From(c.value("start", json()), spec.start);
                spec.heading = c.value("heading", 0.0F);
            }
            scene.conveyors.push_back(conveyor);
        }
        const auto checkConveyor = [&](std::size_t index, const std::string& what) -> tl::expected<void, std::string> {
            if (index >= scene.conveyors.size()) {
                return tl::unexpected(fmt::format("{} is on conveyor {}, which doesn't exist", what, index));
            }
            return {};
        };
        for (const auto& p : data.value("photocells", json::array())) {
            PhotocellSpec photocell;
            photocell.name = p.value("name", photocell.name);
            photocell.conveyor = p.value("conveyor", photocell.conveyor);
            photocell.along = p.value("along", photocell.along);
            photocell.height = p.value("height", photocell.height);
            photocell.beamDiameter = p.value("beamDiameter", photocell.beamDiameter);
            photocell.onFraction = p.value("onFraction", photocell.onFraction);
            photocell.offFraction = p.value("offFraction", photocell.offFraction);
            photocell.responseTime = p.value("responseTime", photocell.responseTime);
            photocell.input = p.value("input", std::string());
            photocell.onlyNok = p.value("onlyNok", false);
            if (auto ok = checkConveyor(photocell.conveyor, "photocell " + photocell.name); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.photocells.push_back(photocell);
        }
        for (const auto& s : data.value("spawners", json::array())) {
            SpawnerSpec spawner;
            spawner.conveyor = s.value("conveyor", spawner.conveyor);
            spawner.interval = s.value("interval", spawner.interval);
            spawner.intervalJitter = s.value("intervalJitter", spawner.intervalJitter);
            spawner.size = vec3From(s.value("size", json()), spawner.size);
            spawner.offset = s.value("offset", spawner.offset);
            spawner.enabled = s.value("enabled", spawner.enabled);
            spawner.nokFraction = s.value("nokFraction", spawner.nokFraction);
            if (auto ok = checkConveyor(spawner.conveyor, "a spawner"); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.spawners.push_back(spawner);
        }
        for (const auto& j : data.value("pushers", json::array())) {
            PusherSpec p;
            p.name = j.value("name", p.name);
            p.conveyor = j.value("conveyor", p.conveyor);
            p.along = j.value("along", p.along);
            p.side = sideFrom(j.value("side", json()), p.side);
            p.paddleLength = j.value("paddleLength", p.paddleLength);
            p.stroke = strokeFrom(j.value("stroke", json()), p.stroke);
            ioFromJson(j, p);
            if (auto ok = checkConveyor(p.conveyor, "pusher " + p.name); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.pushers.push_back(p);
        }
        for (const auto& j : data.value("diverters", json::array())) {
            DiverterSpec d;
            d.name = j.value("name", d.name);
            d.conveyor = j.value("conveyor", d.conveyor);
            d.along = j.value("along", d.along);
            d.side = sideFrom(j.value("side", json()), d.side);
            d.angle = j.value("angle", d.angle);
            d.stroke = strokeFrom(j.value("stroke", json()), d.stroke);
            ioFromJson(j, d);
            if (auto ok = checkConveyor(d.conveyor, "diverter " + d.name); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.diverters.push_back(d);
        }
        for (const auto& j : data.value("stoppers", json::array())) {
            StopperSpec s;
            s.name = j.value("name", s.name);
            s.conveyor = j.value("conveyor", s.conveyor);
            s.along = j.value("along", s.along);
            s.height = j.value("height", s.height);
            s.stroke = strokeFrom(j.value("stroke", json()), s.stroke);
            ioFromJson(j, s);
            if (auto ok = checkConveyor(s.conveyor, "stopper " + s.name); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.stoppers.push_back(s);
        }
        for (const auto& j : data.value("bins", json::array())) {
            BinSpec b;
            b.name = j.value("name", b.name);
            b.conveyor = j.value("conveyor", b.conveyor);
            b.along = j.value("along", b.along);
            b.side = sideFrom(j.value("side", json()), b.side);
            b.size = vec3From(j.value("size", json()), b.size);
            if (auto ok = checkConveyor(b.conveyor, "bin " + b.name); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.bins.push_back(b);
        }
        for (const auto& j : data.value("buttons", json::array())) {
            ButtonSpec b;
            b.name = j.value("name", b.name);
            b.address = j.value("address", b.address);
            b.normallyClosed = j.value("normallyClosed", b.normallyClosed);
            b.latching = j.value("latching", b.latching);
            scene.buttons.push_back(b);
        }
        for (const auto& j : data.value("lamps", json::array())) {
            LampSpec l;
            l.name = j.value("name", l.name);
            l.address = j.value("address", l.address);
            scene.lamps.push_back(l);
        }
        for (const auto& j : data.value("sorters", json::array())) {
            SorterSpec s;
            s.name = j.value("name", s.name);
            s.nokInput = j.value("nokInput", s.nokInput);
            s.encoderInput = j.value("encoderInput", s.encoderInput);
            s.valveOutput = j.value("valveOutput", s.valveOutput);
            s.extendedInput = j.value("extendedInput", s.extendedInput);
            s.distance = j.value("distance", s.distance);
            scene.sorters.push_back(s);
        }
        scene.control = controlFrom(data.value("control", std::string("manual")));
        scene.plcProgram = data.value("plcProgram", std::string());
        scene.monitorPort = data.value("monitorPort", 0);
        scene.s7Host = data.value("s7Host", scene.s7Host);
        scene.s7Port = data.value("s7Port", scene.s7Port);
        return scene;
    } catch (const json::exception& error) {
        return tl::unexpected(std::string("the scene file isn't valid: ") + error.what());
    }
}

tl::expected<void, std::string> saveScene(const Scene& scene, const std::filesystem::path& path) {
    auto temporary = path;
    temporary += ".tmp";
    {
        std::ofstream file(temporary, std::ios::binary);
        file << sceneToJson(scene);
        if (!file) {
            return tl::unexpected("can't write " + temporary.string());
        }
    }
    std::error_code error;
    std::filesystem::rename(temporary, path, error);
    if (error) {
        return tl::unexpected("can't replace " + path.string() + ": " + error.message());
    }
    return {};
}

tl::expected<Scene, std::string> loadScene(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return tl::unexpected("can't read " + path.string());
    }
    std::stringstream text;
    text << file.rdbuf();
    auto scene = sceneFromJson(text.str());
    if (!scene) {
        return tl::unexpected(path.filename().string() + ": " + scene.error());
    }
    return scene;
}

} // namespace fsim
