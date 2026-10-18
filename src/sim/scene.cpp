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
    return a.control == b.control && a.sorters == b.sorters && a.cameraTriggers == b.cameraTriggers &&
           a.plcProgram == b.plcProgram && a.monitorPort == b.monitorPort && a.s7Host == b.s7Host &&
           a.s7Port == b.s7Port && a.s7Bytes == b.s7Bytes;
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

Scene tinDemo() {
    Scene scene;
    PlacedConveyor conveyor;
    conveyor.spec.name = "C1";
    conveyor.spec.length = 3.0F;
    conveyor.spec.width = 0.12F;
    conveyor.spec.targetSpeed = 1.0F;
    conveyor.spec.acceleration = 2.0F;
    conveyor.spec.guideLeft = true;
    conveyor.spec.guideRight = true;
    conveyor.spec.encoderInput = "%ID8";
    scene.conveyors.push_back(conveyor);
    // Tins 73 mm across, 110 mm high, every 0.2 s give or take 0.08 s: at
    // 1 m/s, 0.12 to 0.28 m apart, centre to centre.
    SpawnerSpec spawner;
    spawner.shape = Shape::Tin;
    spawner.size = {0.073F, 0.11F, 0.073F};
    spawner.interval = 0.2;
    spawner.intervalJitter = 0.08;
    spawner.atBeltSpeed = true;
    scene.spawners.push_back(spawner);
    PhotocellSpec photocell;
    photocell.name = "B1";
    photocell.along = 1.2F;
    photocell.input = "%IX0.0";
    scene.photocells.push_back(photocell);
    // Three cameras around the tin where it is when the photocell triggers
    // (from above, the left and the right), and a fourth 15 cm further on,
    // where there was room for it.
    const struct {
        const char* name;
        float along;
        float tilt;
    } cameras[] = {{"K1", 1.17F, 0.0F}, {"K2", 1.17F, 0.9F}, {"K3", 1.17F, -0.9F}, {"K4", 1.32F, 0.0F}};
    for (const auto& c : cameras) {
        const auto index = addCamera(scene, 0);
        scene.cameras[index].name = c.name;
        scene.cameras[index].along = c.along;
        scene.cameras[index].tilt = c.tilt;
        scene.cameraTriggers.push_back(triggerFor(scene, 0, index));
    }
    scene.control = Control::BuiltInCameras;
    return scene;
}

std::unique_ptr<ControlProgram> makeProgram(const Scene& scene) {
    if (scene.control == Control::BuiltInSorting) {
        return std::make_unique<BuiltInSorting>(scene.sorters);
    }
    if (scene.control == Control::BuiltInCameras) {
        return std::make_unique<CameraTriggering>(scene.cameraTriggers);
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
        if (c.spec.drive.fitted) {
            add(c.spec.drive.controlOutput);
            add(c.spec.drive.setpointOutput);
            add(c.spec.drive.statusInput);
            add(c.spec.drive.actualInput);
        }
    }
    for (const auto& p : scene.potentiometers) {
        add(p.address);
    }
    for (const auto& p : scene.proximities) {
        add(p.input);
    }
    for (const auto& l : scene.lasers) {
        add(l.output);
    }
    for (const auto& c : scene.curtains) {
        add(c.clearInput);
        add(c.countInput);
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
    for (const auto& c : scene.cameras) {
        add(c.triggerOutput);
        add(c.readyInput);
    }
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

// The delay from the photocell's edge to a camera's trigger, so the item is
// in the middle of the picture. The photocell switches when the item's
// front reaches the beam, so its centre is half an item before it; it has to
// travel from there to the middle of the picture:
//
//   delay = (camera - photocell + item length / 2) / speed - 6 ms
//
// The 6 ms is what's already on the way on average: the photocell's 1 ms
// response and half a 10 ms scan before the PLC sees the edge. Like the
// sorter's distance, it's only right at the belt's set speed.
//
// And it's rounded to whole scans: the program looks at its timer once per
// scan, so a timer can only end on one. 151 ms really becomes 160, and the
// item is 9 mm late at 1 m/s; 150 is right on time. (A delay under half a
// scan becomes none: 2 ms would really be 10.)
CameraTriggerSpec triggerFor(const Scene& scene, std::size_t photocell, std::size_t camera) {
    const auto& sensor = scene.photocells.at(photocell);
    const auto& spec = scene.cameras.at(camera);
    const auto& conveyor = scene.conveyors.at(spec.conveyor).spec;
    float itemLength = 0.2F;
    for (const auto& spawner : scene.spawners) {
        if (spawner.conveyor == spec.conveyor) {
            itemLength = spawner.size.x;
        }
    }
    CameraTriggerSpec trigger;
    trigger.name = spec.name;
    trigger.sensorInput = sensor.input;
    trigger.triggerOutput = spec.triggerOutput;
    if (conveyor.targetSpeed > 0.0F) {
        const double travel = spec.along - sensor.along + itemLength / 2.0F;
        const double delay = travel / conveyor.targetSpeed - (sensor.responseTime + 0.005);
        const double scan = World::kStep * World::kScanSteps;
        trigger.delay = std::max(0.0, std::round(delay / scan) * scan);
    }
    return trigger;
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
    for (const auto& camera : scene.cameras) {
        world->addCamera(camera); // (one on a missing conveyor sees nothing)
    }
    world->setFieldSensors(scene.proximities, scene.lasers, scene.curtains);
    world->setPanel(scene.buttons, scene.lamps, scene.potentiometers);
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
    world.setPanel(scene.buttons, scene.lamps, scene.potentiometers);
    world.setCameras(scene.cameras);
    world.setFieldSensors(scene.proximities, scene.lasers, scene.curtains);
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
    dropFrom(scene.cameras, conveyor);
    dropFrom(scene.proximities, conveyor);
    dropFrom(scene.lasers, conveyor);
    dropFrom(scene.curtains, conveyor);
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

void fitDrive(Scene& scene, std::size_t conveyor) {
    auto& drive = scene.conveyors.at(conveyor).spec.drive;
    if (drive.fitted) {
        return;
    }
    // One at a time: each takes its address before the next is looked for.
    drive.fitted = true;
    drive.controlOutput = freeAddress(scene, IoAddress::Area::Output, IoAddress::Size::Word);
    drive.setpointOutput = freeAddress(scene, IoAddress::Area::Output, IoAddress::Size::Word);
    drive.statusInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Word);
    drive.actualInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Word);
}

Scene driveDemo(std::string program) {
    Scene scene;
    PlacedConveyor conveyor;
    conveyor.spec.name = "C1";
    conveyor.spec.length = 5.0F;
    conveyor.spec.targetSpeed = 0.5F;
    conveyor.spec.acceleration = 0.5F; // a gentle ramp, to watch
    conveyor.spec.encoderInput = "%ID8";
    // The telegram where a TIA project would often put it, after the encoder:
    // STW1, NSOLL_A out; ZSW1, NIST_A in.
    conveyor.spec.drive.fitted = true;
    conveyor.spec.drive.referenceSpeed = 1.0F;
    conveyor.spec.drive.controlOutput = "%QW12";
    conveyor.spec.drive.setpointOutput = "%QW14";
    conveyor.spec.drive.statusInput = "%IW12";
    conveyor.spec.drive.actualInput = "%IW14";
    scene.conveyors.push_back(conveyor);
    SpawnerSpec spawner;
    spawner.interval = 1.5;
    scene.spawners.push_back(spawner);
    scene.buttons = {
        ButtonSpec{"Start", "%IX0.0", false, false},
        ButtonSpec{"Stop", "%IX0.1", true, false}, // wired normally closed
        ButtonSpec{"Acknowledge", "%IX0.2", false, false},
    };
    scene.potentiometers = {PotentiometerSpec{"Speed", "%IW16"}};
    scene.lamps = {LampSpec{"Running", "%QX0.0"}, LampSpec{"Fault", "%QX0.1"}, LampSpec{"At speed", "%QX0.2"}};
    scene.control = Control::SoftPlc;
    scene.plcProgram = std::move(program);
    return scene;
}

Scene sensorsDemo() {
    Scene scene;
    PlacedConveyor conveyor;
    conveyor.spec.name = "C1";
    conveyor.spec.length = 4.0F;
    conveyor.spec.width = 0.10F; // a narrow lane between guides: items pass close to the switches
    conveyor.spec.targetSpeed = 0.4F;
    conveyor.spec.guideLeft = true;
    conveyor.spec.guideRight = true;
    scene.conveyors.push_back(conveyor);
    // Cardboard boxes of different heights, and tins, from the same infeed.
    SpawnerSpec boxes;
    boxes.size = {0.12F, 0.09F, 0.09F};
    boxes.heightJitter = 0.03F;
    boxes.interval = 1.6;
    boxes.atBeltSpeed = true;
    scene.spawners.push_back(boxes);
    SpawnerSpec tins;
    tins.shape = Shape::Tin;
    tins.size = {0.073F, 0.11F, 0.073F};
    tins.interval = 2.3;
    tins.atBeltSpeed = true;
    scene.spawners.push_back(tins);
    // M30 proximity switches on the left guide, 15 mm sn: the tins pass 13.5 mm
    // from them, the boxes 5 mm.
    const auto proximity = [&](const char* name, float along, bool inductive) {
        const auto index = addProximity(scene, 0);
        auto& p = scene.proximities[index];
        p.name = name;
        p.along = along;
        p.inductive = inductive;
        p.faceDiameter = 0.030F;
        p.range = 0.015F;
        p.height = 0.04F;
    };
    proximity("BI1", 1.2F, true);  // inductive: tins only
    proximity("BK1", 1.6F, false); // capacitive: tins, and boxes (cardboard, closer)
    const auto laser = addLaser(scene, 0);
    scene.lasers[laser].along = 2.2F;
    scene.lasers[laser].mountHeight = 0.5F;
    scene.lasers[laser].rangeMin = 0.25F; // 0 to 250 mm high items: 0..27648
    scene.lasers[laser].rangeMax = 0.5F;
    const auto curtain = addLightCurtain(scene, 0);
    scene.curtains[curtain].along = 2.8F;
    return scene;
}

std::size_t addProximity(Scene& scene, std::size_t conveyor) {
    ProximitySpec p;
    p.name = freeName(scene.proximities, "BP", [](const ProximitySpec& s) { return s.name; });
    p.conveyor = conveyor;
    p.along = scene.conveyors.at(conveyor).spec.length * 0.4F;
    p.input = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    scene.proximities.push_back(p);
    return scene.proximities.size() - 1;
}

std::size_t addLaser(Scene& scene, std::size_t conveyor) {
    LaserSpec l;
    l.name = freeName(scene.lasers, "BL", [](const LaserSpec& s) { return s.name; });
    l.conveyor = conveyor;
    l.along = scene.conveyors.at(conveyor).spec.length * 0.5F;
    l.output = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Word);
    scene.lasers.push_back(l);
    return scene.lasers.size() - 1;
}

std::size_t addLightCurtain(Scene& scene, std::size_t conveyor) {
    LightCurtainSpec c;
    c.name = freeName(scene.curtains, "BC", [](const LightCurtainSpec& s) { return s.name; });
    c.conveyor = conveyor;
    c.along = scene.conveyors.at(conveyor).spec.length * 0.7F;
    c.clearInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    scene.curtains.push_back(c); // the clear bit taken before the word is looked for
    scene.curtains.back().countInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Word);
    return scene.curtains.size() - 1;
}

void removeProximity(Scene& scene, std::size_t index) {
    eraseAt(scene.proximities, index);
}
void removeLaser(Scene& scene, std::size_t index) {
    eraseAt(scene.lasers, index);
}
void removeLightCurtain(Scene& scene, std::size_t index) {
    eraseAt(scene.curtains, index);
}

std::size_t addCamera(Scene& scene, std::size_t conveyor) {
    CameraSpec camera;
    camera.name = freeName(scene.cameras, "K", [](const CameraSpec& c) { return c.name; });
    camera.conveyor = conveyor;
    camera.along = scene.conveyors.at(conveyor).spec.length * 0.6F;
    camera.triggerOutput = freeAddress(scene, IoAddress::Area::Output, IoAddress::Size::Bit);
    camera.readyInput = freeAddress(scene, IoAddress::Area::Input, IoAddress::Size::Bit);
    scene.cameras.push_back(camera);
    return scene.cameras.size() - 1;
}

void removeCamera(Scene& scene, std::size_t index) {
    eraseAt(scene.cameras, index);
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
    case Control::BuiltInCameras:
        return "built-in camera triggering";
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
    for (const auto control : {Control::BuiltInSorting, Control::SoftPlc, Control::S7Plc, Control::BuiltInCameras}) {
        if (name == controlName(control)) {
            return control;
        }
    }
    return Control::Manual;
}

const char* mountName(Mount mount) {
    switch (mount) {
    case Mount::Above:
        return "above";
    case Mount::Right:
        return "right";
    case Mount::Left:
        break;
    }
    return "left";
}
Mount mountFrom(const std::string& name) {
    return name == "above" ? Mount::Above : name == "right" ? Mount::Right : Mount::Left;
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
                         {"drive",
                          {{"fitted", c.spec.drive.fitted},
                           {"referenceSpeed", c.spec.drive.referenceSpeed},
                           {"quickStop", c.spec.drive.quickStop},
                           {"coast", c.spec.drive.coast},
                           {"controlOutput", c.spec.drive.controlOutput},
                           {"setpointOutput", c.spec.drive.setpointOutput},
                           {"statusInput", c.spec.drive.statusInput},
                           {"actualInput", c.spec.drive.actualInput}}},
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
                                    {"nokFraction", s.nokFraction},
                                    {"shape", s.shape == Shape::Tin ? "tin" : "box"},
                                    {"heightJitter", s.heightJitter},
                                    {"atBeltSpeed", s.atBeltSpeed}});
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
    data["cameras"] = json::array();
    for (const auto& c : scene.cameras) {
        data["cameras"].push_back({{"name", c.name},
                                   {"conveyor", c.conveyor},
                                   {"along", c.along},
                                   {"fieldOfView", c.fieldOfView},
                                   {"pixels", c.pixels},
                                   {"tilt", c.tilt},
                                   {"triggerDelay", c.triggerDelay},
                                   {"exposure", c.exposure},
                                   {"readout", c.readout},
                                   {"triggerOutput", c.triggerOutput},
                                   {"readyInput", c.readyInput}});
    }
    data["proximities"] = json::array();
    for (const auto& p : scene.proximities) {
        data["proximities"].push_back({{"name", p.name},
                                       {"conveyor", p.conveyor},
                                       {"along", p.along},
                                       {"mount", mountName(p.mount)},
                                       {"inductive", p.inductive},
                                       {"faceDiameter", p.faceDiameter},
                                       {"range", p.range},
                                       {"height", p.height},
                                       {"inset", p.inset},
                                       {"responseTime", p.responseTime},
                                       {"input", p.input}});
    }
    data["lasers"] = json::array();
    for (const auto& l : scene.lasers) {
        data["lasers"].push_back({{"name", l.name},
                                  {"conveyor", l.conveyor},
                                  {"along", l.along},
                                  {"across", l.across},
                                  {"mountHeight", l.mountHeight},
                                  {"rangeMin", l.rangeMin},
                                  {"rangeMax", l.rangeMax},
                                  {"resolution", l.resolution},
                                  {"noise", l.noise},
                                  {"measureTime", l.measureTime},
                                  {"output", l.output}});
    }
    data["curtains"] = json::array();
    for (const auto& c : scene.curtains) {
        data["curtains"].push_back({{"name", c.name},
                                    {"conveyor", c.conveyor},
                                    {"along", c.along},
                                    {"beams", c.beams},
                                    {"pitch", c.pitch},
                                    {"lowest", c.lowest},
                                    {"scanTime", c.scanTime},
                                    {"clearInput", c.clearInput},
                                    {"countInput", c.countInput}});
    }
    data["buttons"] = json::array();
    for (const auto& b : scene.buttons) {
        data["buttons"].push_back(
            {{"name", b.name}, {"address", b.address}, {"normallyClosed", b.normallyClosed}, {"latching", b.latching}});
    }
    data["potentiometers"] = json::array();
    for (const auto& p : scene.potentiometers) {
        data["potentiometers"].push_back({{"name", p.name}, {"address", p.address}});
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
    data["cameraTriggers"] = json::array();
    for (const auto& t : scene.cameraTriggers) {
        data["cameraTriggers"].push_back({{"name", t.name},
                                          {"sensorInput", t.sensorInput},
                                          {"triggerOutput", t.triggerOutput},
                                          {"delay", t.delay},
                                          {"method", t.method == DelayMethod::Timer ? "timer" : "per item"}});
    }
    data["control"] = controlName(scene.control);
    data["plcProgram"] = scene.plcProgram;
    data["monitorPort"] = scene.monitorPort;
    data["s7Host"] = scene.s7Host;
    data["s7Port"] = scene.s7Port;
    data["s7Bytes"] = scene.s7Bytes;
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
            const auto drive = c.value("drive", json::object());
            spec.drive.fitted = drive.value("fitted", false);
            spec.drive.referenceSpeed = drive.value("referenceSpeed", spec.drive.referenceSpeed);
            spec.drive.quickStop = drive.value("quickStop", spec.drive.quickStop);
            spec.drive.coast = drive.value("coast", spec.drive.coast);
            spec.drive.controlOutput = drive.value("controlOutput", std::string());
            spec.drive.setpointOutput = drive.value("setpointOutput", std::string());
            spec.drive.statusInput = drive.value("statusInput", std::string());
            spec.drive.actualInput = drive.value("actualInput", std::string());
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
            spawner.shape = s.value("shape", std::string("box")) == "tin" ? Shape::Tin : Shape::Box;
            spawner.heightJitter = s.value("heightJitter", 0.0F);
            spawner.atBeltSpeed = s.value("atBeltSpeed", false);
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
        for (const auto& j : data.value("cameras", json::array())) {
            CameraSpec c;
            c.name = j.value("name", c.name);
            c.conveyor = j.value("conveyor", c.conveyor);
            c.along = j.value("along", c.along);
            c.fieldOfView = j.value("fieldOfView", c.fieldOfView);
            c.pixels = j.value("pixels", c.pixels);
            c.tilt = j.value("tilt", c.tilt);
            c.triggerDelay = j.value("triggerDelay", c.triggerDelay);
            c.exposure = j.value("exposure", c.exposure);
            c.readout = j.value("readout", c.readout);
            c.triggerOutput = j.value("triggerOutput", std::string());
            c.readyInput = j.value("readyInput", std::string());
            if (auto ok = checkConveyor(c.conveyor, "camera " + c.name); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.cameras.push_back(c);
        }
        for (const auto& j : data.value("proximities", json::array())) {
            ProximitySpec p;
            p.name = j.value("name", p.name);
            p.conveyor = j.value("conveyor", p.conveyor);
            p.along = j.value("along", p.along);
            p.mount = mountFrom(j.value("mount", std::string("left")));
            p.inductive = j.value("inductive", p.inductive);
            p.faceDiameter = j.value("faceDiameter", p.faceDiameter);
            p.range = j.value("range", p.range);
            p.height = j.value("height", p.height);
            p.inset = j.value("inset", p.inset);
            p.responseTime = j.value("responseTime", p.responseTime);
            p.input = j.value("input", std::string());
            if (auto ok = checkConveyor(p.conveyor, "proximity switch " + p.name); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.proximities.push_back(p);
        }
        for (const auto& j : data.value("lasers", json::array())) {
            LaserSpec l;
            l.name = j.value("name", l.name);
            l.conveyor = j.value("conveyor", l.conveyor);
            l.along = j.value("along", l.along);
            l.across = j.value("across", l.across);
            l.mountHeight = j.value("mountHeight", l.mountHeight);
            l.rangeMin = j.value("rangeMin", l.rangeMin);
            l.rangeMax = j.value("rangeMax", l.rangeMax);
            l.resolution = j.value("resolution", l.resolution);
            l.noise = j.value("noise", l.noise);
            l.measureTime = j.value("measureTime", l.measureTime);
            l.output = j.value("output", std::string());
            if (auto ok = checkConveyor(l.conveyor, "laser " + l.name); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.lasers.push_back(l);
        }
        for (const auto& j : data.value("curtains", json::array())) {
            LightCurtainSpec c;
            c.name = j.value("name", c.name);
            c.conveyor = j.value("conveyor", c.conveyor);
            c.along = j.value("along", c.along);
            c.beams = j.value("beams", c.beams);
            c.pitch = j.value("pitch", c.pitch);
            c.lowest = j.value("lowest", c.lowest);
            c.scanTime = j.value("scanTime", c.scanTime);
            c.clearInput = j.value("clearInput", std::string());
            c.countInput = j.value("countInput", std::string());
            if (auto ok = checkConveyor(c.conveyor, "light curtain " + c.name); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.curtains.push_back(c);
        }
        for (const auto& j : data.value("buttons", json::array())) {
            ButtonSpec b;
            b.name = j.value("name", b.name);
            b.address = j.value("address", b.address);
            b.normallyClosed = j.value("normallyClosed", b.normallyClosed);
            b.latching = j.value("latching", b.latching);
            scene.buttons.push_back(b);
        }
        for (const auto& j : data.value("potentiometers", json::array())) {
            PotentiometerSpec p;
            p.name = j.value("name", p.name);
            p.address = j.value("address", p.address);
            scene.potentiometers.push_back(p);
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
        for (const auto& j : data.value("cameraTriggers", json::array())) {
            CameraTriggerSpec t;
            t.name = j.value("name", t.name);
            t.sensorInput = j.value("sensorInput", t.sensorInput);
            t.triggerOutput = j.value("triggerOutput", t.triggerOutput);
            t.delay = j.value("delay", t.delay);
            t.method =
                j.value("method", std::string("timer")) == "per item" ? DelayMethod::PerItem : DelayMethod::Timer;
            scene.cameraTriggers.push_back(t);
        }
        scene.control = controlFrom(data.value("control", std::string("manual")));
        scene.plcProgram = data.value("plcProgram", std::string());
        scene.monitorPort = data.value("monitorPort", 0);
        scene.s7Host = data.value("s7Host", scene.s7Host);
        scene.s7Port = data.value("s7Port", scene.s7Port);
        scene.s7Bytes = data.value("s7Bytes", scene.s7Bytes);
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

void moveScene(Scene& scene, const std::filesystem::path& from, const std::filesystem::path& to) {
    if (scene.plcProgram.empty()) {
        return;
    }
    const std::filesystem::path program = scene.plcProgram;
    if (program.is_absolute()) {
        return;
    }
    const auto file = std::filesystem::absolute(from / program).lexically_normal();
    const auto relative = file.lexically_relative(std::filesystem::absolute(to).lexically_normal());
    scene.plcProgram = relative.empty() ? file.string() : relative.string();
}

} // namespace fsim
