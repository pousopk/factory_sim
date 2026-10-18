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

glm::vec3 directionOf(float heading) {
    return {std::cos(heading), 0.0F, -std::sin(heading)};
}

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
    scene.photocells.push_back(photocell);
    return scene;
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
    if (a.conveyors.size() != b.conveyors.size()) {
        return false;
    }
    const auto pa = placeConveyors(a);
    const auto pb = placeConveyors(b);
    for (std::size_t i = 0; i < pa.size(); ++i) {
        if (pa[i].start != pb[i].start || pa[i].heading != pb[i].heading || pa[i].length != pb[i].length ||
            pa[i].width != pb[i].width || pa[i].beltHeight != pb[i].beltHeight) {
            return false;
        }
    }
    return true;
}

std::unique_ptr<World> buildWorld(const Scene& scene) {
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
    return world;
}

void applyLive(World& world, const Scene& scene) {
    const auto placed = placeConveyors(scene);
    for (std::size_t i = 0; i < placed.size() && i < world.conveyors().size(); ++i) {
        world.conveyors()[i].spec = placed[i];
    }
    world.setPhotocells(scene.photocells);
    world.setSpawners(scene.spawners);
}

// ---------------------------------------------------------------- Editing

std::size_t addConveyorAfter(Scene& scene, std::size_t conveyor, Turn turn) {
    PlacedConveyor next;
    next.spec = scene.conveyors.at(conveyor).spec;
    next.spec.name = freeName(scene.conveyors, "C", [](const PlacedConveyor& c) { return c.spec.name; });
    next.follows = conveyor;
    next.turn = turn;
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
    std::erase_if(scene.photocells, [&](const PhotocellSpec& p) { return p.conveyor == conveyor; });
    std::erase_if(scene.spawners, [&](const SpawnerSpec& s) { return s.conveyor == conveyor; });
    for (auto& p : scene.photocells) {
        shiftDown(p.conveyor, conveyor);
    }
    for (auto& s : scene.spawners) {
        shiftDown(s.conveyor, conveyor);
    }
}

std::size_t addPhotocell(Scene& scene, std::size_t conveyor) {
    PhotocellSpec photocell;
    photocell.name = freeName(scene.photocells, "B", [](const PhotocellSpec& p) { return p.name; });
    photocell.conveyor = conveyor;
    photocell.along = scene.conveyors.at(conveyor).spec.length / 2.0F;
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
    json data = {
        {"format", kFormat}, {"conveyors", json::array()}, {"photocells", json::array()}, {"spawners", json::array()}};
    for (const auto& c : scene.conveyors) {
        json conveyor = {{"name", c.spec.name},
                         {"length", c.spec.length},
                         {"width", c.spec.width},
                         {"beltHeight", c.spec.beltHeight},
                         {"targetSpeed", c.spec.targetSpeed},
                         {"acceleration", c.spec.acceleration},
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
                                      {"responseTime", p.responseTime}});
    }
    for (const auto& s : scene.spawners) {
        data["spawners"].push_back({{"conveyor", s.conveyor},
                                    {"interval", s.interval},
                                    {"intervalJitter", s.intervalJitter},
                                    {"size", vec3(s.size)},
                                    {"offset", s.offset},
                                    {"enabled", s.enabled}});
    }
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
            if (auto ok = checkConveyor(spawner.conveyor, "a spawner"); !ok) {
                return tl::unexpected(ok.error());
            }
            scene.spawners.push_back(spawner);
        }
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
