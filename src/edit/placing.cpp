#include "edit/placing.hpp"
#include "sim/mounting.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fmt/format.h>
#include <limits>

namespace fsim {

float alongNearest(const ConveyorSpec& conveyor, glm::vec3 origin, glm::vec3 direction) {
    const auto start = beltPoint(conveyor, 0.0F);
    const auto u = directionOf(conveyor.heading);
    // Where the mouse is over the belt's level, projected onto its centre
    // line: what the eye expects, the point "under" the mouse.
    if (std::abs(direction.y) > 0.05F) {
        if (const auto hit = onPlane(origin, direction, conveyor.beltHeight)) {
            return glm::dot(*hit - start, u);
        }
    }
    // Looking almost level with the belt, that point runs off to infinity:
    // then the closest points of the two lines (the ray, and the centre line
    // start + u * along), where the line between them is at right angles to both.
    const auto w = start - origin;
    const float b = glm::dot(u, direction);
    const float d = glm::dot(u, w);
    const float e = glm::dot(direction, w);
    const float denominator = 1.0F - b * b; // both unit vectors
    if (std::abs(denominator) < 1e-6F) {
        return -d; // looking straight along the belt
    }
    return (b * e - d) / denominator;
}

std::optional<glm::vec3> onPlane(glm::vec3 origin, glm::vec3 direction, float height) {
    if (std::abs(direction.y) < 1e-6F) {
        return std::nullopt;
    }
    const float t = (height - origin.y) / direction.y;
    if (t <= 0.0F) {
        return std::nullopt;
    }
    return origin + direction * t;
}

std::optional<BeltHit> beltUnder(const std::vector<ConveyorSpec>& conveyors, glm::vec3 origin, glm::vec3 direction) {
    std::optional<BeltHit> best;
    for (std::size_t i = 0; i < conveyors.size(); ++i) {
        const auto& c = conveyors[i];
        const auto hit = onPlane(origin, direction, c.beltHeight);
        if (!hit) {
            continue;
        }
        const auto offset = *hit - beltPoint(c, 0.0F);
        const float along = glm::dot(offset, directionOf(c.heading));
        const float across = glm::dot(offset, leftOf(directionOf(c.heading)));
        const float distance = glm::length(*hit - origin);
        if (along >= 0.0F && along <= c.length && std::abs(across) <= c.width / 2.0F &&
            (!best || distance < best->distance)) {
            best = BeltHit{i, along, distance};
        }
    }
    return best;
}

float snap(float value, float step) {
    return step > 0.0F ? std::round(value / step) * step : value;
}

namespace {

// The place-along field of a device: its conveyor and its "along" (a
// spawner's offset). Calls `f(conveyor, along)` with references to them,
// const for a const scene.
template <class S, class F> bool withPlace(S& scene, Pick pick, F f) {
    const auto at = [&](auto& list) {
        if (pick.index >= list.size() || list[pick.index].place.parent == Parent::World) {
            return false; // (one standing on its own has no place along a belt)
        }
        f(list[pick.index].conveyor, list[pick.index].along);
        return true;
    };
    switch (pick.kind) {
    case Pick::Kind::Photocell:
        return at(scene.photocells);
    case Pick::Kind::Pusher:
        return at(scene.pushers);
    case Pick::Kind::Diverter:
        return at(scene.diverters);
    case Pick::Kind::Stopper:
        return at(scene.stoppers);
    case Pick::Kind::AirJet:
        return at(scene.airJets);
    case Pick::Kind::Bin:
        return at(scene.bins);
    case Pick::Kind::Camera:
        return at(scene.cameras);
    case Pick::Kind::Proximity:
        return at(scene.proximities);
    case Pick::Kind::Laser:
        return at(scene.lasers);
    case Pick::Kind::LightCurtain:
        return at(scene.curtains);
    case Pick::Kind::Spawner:
        if (pick.index >= scene.spawners.size() || scene.spawners[pick.index].place.parent == Parent::World) {
            return false;
        }
        f(scene.spawners[pick.index].conveyor, scene.spawners[pick.index].offset);
        return true;
    case Pick::Kind::Conveyor:
    case Pick::Kind::Item:
    case Pick::Kind::None:
        break;
    }
    return false;
}

// The next free name like `name`: the same letters, the first free number.
template <class List, class NameOf> std::string nextName(const List& list, const std::string& name, NameOf nameOf) {
    auto stem = name;
    while (!stem.empty() && std::isdigit(static_cast<unsigned char>(stem.back()))) {
        stem.pop_back();
    }
    for (int n = 1;; ++n) {
        const auto candidate = fmt::format("{}{}", stem, n);
        if (std::none_of(list.begin(), list.end(), [&](const auto& x) { return nameOf(x) == candidate; })) {
            return candidate;
        }
    }
}

// Gives an address field a free address of the same kind (area and size)
// as the one it had; an empty one stays empty.
void rewire(Scene& scene, std::string& field) {
    const auto old = parseAddress(field);
    field.clear();
    if (old) {
        field = freeAddress(scene, old->area, old->size);
    }
}

} // namespace

std::optional<std::pair<std::size_t, float>> placeOf(const Scene& scene, Pick pick) {
    std::optional<std::pair<std::size_t, float>> place;
    withPlace(scene, pick, [&](std::size_t conveyor, float along) { place = {conveyor, along}; });
    return place;
}

void moveAlong(Scene& scene, Pick pick, float along) {
    withPlace(scene, pick, [&](std::size_t& conveyor, float& at) {
        const float length = conveyor < scene.conveyors.size() ? scene.conveyors[conveyor].spec.length : 0.0F;
        at = std::clamp(along, 0.0F, length);
    });
}

std::optional<Pick> addAt(Scene& scene, Pick::Kind kind, std::size_t conveyor, float along) {
    if (conveyor >= scene.conveyors.size()) {
        return std::nullopt;
    }
    std::optional<Pick> added;
    switch (kind) {
    case Pick::Kind::Photocell:
        added = Pick{kind, addPhotocell(scene, conveyor)};
        break;
    case Pick::Kind::Spawner:
        added = Pick{kind, addSpawner(scene, conveyor)};
        break;
    case Pick::Kind::Pusher:
        added = Pick{kind, addPusher(scene, conveyor)};
        break;
    case Pick::Kind::Diverter:
        added = Pick{kind, addDiverter(scene, conveyor)};
        break;
    case Pick::Kind::Stopper:
        added = Pick{kind, addStopper(scene, conveyor)};
        break;
    case Pick::Kind::AirJet:
        added = Pick{kind, addAirJet(scene, conveyor)};
        break;
    case Pick::Kind::Bin:
        added = Pick{kind, addBin(scene, conveyor)};
        break;
    case Pick::Kind::Camera:
        added = Pick{kind, addCamera(scene, conveyor)};
        break;
    case Pick::Kind::Proximity:
        added = Pick{kind, addProximity(scene, conveyor)};
        break;
    case Pick::Kind::Laser:
        added = Pick{kind, addLaser(scene, conveyor)};
        break;
    case Pick::Kind::LightCurtain:
        added = Pick{kind, addLightCurtain(scene, conveyor)};
        break;
    case Pick::Kind::Conveyor:
    case Pick::Kind::Item:
    case Pick::Kind::None:
        return std::nullopt;
    }
    moveAlong(scene, *added, along);
    return added;
}

std::optional<Clip> copyOf(const Scene& scene, Pick pick) {
    const auto from = [&](const auto& list) -> std::optional<Clip> {
        if (pick.index >= list.size()) {
            return std::nullopt;
        }
        return Clip{list[pick.index]};
    };
    switch (pick.kind) {
    case Pick::Kind::Conveyor:
        return from(scene.conveyors);
    case Pick::Kind::Photocell:
        return from(scene.photocells);
    case Pick::Kind::Spawner:
        return from(scene.spawners);
    case Pick::Kind::Pusher:
        return from(scene.pushers);
    case Pick::Kind::Diverter:
        return from(scene.diverters);
    case Pick::Kind::Stopper:
        return from(scene.stoppers);
    case Pick::Kind::AirJet:
        return from(scene.airJets);
    case Pick::Kind::Bin:
        return from(scene.bins);
    case Pick::Kind::Camera:
        return from(scene.cameras);
    case Pick::Kind::Proximity:
        return from(scene.proximities);
    case Pick::Kind::Laser:
        return from(scene.lasers);
    case Pick::Kind::LightCurtain:
        return from(scene.curtains);
    case Pick::Kind::Item: // items aren't part of the scene: nothing to copy
    case Pick::Kind::None:
        break;
    }
    return std::nullopt;
}

Pick paste(Scene& scene, const Clip& clip, std::optional<std::size_t> conveyor, std::optional<float> along) {
    // A device: on the given conveyor (or its own), at the given place (or
    // 0.2 m further on), with a new name and its own wiring.
    const auto device = [&](auto copy, auto& list, Pick::Kind kind, auto... fields) {
        copy.name = nextName(list, copy.name, [](const auto& x) { return x.name; });
        if (conveyor && *conveyor < scene.conveyors.size()) {
            copy.conveyor = *conveyor;
        }
        list.push_back(copy);
        const auto index = list.size() - 1;
        (rewire(scene, list[index].*fields), ...);
        const Pick pick{kind, index};
        if (list[index].place.parent == Parent::World) {
            list[index].place.offset.x += 0.2F; // standing on its own: beside the original
            return pick;
        }
        const float to = along ? *along : copy.along + 0.2F;
        moveAlong(scene, pick, to);
        return pick;
    };
    return std::visit(
        [&](const auto& thing) -> Pick {
            using T = std::decay_t<decltype(thing)>;
            if constexpr (std::is_same_v<T, PlacedConveyor>) {
                // Beside the original, on its own, everything wired afresh.
                auto copy = thing;
                const auto placed = placeConveyors(scene);
                auto original = copy.spec;
                for (std::size_t i = 0; i < scene.conveyors.size(); ++i) {
                    if (scene.conveyors[i] == thing) {
                        original = placed[i];
                    }
                }
                copy.follows.reset();
                copy.joins.reset();
                copy.spec.start = original.start + leftOf(directionOf(original.heading)) * (original.width + 0.3F);
                copy.spec.heading = original.heading;
                copy.spec.beltHeight = original.beltHeight;
                copy.spec.name = nextName(scene.conveyors, copy.spec.name, [](const auto& c) { return c.spec.name; });
                scene.conveyors.push_back(copy);
                auto& spec = scene.conveyors.back().spec;
                rewire(scene, spec.runOutput);
                rewire(scene, spec.encoderInput);
                rewire(scene, spec.releaseOutput);
                for (auto& zone : spec.zoneIo) {
                    rewire(scene, zone.eyeInput);
                    rewire(scene, zone.motorOutput);
                }
                rewire(scene, spec.carriageOutput);
                rewire(scene, spec.carriageEndInput);
                rewire(scene, spec.carriageHomeInput);
                rewire(scene, spec.drive.controlOutput);
                rewire(scene, spec.drive.setpointOutput);
                rewire(scene, spec.drive.statusInput);
                rewire(scene, spec.drive.actualInput);
                return Pick{Pick::Kind::Conveyor, scene.conveyors.size() - 1};
            } else if constexpr (std::is_same_v<T, SpawnerSpec>) {
                auto copy = thing;
                if (conveyor && *conveyor < scene.conveyors.size()) {
                    copy.conveyor = *conveyor;
                }
                if (copy.place.parent == Parent::World) {
                    copy.place.offset.x += 0.2F;
                    scene.spawners.push_back(copy);
                    return Pick{Pick::Kind::Spawner, scene.spawners.size() - 1};
                }
                scene.spawners.push_back(copy);
                const Pick pick{Pick::Kind::Spawner, scene.spawners.size() - 1};
                moveAlong(scene, pick, along ? *along : copy.offset + 0.2F);
                return pick;
            } else if constexpr (std::is_same_v<T, PhotocellSpec>) {
                return device(thing, scene.photocells, Pick::Kind::Photocell, &PhotocellSpec::input);
            } else if constexpr (std::is_same_v<T, PusherSpec>) {
                return device(thing, scene.pushers, Pick::Kind::Pusher, &PusherSpec::valveOutput,
                              &PusherSpec::extendedInput, &PusherSpec::retractedInput);
            } else if constexpr (std::is_same_v<T, DiverterSpec>) {
                return device(thing, scene.diverters, Pick::Kind::Diverter, &DiverterSpec::valveOutput,
                              &DiverterSpec::extendedInput, &DiverterSpec::retractedInput);
            } else if constexpr (std::is_same_v<T, StopperSpec>) {
                return device(thing, scene.stoppers, Pick::Kind::Stopper, &StopperSpec::valveOutput,
                              &StopperSpec::extendedInput, &StopperSpec::retractedInput);
            } else if constexpr (std::is_same_v<T, AirJetSpec>) {
                return device(thing, scene.airJets, Pick::Kind::AirJet, &AirJetSpec::valveOutput);
            } else if constexpr (std::is_same_v<T, BinSpec>) {
                return device(thing, scene.bins, Pick::Kind::Bin);
            } else if constexpr (std::is_same_v<T, CameraSpec>) {
                return device(thing, scene.cameras, Pick::Kind::Camera, &CameraSpec::triggerOutput,
                              &CameraSpec::readyInput);
            } else if constexpr (std::is_same_v<T, ProximitySpec>) {
                return device(thing, scene.proximities, Pick::Kind::Proximity, &ProximitySpec::input);
            } else if constexpr (std::is_same_v<T, LaserSpec>) {
                return device(thing, scene.lasers, Pick::Kind::Laser, &LaserSpec::output);
            } else {
                static_assert(std::is_same_v<T, LightCurtainSpec>, "a kind of clip without a paste");
                return device(thing, scene.curtains, Pick::Kind::LightCurtain, &LightCurtainSpec::clearInput,
                              &LightCurtainSpec::countInput);
            }
        },
        clip);
}

// ---------------------------------------------------------------- Placing anywhere

namespace {

// Calls `f(spec, along)` with the device `pick` names (a spawner's along
// is its offset), const for a const scene. False if it names no device.
template <class S, class F> bool withDevice(S& scene, Pick pick, F f) {
    const auto at = [&](auto& list) {
        if (pick.index >= list.size()) {
            return false;
        }
        f(list[pick.index], list[pick.index].along);
        return true;
    };
    switch (pick.kind) {
    case Pick::Kind::Photocell:
        return at(scene.photocells);
    case Pick::Kind::Pusher:
        return at(scene.pushers);
    case Pick::Kind::Diverter:
        return at(scene.diverters);
    case Pick::Kind::Stopper:
        return at(scene.stoppers);
    case Pick::Kind::AirJet:
        return at(scene.airJets);
    case Pick::Kind::Bin:
        return at(scene.bins);
    case Pick::Kind::Camera:
        return at(scene.cameras);
    case Pick::Kind::Proximity:
        return at(scene.proximities);
    case Pick::Kind::Laser:
        return at(scene.lasers);
    case Pick::Kind::LightCurtain:
        return at(scene.curtains);
    case Pick::Kind::Spawner:
        if (pick.index >= scene.spawners.size()) {
            return false;
        }
        f(scene.spawners[pick.index], scene.spawners[pick.index].offset);
        return true;
    case Pick::Kind::Conveyor:
    case Pick::Kind::Item:
    case Pick::Kind::None:
        break;
    }
    return false;
}

// A device's conveyor, as placed, if it's on one that exists.
const ConveyorSpec* beltOf(const std::vector<ConveyorSpec>& placed, const auto& spec) {
    return spec.place.parent == Parent::Conveyor && spec.conveyor < placed.size() ? &placed[spec.conveyor] : nullptr;
}

const glm::vec3 kUp{0.0F, 1.0F, 0.0F};

} // namespace

const Placement* placementOf(const Scene& scene, Pick pick) {
    const Placement* place = nullptr;
    withDevice(scene, pick, [&](const auto& spec, float) { place = &spec.place; });
    return place;
}

Placement* placementOf(Scene& scene, Pick pick) {
    Placement* place = nullptr;
    withDevice(scene, pick, [&](auto& spec, float) { place = &spec.place; });
    return place;
}

std::optional<std::size_t> nearestConveyor(const Scene& scene, Pick pick) {
    const auto target = gizmoTarget(scene, pick);
    if (!target || pick.kind == Pick::Kind::Conveyor) {
        return std::nullopt;
    }
    // The distance to each belt's centre line, at its surface.
    const auto placed = placeConveyors(scene);
    std::optional<std::size_t> best;
    float nearest = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < placed.size(); ++i) {
        const auto& belt = placed[i];
        const auto start = beltPoint(belt, 0.0F);
        const auto along = directionOf(belt.heading);
        const float t = std::clamp(glm::dot(target->frame.origin - start, along), 0.0F, belt.length);
        const float d = glm::length(target->frame.origin - (start + along * t));
        if (d < nearest) {
            nearest = d;
            best = i;
        }
    }
    return best;
}

std::optional<GizmoTarget> gizmoTarget(const Scene& scene, Pick pick) {
    const auto placed = placeConveyors(scene);
    if (pick.kind == Pick::Kind::Conveyor) {
        if (pick.index >= scene.conveyors.size() || scene.conveyors[pick.index].follows ||
            scene.conveyors[pick.index].joins) {
            return std::nullopt;
        }
        // It moves across the floor and up, and turns only about the vertical.
        const auto& c = placed[pick.index];
        return GizmoTarget{beltFrame(c, c.length / 2.0F), {true, true, true}, {false, true, false}};
    }
    std::optional<GizmoTarget> target;
    withDevice(scene, pick, [&](const auto& spec, float along) {
        target = GizmoTarget{placedFrame(spec.place, beltOf(placed, spec), along)};
    });
    return target;
}

void placeAt(Scene& scene, const Scene& original, Pick pick, const Frame& to) {
    const auto placed = placeConveyors(original);
    if (pick.kind == Pick::Kind::Conveyor) {
        if (pick.index >= scene.conveyors.size() || pick.index >= original.conveyors.size()) {
            return;
        }
        const auto& before = original.conveyors[pick.index].spec;
        auto& spec = scene.conveyors[pick.index].spec;
        // Its heading from where the frame's x points (only turned about the
        // vertical); its start half its length back from the frame, on the
        // floor; its belt at the frame's height.
        const auto x = to.rotation * glm::vec3(1.0F, 0.0F, 0.0F);
        spec.heading = std::atan2(-x.z, x.x);
        const auto start = to.origin - directionOf(spec.heading) * (before.length / 2.0F);
        spec.start = {start.x, before.start.y, start.z};
        spec.beltHeight = std::max(0.05F, to.origin.y);
        return;
    }
    const auto* scenePlace = placementOf(original, pick);
    if (scenePlace == nullptr) {
        return;
    }
    withDevice(original, pick, [&](const auto& before, float along) {
        const auto* belt = beltOf(placed, before);
        auto place = before.place;
        float newAlong = along;
        if (belt != nullptr) {
            // Along the belt: its place along it; the rest: the nudge.
            const auto start = placedFrame(before.place, belt, along);
            const float moved = glm::dot(to.origin - start.origin, directionOf(belt->heading));
            newAlong = std::clamp(along + moved, 0.0F, belt->length);
            place = placementAt(to, Parent::Conveyor, belt, newAlong, place.span, place.height);
        } else {
            place = placementAt(to, Parent::World, nullptr, along, place.span, place.height);
        }
        withDevice(scene, pick, [&](auto& spec, float& a) {
            spec.place = place;
            a = newAlong;
        });
    });
}

void detach(Scene& scene, Pick pick) {
    const auto placed = placeConveyors(scene);
    withDevice(scene, pick, [&](auto& spec, float along) {
        const auto* belt = beltOf(placed, spec);
        if (belt == nullptr) {
            return; // on its own already
        }
        // Worked out from now on on a stand-in as wide and high as its belt.
        spec.place = placementAt(placedFrame(spec.place, belt, along), Parent::World, nullptr, along, belt->width,
                                 belt->beltHeight);
    });
}

void attach(Scene& scene, Pick pick, std::size_t conveyor) {
    const auto placed = placeConveyors(scene);
    if (conveyor >= placed.size()) {
        return;
    }
    const auto& belt = placed[conveyor];
    withDevice(scene, pick, [&](auto& spec, float& along) {
        const auto here = placedFrame(spec.place, beltOf(placed, spec), along);
        // Its usual place on that belt, as near as it can be to where it is.
        const auto start = beltPoint(belt, 0.0F) - kUp * belt.beltHeight;
        along = std::clamp(glm::dot(here.origin - start, directionOf(belt.heading)), 0.0F, belt.length);
        spec.conveyor = conveyor;
        spec.place = Placement{};
    });
}

} // namespace fsim
