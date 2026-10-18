#include "render/scene.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim {

namespace {

const glm::vec3 kBeltColor{0.18F, 0.19F, 0.21F};  // dark rubber
const glm::vec3 kFrameColor{0.62F, 0.65F, 0.70F}; // aluminium profile
const glm::vec3 kLegColor{0.45F, 0.47F, 0.50F};
const glm::vec3 kHousingColor{0.20F, 0.35F, 0.70F}; // a blue sensor housing
const glm::vec3 kReflectorColor{0.75F, 0.20F, 0.15F};
const glm::vec3 kBeamColor{1.0F, 0.15F, 0.10F};
const glm::vec3 kLedOn{1.0F, 0.85F, 0.10F};
const glm::vec3 kLedOff{0.25F, 0.22F, 0.12F};
const glm::vec3 kGateColor{0.85F, 0.62F, 0.15F};
const glm::vec3 kGuideColor{0.85F, 0.80F, 0.30F};
const glm::vec3 kCylinderColor{0.70F, 0.72F, 0.75F};
const glm::vec3 kPaddleColor{0.20F, 0.45F, 0.75F};
const glm::vec3 kArmColor{0.20F, 0.60F, 0.45F};
const glm::vec3 kBladeColor{0.75F, 0.25F, 0.20F};
const glm::vec3 kBinColor{0.30F, 0.42F, 0.35F};

DrawBox fromPose(const Pose& pose, glm::vec3 color) {
    DrawBox box;
    box.center = pose.center;
    box.rotation = pose.rotation;
    box.size = pose.size;
    box.color = color;
    return box;
}

DrawBox led(glm::vec3 center, glm::quat rotation, bool on) {
    DrawBox box = fromPose(Pose{center, rotation, glm::vec3(0.012F, 0.008F, 0.012F)}, on ? kLedOn : kLedOff);
    box.unlit = true;
    return box;
}

// Gives every box in `parts` an owner, and appends them to `boxes`.
void addParts(std::vector<DrawBox>& boxes, std::vector<DrawBox> parts, Pick pick, const std::optional<Pick>& selected) {
    for (auto& part : parts) {
        part.pick = pick;
        part.highlight = selected == pick;
        boxes.push_back(part);
    }
}

} // namespace

std::vector<DrawBox> conveyorBoxes(const Conveyor& conveyor) {
    const auto& spec = conveyor.spec;
    const auto rotation = glm::angleAxis(spec.heading, glm::vec3(0.0F, 1.0F, 0.0F));
    const auto along = conveyor.direction();
    const glm::vec3 side{along.z, 0.0F, -along.x}; // horizontal, at right angles to the travel
    const auto middle = conveyor.beltPoint(spec.length / 2.0F);
    constexpr float beltThickness = 0.02F;

    std::vector<DrawBox> boxes;
    // The belt itself: its top surface exactly at beltHeight, with stripes
    // that move as far as the belt surface has travelled.
    boxes.push_back(DrawBox{middle - glm::vec3(0.0F, beltThickness / 2.0F, 0.0F),
                            rotation,
                            {spec.length, beltThickness, spec.width},
                            kBeltColor,
                            kStripeSpacing,
                            static_cast<float>(std::fmod(conveyor.travelled, kStripeSpacing))});

    // Side frames, just outside the belt, their top a little above it so
    // boxes look guided.
    const float frameTop = spec.beltHeight + 0.02F;
    for (const float sign : {-1.0F, 1.0F}) {
        const auto offset = side * sign * (spec.width / 2.0F + kFrameWidth / 2.0F);
        boxes.push_back(DrawBox{glm::vec3(middle.x, frameTop - kFrameHeight / 2.0F, middle.z) + offset,
                                rotation,
                                {spec.length, kFrameHeight, kFrameWidth},
                                kFrameColor});
    }

    // Side guides, where the physics has them: rails along the belt's edges.
    const auto left = leftOf(along);
    for (const auto& [wanted, sign] : {std::pair{spec.guideLeft, 1.0F}, std::pair{spec.guideRight, -1.0F}}) {
        if (wanted) {
            boxes.push_back(fromPose(Pose{middle + left * sign * (spec.width / 2.0F + kGuideThickness / 2.0F) +
                                              glm::vec3(0.0F, kGuideHeight / 2.0F, 0.0F),
                                          rotation, glm::vec3(spec.length, kGuideHeight, kGuideThickness)},
                                     kGuideColor));
        }
    }

    // Four legs, from the floor to the frame, near the ends.
    const float legHeight = frameTop - kFrameHeight;
    for (const float at : {kLegSize, spec.length - kLegSize}) {
        const auto base = spec.start + along * at;
        for (const float sign : {-1.0F, 1.0F}) {
            const auto offset = side * sign * (spec.width / 2.0F + kFrameWidth / 2.0F);
            boxes.push_back(DrawBox{base + offset + glm::vec3(0.0F, legHeight / 2.0F, 0.0F),
                                    rotation,
                                    {kLegSize, legHeight, kLegSize},
                                    kLegColor});
        }
    }
    return boxes;
}

std::vector<DrawBox> photocellBoxes(const Photocell& photocell, const Conveyor& conveyor) {
    const auto& spec = photocell.spec;
    const auto rotation = glm::angleAxis(conveyor.spec.heading, glm::vec3(0.0F, 1.0F, 0.0F));
    const auto along = conveyor.direction();
    const glm::vec3 side{along.z, 0.0F, -along.x};
    const auto beamCentre = conveyor.beltPoint(spec.along) + glm::vec3(0.0F, spec.height, 0.0F);
    const float halfSpan = conveyor.spec.width / 2.0F + kFrameWidth + kHousingSize / 2.0F;

    std::vector<DrawBox> boxes;
    const auto housing = beamCentre + side * halfSpan;
    boxes.push_back(
        DrawBox{housing, rotation, glm::vec3(kHousingSize, kHousingSize * 1.6F, kHousingSize), kHousingColor});
    boxes.push_back(DrawBox{beamCentre - side * halfSpan, rotation,
                            glm::vec3(kHousingSize * 0.4F, kHousingSize, kHousingSize * 0.3F), kReflectorColor});
    // The beam: as wide as its real diameter (but at least 2 mm, to be visible).
    const float thickness = std::max(spec.beamDiameter, 0.002F);
    boxes.push_back(DrawBox{beamCentre, rotation, glm::vec3(thickness, thickness, 2.0F * halfSpan - kHousingSize),
                            kBeamColor, 0.0F, 0.0F, 0.55F, true});
    // The indicator LED on top of the housing.
    boxes.push_back(DrawBox{housing + glm::vec3(0.0F, kHousingSize * 0.8F + 0.004F, 0.0F), rotation,
                            glm::vec3(0.012F, 0.008F, 0.012F), photocell.output.value() ? kLedOn : kLedOff, 0.0F, 0.0F,
                            1.0F, true});
    return boxes;
}

std::vector<DrawBox> pusherBoxes(const Pusher& pusher, const ConveyorSpec& conveyor) {
    const auto paddle = pusherPaddle(pusher.spec, conveyor, pusher.stroke.position());
    const auto rest = pusherPaddle(pusher.spec, conveyor, 0.0);
    const auto out = glm::normalize(rest.center - beltPoint(conveyor, pusher.spec.along) -
                                    glm::vec3(0.0F, rest.center.y - conveyor.beltHeight, 0.0F));
    // The cylinder stands outside the frame, behind the paddle's rest place.
    constexpr float cylinderLength = 0.25F;
    const auto cylinder = rest.center + out * (0.06F + cylinderLength / 2.0F);
    const auto rotation = paddle.rotation;
    std::vector<DrawBox> boxes;
    boxes.push_back(fromPose(Pose{cylinder, rotation, {0.07F, 0.07F, cylinderLength}}, kCylinderColor));
    // The rod, from the cylinder to the paddle.
    const auto rodStart = cylinder - out * (cylinderLength / 2.0F);
    const float rodLength = glm::length(rodStart - paddle.center);
    boxes.push_back(
        fromPose(Pose{(rodStart + paddle.center) / 2.0F, rotation, {0.02F, 0.02F, rodLength}}, kCylinderColor));
    boxes.push_back(fromPose(paddle, kPaddleColor));
    // The end position sensors' LEDs: retracted at the back, extended at the front.
    const glm::vec3 up{0.0F, 0.04F, 0.0F};
    boxes.push_back(
        led(cylinder + out * (cylinderLength / 2.0F - 0.02F) + up, rotation, pusher.stroke.retracted().value()));
    boxes.push_back(
        led(cylinder - out * (cylinderLength / 2.0F - 0.02F) + up, rotation, pusher.stroke.extended().value()));
    return boxes;
}

std::vector<DrawBox> diverterBoxes(const Diverter& diverter, const ConveyorSpec& conveyor) {
    const auto arm = diverterArm(diverter.spec, conveyor, diverter.stroke.position());
    const auto rest = diverterArm(diverter.spec, conveyor, 0.0);
    // The pivot: a post at the arm's upstream end, standing on the frame.
    const auto pivot = rest.center - directionOf(conveyor.heading) * (rest.size.x / 2.0F);
    std::vector<DrawBox> boxes;
    boxes.push_back(
        fromPose(Pose{glm::vec3(pivot.x, conveyor.beltHeight + 0.06F, pivot.z), rest.rotation, {0.05F, 0.12F, 0.05F}},
                 kCylinderColor));
    boxes.push_back(fromPose(arm, kArmColor));
    boxes.push_back(led(glm::vec3(pivot.x, conveyor.beltHeight + 0.125F, pivot.z), rest.rotation,
                        diverter.stroke.extended().value()));
    return boxes;
}

std::vector<DrawBox> stopperBoxes(const Stopper& stopper, const ConveyorSpec& conveyor) {
    return {fromPose(stopperBlade(stopper.spec, conveyor, stopper.stroke.position()), kBladeColor)};
}

std::vector<DrawBox> binBoxes(const Bin& bin, const ConveyorSpec& conveyor) {
    std::vector<DrawBox> boxes;
    for (const auto& wall : binWalls(bin.spec, conveyor)) {
        boxes.push_back(fromPose(wall, kBinColor));
    }
    return boxes;
}

std::vector<DrawBox> spawnerBoxes(const SpawnerSpec& spawner, const Conveyor& conveyor) {
    const auto rotation = glm::angleAxis(conveyor.spec.heading, glm::vec3(0.0F, 1.0F, 0.0F));
    const auto along = conveyor.direction();
    const glm::vec3 side{along.z, 0.0F, -along.x};
    const auto base = conveyor.beltPoint(spawner.offset);
    const float clearance = std::max(0.35F, spawner.size.y + 0.1F); // high enough for its boxes
    const float halfSpan = conveyor.spec.width / 2.0F + kFrameWidth;
    constexpr float bar = 0.03F;
    std::vector<DrawBox> boxes;
    for (const float sign : {-1.0F, 1.0F}) {
        boxes.push_back(DrawBox{base + side * sign * halfSpan + glm::vec3(0.0F, clearance / 2.0F, 0.0F), rotation,
                                glm::vec3(bar, clearance, bar), kGateColor});
    }
    boxes.push_back(DrawBox{base + glm::vec3(0.0F, clearance, 0.0F), rotation,
                            glm::vec3(bar, bar, 2.0F * halfSpan + bar), kGateColor});
    return boxes;
}

std::vector<DrawBox> sceneBoxes(const World& world, double ghostSeconds, std::optional<Pick> selected) {
    std::vector<DrawBox> boxes;
    const auto& conveyors = world.conveyors();
    for (std::size_t i = 0; i < conveyors.size(); ++i) {
        addParts(boxes, conveyorBoxes(conveyors[i]), Pick{Pick::Kind::Conveyor, i}, selected);
    }
    for (std::size_t i = 0; i < world.photocells().size(); ++i) {
        const auto& photocell = world.photocells()[i];
        if (photocell.spec.conveyor < conveyors.size()) {
            addParts(boxes, photocellBoxes(photocell, conveyors[photocell.spec.conveyor]),
                     Pick{Pick::Kind::Photocell, i}, selected);
        }
    }
    const auto onConveyor = [&](std::size_t conveyor) { return conveyor < conveyors.size(); };
    for (std::size_t i = 0; i < world.pushers().size(); ++i) {
        const auto& a = world.pushers()[i];
        if (onConveyor(a.spec.conveyor)) {
            addParts(boxes, pusherBoxes(a, conveyors[a.spec.conveyor].spec), Pick{Pick::Kind::Pusher, i}, selected);
        }
    }
    for (std::size_t i = 0; i < world.diverters().size(); ++i) {
        const auto& a = world.diverters()[i];
        if (onConveyor(a.spec.conveyor)) {
            addParts(boxes, diverterBoxes(a, conveyors[a.spec.conveyor].spec), Pick{Pick::Kind::Diverter, i}, selected);
        }
    }
    for (std::size_t i = 0; i < world.stoppers().size(); ++i) {
        const auto& a = world.stoppers()[i];
        if (onConveyor(a.spec.conveyor)) {
            addParts(boxes, stopperBoxes(a, conveyors[a.spec.conveyor].spec), Pick{Pick::Kind::Stopper, i}, selected);
        }
    }
    for (std::size_t i = 0; i < world.bins().size(); ++i) {
        const auto& b = world.bins()[i];
        if (onConveyor(b.spec.conveyor)) {
            addParts(boxes, binBoxes(b, conveyors[b.spec.conveyor].spec), Pick{Pick::Kind::Bin, i}, selected);
        }
    }
    for (std::size_t i = 0; i < world.spawners().size(); ++i) {
        const auto& spawner = world.spawners()[i];
        if (spawner.conveyor < conveyors.size()) {
            addParts(boxes, spawnerBoxes(spawner, conveyors[spawner.conveyor]), Pick{Pick::Kind::Spawner, i}, selected);
        }
    }
    for (const auto& item : world.items()) {
        boxes.push_back(DrawBox{item.position, item.rotation, item.size, item.color});
    }
    for (const auto& photocell : world.photocells()) {
        if (!photocell.triggers.empty()) {
            const auto& trigger = photocell.triggers.front();
            if (world.time() - trigger.outputTime < ghostSeconds && trigger.itemSize != glm::vec3(0.0F)) {
                boxes.push_back(DrawBox{trigger.itemCenter,
                                        trigger.itemRotation,
                                        trigger.itemSize + glm::vec3(0.002F),
                                        {1.0F, 1.0F, 1.0F},
                                        0.0F,
                                        0.0F,
                                        0.35F});
            }
        }
    }
    return boxes;
}

std::optional<float> hitDistance(const DrawBox& box, glm::vec3 origin, glm::vec3 direction) {
    // In the box's own axes the box is axis-aligned, from -half to +half:
    // the ray enters it at the latest of the three entry points and leaves
    // at the earliest exit (the "slab" method).
    const auto inverse = glm::conjugate(box.rotation);
    const auto o = inverse * (origin - box.center);
    const auto d = inverse * direction;
    const auto half = box.size / 2.0F;
    float enter = -std::numeric_limits<float>::max();
    float leave = std::numeric_limits<float>::max();
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(d[axis]) < 1e-9F) {
            if (std::abs(o[axis]) > half[axis]) {
                return std::nullopt; // parallel to this pair of faces, and outside them
            }
            continue;
        }
        float a = (-half[axis] - o[axis]) / d[axis];
        float b = (half[axis] - o[axis]) / d[axis];
        if (a > b) {
            std::swap(a, b);
        }
        enter = std::max(enter, a);
        leave = std::min(leave, b);
    }
    if (enter > leave || leave < 0.0F) {
        return std::nullopt;
    }
    return std::max(enter, 0.0F);
}

std::optional<Pick> pickAt(const std::vector<DrawBox>& boxes, glm::vec3 origin, glm::vec3 direction) {
    std::optional<Pick> best;
    float nearest = std::numeric_limits<float>::max();
    for (const auto& box : boxes) {
        if (box.pick.kind == Pick::Kind::None) {
            continue;
        }
        if (const auto distance = hitDistance(box, origin, direction); distance && *distance < nearest) {
            nearest = *distance;
            best = box.pick;
        }
    }
    return best;
}

} // namespace fsim
