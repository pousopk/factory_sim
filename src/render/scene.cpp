#include "render/scene.hpp"
#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
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
const glm::vec3 kCameraColor{0.15F, 0.16F, 0.18F};
const glm::vec3 kLensColor{0.05F, 0.05F, 0.06F};
const glm::vec3 kViewColor{0.35F, 0.60F, 1.00F};
const glm::vec3 kPostColor{0.55F, 0.57F, 0.60F};

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
void addParts(std::vector<DrawBox>& boxes, std::vector<DrawBox> parts, Pick pick, const std::optional<Pick>& selected,
              const std::optional<Pick>& hovered = std::nullopt) {
    for (auto& part : parts) {
        part.pick = pick;
        part.highlight = selected == pick;
        part.hovered = hovered == pick;
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

    // An accumulation conveyor's zones: a dark line across the belt where each
    // one ends (the eyes are drawn as photocells).
    for (int z = 1; z < spec.zones; ++z) {
        const float at = spec.length * static_cast<float>(z) / static_cast<float>(spec.zones);
        boxes.push_back(DrawBox{conveyor.beltPoint(at) + glm::vec3(0.0F, 0.001F, 0.0F),
                                rotation,
                                {0.012F, 0.002F, spec.width},
                                {0.05F, 0.05F, 0.06F}});
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

std::vector<DrawBox> cameraBoxes(const Camera& camera, const Conveyor& conveyor, double now) {
    const auto& spec = camera.spec;
    const auto along = conveyor.direction();
    const auto left = leftOf(along);
    const auto target = conveyor.beltPoint(spec.along); // the middle of the picture, on the belt
    // Looking at the target from its working distance, turned by the tilt
    // around the direction of travel: straight down at 0, from the left above 0.
    const float distance = spec.workingDistance;
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const auto toCamera = up * std::cos(spec.tilt) + left * std::sin(spec.tilt);
    const auto body = target + toCamera * distance;
    // The body's axes: x along the travel, y towards the belt's centre line.
    const auto rotation = glm::quat_cast(glm::mat3(along, toCamera, glm::cross(along, toCamera)));
    const auto heading = glm::angleAxis(conveyor.spec.heading, up);

    std::vector<DrawBox> boxes;
    boxes.push_back(DrawBox{body, rotation, glm::vec3(0.07F, 0.10F, 0.06F), kCameraColor});
    boxes.push_back(DrawBox{body - toCamera * 0.065F, rotation, glm::vec3(0.035F, 0.03F, 0.035F), kLensColor});
    // The LED on its back: lit while busy.
    boxes.push_back(DrawBox{body + toCamera * 0.052F, rotation, glm::vec3(0.012F, 0.006F, 0.012F),
                            camera.busy.value() ? kLedOn : kLedOff, 0.0F, 0.0F, 1.0F, true});
    // A post beside the belt, up to the camera's height, and an arm across to it.
    const float outward = std::abs(spec.tilt) < 0.3F ? 1.0F : (spec.tilt > 0.0F ? 1.0F : -1.0F);
    const auto foot = conveyor.beltPoint(spec.along) + left * outward * (conveyor.spec.width / 2.0F + 0.12F);
    const float top = body.y + 0.06F;
    boxes.push_back(DrawBox{glm::vec3(foot.x, top / 2.0F, foot.z), heading, glm::vec3(0.03F, top, 0.03F), kPostColor});
    const auto armEnd = glm::vec3(body.x, top, body.z);
    const auto armStart = glm::vec3(foot.x, top, foot.z);
    const float armLength = glm::length(armEnd - armStart);
    if (armLength > 0.01F) {
        boxes.push_back(
            DrawBox{(armStart + armEnd) / 2.0F, heading, glm::vec3(0.025F, 0.025F, armLength + 0.03F), kPostColor});
    }
    // What it sees: a see-through patch on the belt.
    auto color = kViewColor;
    float alpha = 0.20F;
    if (!camera.shots.empty()) {
        const auto& last = camera.shots.front();
        const double since = now - last.exposure;
        if (last.taken && since < 0.12) {
            color = glm::vec3(1.0F);
            alpha = 0.55F;
        } else if (!last.taken && since < 0.35) {
            color = glm::vec3(1.0F, 0.15F, 0.10F);
            alpha = 0.55F;
        }
    }
    // A line-scan camera sees a thin line across the belt.
    const auto view = spec.kind == CameraKind::Line ? glm::vec3(0.004F, 0.002F, spec.lineWidth)
                                                    : glm::vec3(spec.fieldOfView, 0.002F, conveyor.spec.width);
    boxes.push_back(DrawBox{target + glm::vec3(0.0F, 0.002F, 0.0F), heading, view, color, 0.0F, 0.0F,
                            spec.kind == CameraKind::Line ? 0.8F : alpha, true});
    return boxes;
}

std::vector<DrawBox> proximityBoxes(const Proximity& proximity, const Conveyor& conveyor) {
    const auto& spec = proximity.spec;
    const auto along = conveyor.direction();
    const auto left = leftOf(along);
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const auto heading = glm::angleAxis(conveyor.spec.heading, up);
    // The face, and the direction the barrel points from it (away from the belt).
    glm::vec3 face;
    glm::vec3 back;
    switch (spec.mount) {
    case Mount::Above:
        face = conveyor.beltPoint(spec.along) + up * spec.height;
        back = up;
        break;
    case Mount::Left:
        face = conveyor.beltPoint(spec.along) + left * (conveyor.spec.width / 2.0F - spec.inset) + up * spec.height;
        back = left;
        break;
    case Mount::Right:
        face = conveyor.beltPoint(spec.along) - left * (conveyor.spec.width / 2.0F - spec.inset) + up * spec.height;
        back = -left;
        break;
    }
    // A cylinder's own axis is y: turn it onto `back`.
    const auto axis = spec.mount == Mount::Above ? heading : heading * glm::angleAxis(1.5708F, glm::vec3(1, 0, 0));
    constexpr float length = 0.07F;
    std::vector<DrawBox> boxes;
    DrawBox barrel{face + back * (length / 2.0F), axis, glm::vec3(spec.faceDiameter, length, spec.faceDiameter),
                   spec.inductive ? glm::vec3(0.72F, 0.74F, 0.78F) : glm::vec3(0.30F, 0.32F, 0.36F)};
    barrel.round = true;
    boxes.push_back(barrel);
    DrawBox light{face + back * (length + 0.003F),
                  axis,
                  glm::vec3(0.008F, 0.006F, 0.008F),
                  proximity.detector.output.value() ? kLedOn : kLedOff,
                  0.0F,
                  0.0F,
                  1.0F,
                  true};
    light.round = true;
    boxes.push_back(light);
    // Its reach for metal, as a faint disc.
    DrawBox zone{face - back * (spec.range / 2.0F),
                 axis,
                 glm::vec3(spec.faceDiameter, spec.range, spec.faceDiameter),
                 {0.4F, 0.8F, 1.0F},
                 0.0F,
                 0.0F,
                 0.18F,
                 true};
    zone.round = true;
    boxes.push_back(zone);
    return boxes;
}

std::vector<DrawBox> laserBoxes(const Laser& laser, const Conveyor& conveyor) {
    const auto& spec = laser.spec;
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const auto heading = glm::angleAxis(conveyor.spec.heading, up);
    const auto left = leftOf(conveyor.direction());
    const auto spot = conveyor.beltPoint(spec.along) + left * spec.across;
    const auto body = spot + up * (spec.mountHeight + 0.04F);
    std::vector<DrawBox> boxes;
    boxes.push_back(DrawBox{body, heading, glm::vec3(0.06F, 0.08F, 0.05F), {0.85F, 0.65F, 0.15F}});
    // A post beside the belt and an arm over it.
    const auto foot = conveyor.beltPoint(spec.along) + left * (conveyor.spec.width / 2.0F + 0.12F);
    const float top = body.y + 0.05F;
    boxes.push_back(DrawBox{glm::vec3(foot.x, top / 2.0F, foot.z), heading, glm::vec3(0.03F, top, 0.03F), kPostColor});
    const auto armStart = glm::vec3(foot.x, top, foot.z);
    const auto armEnd = glm::vec3(body.x, top, body.z);
    boxes.push_back(DrawBox{(armStart + armEnd) / 2.0F, heading,
                            glm::vec3(0.025F, 0.025F, glm::length(armEnd - armStart) + 0.03F), kPostColor});
    // The beam, down to what it sees (the true surface), and the spot there.
    if (laser.trueDistance > 0.0F) {
        const auto hit = spot + up * (spec.mountHeight - laser.trueDistance);
        boxes.push_back(DrawBox{(hit + body) / 2.0F, heading, glm::vec3(0.002F, glm::length(body - hit), 0.002F),
                                kBeamColor, 0.0F, 0.0F, 0.5F, true});
        boxes.push_back(
            DrawBox{hit + up * 0.001F, heading, glm::vec3(0.006F, 0.002F, 0.006F), kBeamColor, 0.0F, 0.0F, 1.0F, true});
    }
    return boxes;
}

std::vector<DrawBox> curtainBoxes(const LightCurtain& curtain, const Conveyor& conveyor) {
    const auto& spec = curtain.spec;
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const auto heading = glm::angleAxis(conveyor.spec.heading, up);
    const auto left = leftOf(conveyor.direction());
    const auto base = conveyor.beltPoint(spec.along);
    const float halfSpan = conveyor.spec.width / 2.0F + kFrameWidth + 0.02F;
    const float height = spec.lowest + static_cast<float>(std::max(spec.beams - 1, 0)) * spec.pitch + 0.03F;
    std::vector<DrawBox> boxes;
    for (const float sign : {-1.0F, 1.0F}) {
        boxes.push_back(DrawBox{base + left * sign * halfSpan + up * (height / 2.0F), heading,
                                glm::vec3(0.03F, height, 0.03F), sign > 0.0F ? kHousingColor : glm::vec3(0.2F)});
    }
    boxes.push_back(DrawBox{base + left * halfSpan + up * (height + 0.006F), heading, glm::vec3(0.012F, 0.008F, 0.012F),
                            curtain.clear.value() ? kLedOn : kLedOff, 0.0F, 0.0F, 1.0F, true});
    for (int b = 0; b < spec.beams; ++b) {
        const bool broken =
            static_cast<std::size_t>(b) < curtain.broken.size() && curtain.broken[static_cast<std::size_t>(b)];
        if (broken) {
            continue; // the light doesn't get through
        }
        boxes.push_back(DrawBox{base + up * (spec.lowest + static_cast<float>(b) * spec.pitch), heading,
                                glm::vec3(0.002F, 0.002F, 2.0F * halfSpan - 0.03F), kBeamColor, 0.0F, 0.0F, 0.4F,
                                true});
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

std::vector<DrawBox> sceneBoxes(const World& world, double ghostSeconds, std::optional<Pick> selected,
                                std::optional<Pick> hovered) {
    std::vector<DrawBox> boxes;
    const auto& conveyors = world.conveyors();
    for (std::size_t i = 0; i < conveyors.size(); ++i) {
        addParts(boxes, conveyorBoxes(conveyors[i]), Pick{Pick::Kind::Conveyor, i}, selected, hovered);
        for (const auto& eye : conveyors[i].eyes) {
            addParts(boxes, photocellBoxes(eye, conveyors[i]), Pick{Pick::Kind::Conveyor, i}, selected, hovered);
        }
    }
    for (std::size_t i = 0; i < world.photocells().size(); ++i) {
        const auto& photocell = world.photocells()[i];
        if (photocell.spec.conveyor < conveyors.size()) {
            addParts(boxes, photocellBoxes(photocell, conveyors[photocell.spec.conveyor]),
                     Pick{Pick::Kind::Photocell, i}, selected, hovered);
        }
    }
    const auto onConveyor = [&](std::size_t conveyor) { return conveyor < conveyors.size(); };
    for (std::size_t i = 0; i < world.pushers().size(); ++i) {
        const auto& a = world.pushers()[i];
        if (onConveyor(a.spec.conveyor)) {
            addParts(boxes, pusherBoxes(a, conveyors[a.spec.conveyor].spec), Pick{Pick::Kind::Pusher, i}, selected,
                     hovered);
        }
    }
    for (std::size_t i = 0; i < world.diverters().size(); ++i) {
        const auto& a = world.diverters()[i];
        if (onConveyor(a.spec.conveyor)) {
            addParts(boxes, diverterBoxes(a, conveyors[a.spec.conveyor].spec), Pick{Pick::Kind::Diverter, i}, selected,
                     hovered);
        }
    }
    for (std::size_t i = 0; i < world.stoppers().size(); ++i) {
        const auto& a = world.stoppers()[i];
        if (onConveyor(a.spec.conveyor)) {
            addParts(boxes, stopperBoxes(a, conveyors[a.spec.conveyor].spec), Pick{Pick::Kind::Stopper, i}, selected,
                     hovered);
        }
    }
    for (std::size_t i = 0; i < world.bins().size(); ++i) {
        const auto& b = world.bins()[i];
        if (onConveyor(b.spec.conveyor)) {
            addParts(boxes, binBoxes(b, conveyors[b.spec.conveyor].spec), Pick{Pick::Kind::Bin, i}, selected, hovered);
        }
    }
    for (std::size_t i = 0; i < world.spawners().size(); ++i) {
        const auto& spawner = world.spawners()[i];
        if (spawner.conveyor < conveyors.size()) {
            addParts(boxes, spawnerBoxes(spawner, conveyors[spawner.conveyor]), Pick{Pick::Kind::Spawner, i}, selected,
                     hovered);
        }
    }
    for (std::size_t i = 0; i < world.cameras().size(); ++i) {
        const auto& camera = world.cameras()[i];
        if (onConveyor(camera.spec.conveyor)) {
            addParts(boxes, cameraBoxes(camera, conveyors[camera.spec.conveyor], world.time()),
                     Pick{Pick::Kind::Camera, i}, selected, hovered);
        }
    }
    for (std::size_t i = 0; i < world.proximities().size(); ++i) {
        const auto& p = world.proximities()[i];
        if (onConveyor(p.spec.conveyor)) {
            addParts(boxes, proximityBoxes(p, conveyors[p.spec.conveyor]), Pick{Pick::Kind::Proximity, i}, selected,
                     hovered);
        }
    }
    for (std::size_t i = 0; i < world.lasers().size(); ++i) {
        const auto& l = world.lasers()[i];
        if (onConveyor(l.spec.conveyor)) {
            addParts(boxes, laserBoxes(l, conveyors[l.spec.conveyor]), Pick{Pick::Kind::Laser, i}, selected, hovered);
        }
    }
    for (std::size_t i = 0; i < world.curtains().size(); ++i) {
        const auto& c = world.curtains()[i];
        if (onConveyor(c.spec.conveyor)) {
            addParts(boxes, curtainBoxes(c, conveyors[c.spec.conveyor]), Pick{Pick::Kind::LightCurtain, i}, selected,
                     hovered);
        }
    }
    for (const auto& item : world.items()) {
        auto box = itemBox(item);
        box.pick = Pick{Pick::Kind::Item, static_cast<std::size_t>(item.id)};
        box.highlight = selected == box.pick;
        box.hovered = hovered == box.pick;
        boxes.push_back(box);
    }
    for (const auto& camera : world.cameras()) {
        if (!camera.shots.empty()) {
            const auto& shot = camera.shots.front();
            if (shot.taken && shot.item != 0 && world.time() - shot.exposure < ghostSeconds) {
                boxes.push_back(ghostBox(shot.itemCenter, shot.itemRotation, shot.itemSize, shot.itemShape,
                                         shot.itemGeometry, {0.6F, 0.8F, 1.0F}, 0.30F));
            }
        }
    }
    for (const auto& photocell : world.photocells()) {
        if (!photocell.triggers.empty()) {
            const auto& trigger = photocell.triggers.front();
            if (world.time() - trigger.outputTime < ghostSeconds && trigger.itemSize != glm::vec3(0.0F)) {
                boxes.push_back(ghostBox(trigger.itemCenter, trigger.itemRotation, trigger.itemSize, trigger.itemShape,
                                         trigger.itemGeometry, {1.0F, 1.0F, 1.0F}, 0.35F));
            }
        }
    }
    return boxes;
}

DrawBox itemBox(const ItemView& item) {
    DrawBox box{item.position, item.rotation, item.size, item.color};
    box.round = item.shape == Shape::Tin;
    box.mesh = item.geometry;
    return box;
}

DrawBox ghostBox(glm::vec3 center, glm::quat rotation, glm::vec3 size, Shape shape,
                 std::shared_ptr<const ItemGeometry> geometry, glm::vec3 color, float alpha) {
    DrawBox ghost{center, rotation, size + glm::vec3(0.002F), color, 0.0F, 0.0F, alpha};
    ghost.round = shape == Shape::Tin;
    ghost.mesh = std::move(geometry);
    return ghost;
}

CameraView cameraView(const CameraSpec& spec, const Conveyor& conveyor, int maxSize) {
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const auto along = conveyor.direction();
    const auto left = leftOf(along);
    const auto target = conveyor.beltPoint(spec.along);
    const auto toCamera = up * std::cos(spec.tilt) + left * std::sin(spec.tilt);
    CameraView view;
    view.eye = target + toCamera * spec.workingDistance;
    // The picture's x runs along the travel, so its "up" is the belt's left
    // (turned with the camera).
    const auto pictureUp = glm::normalize(glm::cross(toCamera, along));
    view.view = glm::lookAt(view.eye, target, pictureUp);
    // The lens: what the field of view (along) and the pixels make across,
    // seen from the working distance.
    const float across = spec.fieldOfView * static_cast<float>(spec.pixelsAcross) / static_cast<float>(spec.pixels);
    const float angle = 2.0F * std::atan(across / 2.0F / spec.workingDistance);
    const float aspect = static_cast<float>(spec.pixels) / static_cast<float>(spec.pixelsAcross);
    view.projection = glm::perspective(angle, aspect, 0.01F, 20.0F);
    // Drawn at most `maxSize` pixels wide or high, keeping the camera's shape.
    const float scale =
        std::min(1.0F, static_cast<float>(maxSize) / static_cast<float>(std::max(spec.pixels, spec.pixelsAcross)));
    view.width = std::max(1, static_cast<int>(std::lround(static_cast<float>(spec.pixels) * scale)));
    view.height = std::max(1, static_cast<int>(std::lround(static_cast<float>(spec.pixelsAcross) * scale)));
    return view;
}

std::vector<Label> sceneLabels(const World& world) {
    std::vector<Label> labels;
    const auto& conveyors = world.conveyors();
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const auto on = [&](std::size_t conveyor) { return conveyor < conveyors.size(); };
    // A point `height` above the belt at `along`, off to the side by `side` (left positive).
    const auto above = [&](std::size_t conveyor, float along, float height, float side = 0.0F) {
        const auto& c = conveyors[conveyor];
        return c.beltPoint(along) + up * height + leftOf(c.direction()) * side;
    };
    for (std::size_t i = 0; i < conveyors.size(); ++i) {
        labels.push_back(Label{above(i, conveyors[i].spec.length / 2.0F, 0.0F, -conveyors[i].spec.width / 2.0F - 0.1F),
                               conveyors[i].spec.name, Pick{Pick::Kind::Conveyor, i}});
    }
    for (std::size_t i = 0; i < world.photocells().size(); ++i) {
        const auto& p = world.photocells()[i].spec;
        if (on(p.conveyor)) {
            const float side = conveyors[p.conveyor].spec.width / 2.0F + kFrameWidth + kHousingSize;
            labels.push_back(
                Label{above(p.conveyor, p.along, p.height + 0.08F, side), p.name, Pick{Pick::Kind::Photocell, i}});
        }
    }
    for (std::size_t i = 0; i < world.proximities().size(); ++i) {
        const auto& p = world.proximities()[i].spec;
        if (on(p.conveyor)) {
            labels.push_back(
                Label{above(p.conveyor, p.along, p.height + 0.06F), p.name, Pick{Pick::Kind::Proximity, i}});
        }
    }
    for (std::size_t i = 0; i < world.lasers().size(); ++i) {
        const auto& l = world.lasers()[i].spec;
        if (on(l.conveyor)) {
            labels.push_back(
                Label{above(l.conveyor, l.along, l.mountHeight + 0.15F, l.across), l.name, Pick{Pick::Kind::Laser, i}});
        }
    }
    for (std::size_t i = 0; i < world.curtains().size(); ++i) {
        const auto& c = world.curtains()[i].spec;
        if (on(c.conveyor)) {
            const float top = c.lowest + static_cast<float>(std::max(c.beams - 1, 0)) * c.pitch + 0.08F;
            labels.push_back(Label{above(c.conveyor, c.along, top), c.name, Pick{Pick::Kind::LightCurtain, i}});
        }
    }
    const auto actuator = [&](const auto& list, Pick::Kind kind) {
        for (std::size_t i = 0; i < list.size(); ++i) {
            const auto& spec = list[i].spec;
            if (on(spec.conveyor)) {
                labels.push_back(Label{above(spec.conveyor, spec.along, 0.25F), spec.name, Pick{kind, i}});
            }
        }
    };
    actuator(world.pushers(), Pick::Kind::Pusher);
    actuator(world.diverters(), Pick::Kind::Diverter);
    actuator(world.stoppers(), Pick::Kind::Stopper);
    actuator(world.bins(), Pick::Kind::Bin);
    for (std::size_t i = 0; i < world.cameras().size(); ++i) {
        const auto& c = world.cameras()[i].spec;
        if (on(c.conveyor)) {
            labels.push_back(
                Label{above(c.conveyor, c.along, 0.0F) +
                          (up * std::cos(c.tilt) + leftOf(conveyors[c.conveyor].direction()) * std::sin(c.tilt)) * 0.5F,
                      c.name, Pick{Pick::Kind::Camera, i}});
        }
    }
    return labels;
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
