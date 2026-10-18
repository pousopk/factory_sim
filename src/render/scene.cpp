#include "render/scene.hpp"
#include <cmath>

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

std::vector<DrawBox> sceneBoxes(const World& world, double ghostSeconds) {
    std::vector<DrawBox> boxes;
    for (const auto& conveyor : world.conveyors()) {
        const auto parts = conveyorBoxes(conveyor);
        boxes.insert(boxes.end(), parts.begin(), parts.end());
    }
    for (const auto& photocell : world.photocells()) {
        if (photocell.spec.conveyor < world.conveyors().size()) {
            const auto parts = photocellBoxes(photocell, world.conveyors()[photocell.spec.conveyor]);
            boxes.insert(boxes.end(), parts.begin(), parts.end());
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

} // namespace fsim
