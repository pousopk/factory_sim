#include "render/scene.hpp"
#include <cmath>

namespace fsim {

namespace {

const glm::vec3 kBeltColor{0.18F, 0.19F, 0.21F};  // dark rubber
const glm::vec3 kFrameColor{0.62F, 0.65F, 0.70F}; // aluminium profile
const glm::vec3 kLegColor{0.45F, 0.47F, 0.50F};

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

std::vector<DrawBox> sceneBoxes(const World& world) {
    std::vector<DrawBox> boxes;
    for (const auto& conveyor : world.conveyors()) {
        const auto parts = conveyorBoxes(conveyor);
        boxes.insert(boxes.end(), parts.begin(), parts.end());
    }
    for (const auto& item : world.items()) {
        boxes.push_back(DrawBox{item.position, item.rotation, item.size, item.color});
    }
    return boxes;
}

} // namespace fsim
