#pragma once
#include "sim/world.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <optional>
#include <vector>

// What to draw, worked out from the simulation: a list of coloured boxes.
//
// Everything in the scene is a box: a conveyor's belt, its side frames and
// legs, and the items it carries. Turning the World into this list is plain
// geometry, with no OpenGL, so it's tested like the simulation; the renderer
// only draws what the list says.
namespace fsim {

// What a drawn box belongs to, so a click can select it.
struct Pick {
    enum class Kind { None, Conveyor, Photocell, Spawner, Pusher, Diverter, Stopper, Bin };
    Kind kind = Kind::None;
    std::size_t index = 0;

    bool operator==(const Pick&) const = default;
};

struct DrawBox {
    glm::vec3 center;
    glm::quat rotation{1.0F, 0.0F, 0.0F, 0.0F};
    glm::vec3 size; // full size along the box's own x (length), y (height), z (width)
    glm::vec3 color;
    // A moving belt: stripes across it, this far apart (0: no stripes),
    // shifted by `stripeOffset` metres along the box's x, so they move with
    // the belt surface.
    float stripeSpacing = 0.0F;
    float stripeOffset = 0.0F;
    float alpha = 1.0F;     // below 1: see-through (drawn after everything solid)
    bool unlit = false;     // a light source (a beam, an LED): its colour as it is, no shading
    Pick pick{};            // what it belongs to (None: can't be selected)
    bool highlight = false; // part of the selection
};

// The parts of one conveyor: belt, two side frames, legs at both ends.
std::vector<DrawBox> conveyorBoxes(const Conveyor& conveyor);

// A photocell: its housing on one side of the belt, the reflector on the
// other, the beam between them, and the indicator LED, lit while the
// output is on.
std::vector<DrawBox> photocellBoxes(const Photocell& photocell, const Conveyor& conveyor);

// The actuators, at their stroke's current position: a pusher's cylinder,
// rod and paddle (with LEDs for its end position sensors), a diverter's arm
// and pivot, a stopper's blade. A bin's floor and walls.
std::vector<DrawBox> pusherBoxes(const Pusher& pusher, const ConveyorSpec& conveyor);
std::vector<DrawBox> diverterBoxes(const Diverter& diverter, const ConveyorSpec& conveyor);
std::vector<DrawBox> stopperBoxes(const Stopper& stopper, const ConveyorSpec& conveyor);
std::vector<DrawBox> binBoxes(const Bin& bin, const ConveyorSpec& conveyor);

// A spawner: a gate over the belt where its boxes appear.
std::vector<DrawBox> spawnerBoxes(const SpawnerSpec& spawner, const Conveyor& conveyor);

// Everything: every conveyor's parts, the photocells and spawners, every
// item, and a see-through "ghost" of each photocell's last triggered box
// where it was at the output time, for `ghostSeconds` after it. The parts
// of `selected` are highlighted.
std::vector<DrawBox> sceneBoxes(const World& world, double ghostSeconds = 1.0,
                                std::optional<Pick> selected = std::nullopt);

// Where a ray (from `origin`, along the unit vector `direction`) first
// enters a box: the distance along the ray, or nothing if it misses.
std::optional<float> hitDistance(const DrawBox& box, glm::vec3 origin, glm::vec3 direction);

// The selectable box a ray hits first, if any.
std::optional<Pick> pickAt(const std::vector<DrawBox>& boxes, glm::vec3 origin, glm::vec3 direction);

// Sizes of the parts, shared with the tests.
constexpr float kFrameWidth = 0.04F;    // each side frame
constexpr float kFrameHeight = 0.10F;   // from just below the belt surface down
constexpr float kLegSize = 0.05F;       // square legs
constexpr float kStripeSpacing = 0.25F; // metres between belt stripes
constexpr float kHousingSize = 0.05F;   // a photocell's housing (and its reflector)

} // namespace fsim
