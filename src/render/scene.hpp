#pragma once
#include "sim/world.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <optional>
#include <string>
#include <vector>

// What to draw, worked out from the simulation: a list of coloured boxes.
//
// Everything in the scene is a box (a conveyor's belt, its side frames and
// legs, the items it carries), except tins, which are round: a box marked
// `round` is drawn as the cylinder that fits in it, standing on its y axis. Turning the World into this list is plain
// geometry, with no OpenGL, so it's tested like the simulation; the renderer
// only draws what the list says.
namespace fsim {

// What a drawn box belongs to, so a click can select it.
struct Pick {
    enum class Kind {
        None,
        Conveyor,
        Photocell,
        Spawner,
        Pusher,
        Diverter,
        Stopper,
        AirJet,
        Bin,
        Camera,
        Proximity,
        Laser,
        LightCurtain,
        Item // a box or tin on the line: its index is the item's id
    };
    Kind kind = Kind::None;
    std::size_t index = 0;

    bool operator==(const Pick&) const = default;
};

// Whether a kind of thing is a sensor. One placed otherwise than as usual on
// its belt doesn't sense yet (milestone 22): its maths is still in belt terms.
inline bool isSensor(Pick::Kind kind) {
    return kind == Pick::Kind::Photocell || kind == Pick::Kind::Camera || kind == Pick::Kind::Proximity ||
           kind == Pick::Kind::Laser || kind == Pick::Kind::LightCurtain;
}

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
    bool hovered = false;   // under the mouse
    bool round = false;     // a cylinder (a tin): size.x is its diameter, size.y its height
    // Drawn as this item's surface instead of a box, stretched from its own
    // size to `size` (the same size: as it is).
    std::shared_ptr<const ItemGeometry> mesh{};
};

// An item as it's drawn: a box, a tin, or its own surface.
DrawBox itemBox(const ItemView& item);
// A see-through copy of an item where it was at some moment (a photocell's
// trigger, a camera's exposure), a little bigger so it shows around it.
DrawBox ghostBox(glm::vec3 center, glm::quat rotation, glm::vec3 size, Shape shape,
                 std::shared_ptr<const ItemGeometry> geometry, glm::vec3 color, float alpha);

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
// An air jet: its manifold and nozzles, an LED lit while its valve is open,
// and a faint streak of air from each nozzle while it blows.
std::vector<DrawBox> airJetBoxes(const AirJet& jet, const ConveyorSpec& conveyor);
std::vector<DrawBox> binBoxes(const Bin& bin, const ConveyorSpec& conveyor);

// A camera: its body and lens, looking at the belt from where its tilt puts
// it, on a post; its LED, lit while it's busy; and its field of view on the
// belt, which flashes white when it takes a picture and red when it ignores
// a trigger. `now` is the simulated time, for the flashes.
std::vector<DrawBox> cameraBoxes(const Camera& camera, const Conveyor& conveyor, double now);

// A proximity switch: a threaded metal barrel with its face towards the belt
// (or down at it), the LED on its back, and its switching distance drawn as
// a faint disc in front of the face.
std::vector<DrawBox> proximityBoxes(const Proximity& proximity, const Conveyor& conveyor);
// A laser distance sensor over the belt, on a post, and its beam down to
// what it measures, with the red spot there.
std::vector<DrawBox> laserBoxes(const Laser& laser, const Conveyor& conveyor);
// A light curtain: the emitter and receiver columns either side of the belt,
// and its beams across, the broken ones missing.
std::vector<DrawBox> curtainBoxes(const LightCurtain& curtain, const Conveyor& conveyor);

// A spawner: a gate over the belt where its boxes appear.
std::vector<DrawBox> spawnerBoxes(const SpawnerSpec& spawner, const Conveyor& conveyor);

// Everything: every conveyor's parts, the photocells and spawners, every
// item, and a see-through "ghost" of each photocell's last triggered box
// where it was at the output time, for `ghostSeconds` after it (and of
// each camera's last picture, where the item was in it). The parts
// of `selected` are highlighted.
std::vector<DrawBox> sceneBoxes(const World& world, double ghostSeconds = 1.0,
                                std::optional<Pick> selected = std::nullopt,
                                std::optional<Pick> hovered = std::nullopt);

// A device's name, to show over the 3D view at a point above it.
struct Label {
    glm::vec3 position;
    std::string text;
    Pick pick; // what it names
};

// How an area camera sees: from its working distance and tilt, looking at the
// middle of its field of view, the picture's x along the travel, its lens
// angle set by the field of view and the pixels. `width` x `height`: the size
// to draw it at, the camera's shape scaled to at most `maxSize`.
struct CameraView {
    glm::mat4 view{1.0F};
    glm::mat4 projection{1.0F};
    glm::vec3 eye{0.0F};
    int width = 0;
    int height = 0;
};
CameraView cameraView(const CameraSpec& spec, const Conveyor& conveyor, int maxSize = 640);

// A label for every named device in the world: conveyors (over the middle
// of the belt), photocells, proximity switches, lasers, light curtains,
// actuators, bins and cameras, each just above its own parts.
std::vector<Label> sceneLabels(const World& world);

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
