#pragma once
#include "sim/actuators.hpp"
#include "sim/conveyor_spec.hpp"
#include "sim/mounting.hpp"
#include "sim/placement.hpp"
#include "sim/signal.hpp"
#include <deque>
#include <glm/glm.hpp>
#include <string>
#include <vector>

// An air jet: a row of nozzles beside the belt, on one solenoid valve, that
// blows products off the other side. Faster than any cylinder (nothing has
// to move but air), so lines too fast for a pusher reject with jets: bags,
// fish, bottles, anything light.
//
// The valve opens a few milliseconds after its output switches on (the coil
// pulls the armature in), and closes a little later still after it's off
// (the armature drops out and the air in the hose runs out). The air is at a
// fixed pressure: the line's compressed air supply keeps up.
//
// Each nozzle's jet is a narrow cone of air. It's modelled as a few dozen
// thin streams spread over that cone, each carrying an equal share of the
// nozzle's thrust. A stream that hits a product pushes it there, at right
// angles to the surface it hits, by the share of its push that's square to
// that surface (the air slides off along the rest). So a flat face square
// to the jet takes the whole push straight away; a rounded or sloping one,
// like a fish's side, is pushed partly up as well: it lifts. A small
// product further from the nozzle catches fewer streams.
//
//        nozzle   ->  ->  ->   ____
//       [=]=====  ->  ->  ->  |    |   product
//        manifold ->  ->  ->  |____|
namespace fsim {

struct AirJetSpec {
    std::string name = "Y1";
    std::size_t conveyor = 0;
    float along = 2.0F;       // metres from the belt's start, to the middle of the row
    Placement place;          // on its conveyor as usual, nudged, or standing on its own (placement.hpp)
    Side side = Side::Left;   // where it's mounted: it blows towards the other side
    float height = 0.03F;     // the nozzles' height above the belt
    int nozzles = 3;          // in a row along the belt
    float pitch = 0.03F;      // m between nozzles
    float aim = 0.0F;         // radians up from horizontal (negative: down)
    float pressure = 6.0F;    // bar, gauge: what the supply holds
    float bore = 0.002F;      // m, each nozzle's opening
    float spread = 0.2F;      // radians from the jet's axis to its edge (a free jet: about 12 degrees)
    double openTime = 0.004;  // s, from the output switching on to the air flowing
    double closeTime = 0.006; // s, from the output switching off to the air stopping
    std::string valveOutput;  // %QX

    bool operator==(const AirJetSpec&) const = default;
};

// One nozzle's thrust, N: about 1.25 x gauge pressure x the bore's area,
// what blow-off nozzles' datasheets give (a 2 mm nozzle at 6 bar: 2.4 N).
float thrustOf(const AirJetSpec& spec);

// Where each nozzle is, and which way the air leaves it (a unit vector).
struct Nozzle {
    glm::vec3 position;
    glm::vec3 direction;
};
std::vector<Nozzle> nozzlesOf(const AirJetSpec& spec, const ConveyorSpec& conveyor);
// The same where it would usually be on `conveyor`, ignoring its placement.
std::vector<Nozzle> nozzlesOnBelt(const AirJetSpec& spec, const ConveyorSpec& conveyor);

// The streams' directions in one nozzle's cone around `axis`: one along
// it and rings of 6, 12 and 18 around it, so they spread evenly over the
// cone's cross-section.
std::vector<glm::vec3> streamsAround(glm::vec3 axis, float spread);

// The valve and its air, with exact times.
class AirJet {
public:
    explicit AirJet(AirJetSpec settings = {});

    AirJetSpec spec;

    // Switches the valve at `time`: the air follows after the valve's delay.
    void command(double time, bool open);
    bool commanded() const { return open_; }
    // Opens it now and closes it `seconds` later (by hand).
    void blow(double time, double seconds);
    // Moves through one step: returns the share of it the air flowed (0..1).
    double update(double stepStart, double stepEnd);

    const Signal& valve() const { return valve_; } // the output, as commanded
    const Signal& air() const { return air_; }     // the air flowing
    float lastPush = 0.0F;                         // N on products in the last step, all streams together

private:
    bool open_ = false;
    double closeAt_ = -1.0;    // a blow by hand ends here
    std::deque<Edge> pending_; // the air's changes, waiting for their delay, in time order
    Signal valve_;
    Signal air_;
};

} // namespace fsim
