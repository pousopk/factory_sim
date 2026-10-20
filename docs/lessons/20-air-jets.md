# Lesson 20: Air jets

A pusher is a cylinder: it needs tens of milliseconds to get going and a few hundred to go out and back. On a fast line, or with light products, rejects are blown off with **compressed air** instead: a row of nozzles beside the belt on one solenoid valve. Nothing moves but air, so a jet can hit one product and miss the next one a few centimetres behind.

Milestone 20 adds air jets at a fixed supply pressure, lets the built-in sorter fire them, and puts a jet next to a pusher in a demo.

The demo shows one thing above all: **a jet has to suit its product.** Frozen fish, bags of crisps and small boxes need different air, and no single setting rejects them all.

Try it: **File > New from a demo > Rejecting: pusher and air jet**.

---

## Part 1: The jet

### 1.1 The valve and its delays
A solenoid valve doesn't let air through the moment its output switches on:
- **opening (4 ms by default):** the coil's current builds up and pulls the armature in;
- **closing (6 ms):** the armature drops out, and the hose between the valve and the nozzles empties.

So the air flows from 4 ms after the output switches on until 6 ms after it switches off. A 20 ms pulse gives 22 ms of air. A 1 ms pulse still gives 3 ms of air, because closing takes longer than opening.

`AirJet` keeps the air's changes in a queue, in time order, each due a delay after its valve change. Every 1 ms step, `update` works out how much of the step the air flowed (0 to 1). A step in which the air starts halfway pushes for half the step, so timing is exact to a fraction of a millisecond, as for the photocells (lesson 2).

### 1.2 How hard a nozzle blows
A nozzle's **thrust** is roughly proportional to the gauge pressure and its bore's area: about 1.25 × p × A, which is what blow-off nozzle datasheets give. So:
- a 2 mm nozzle at 6 bar gives 2.4 N;
- a 5 mm nozzle gives 6.25 times that, 15 N.

The supply's pressure is fixed: the line's compressor and tank are assumed to keep up.

### 1.3 Streams
A free jet of air spreads in a cone, about 12 degrees from its axis to its edge. Each nozzle's cone is modelled as **37 thin streams**: one along the axis, then rings of 6, 12 and 18 around it. The streams spread evenly over the cone's cross-section, and each carries 1/37 of the thrust.

Each stream is a ray. It finds the first product it hits (`World::hitItem`): exactly, against a box or a tin, and through Jolt's mesh, against any other shape. Each stream then pushes, at the point it hits:
- **at right angles to the surface** (along the inward normal);
- **by the share of its thrust that's square to the surface** (the cosine of the angle between the stream and the normal).

The rest of the air slides off along the surface. That one rule gives three behaviours:

| Surface | What the jet does |
|---|---|
| flat, square to the jet | pushes it straight across, with the whole thrust |
| sloping or rounded, hit below its middle | pushes it across **and up**: it lifts |
| hit off-centre, away from its centre of mass | turns it as well (Jolt applies each force at its point) |

The jet's momentum isn't lost with distance, but it spreads out. A product further from the nozzle, or one smaller than the cone, catches fewer streams.

### 1.4 Forces for the next physics step
`BodyInterface::AddForce(body, force, point)` adds a force at a point. Jolt applies all such forces during the next `Update`, then clears them. So `updateAirJets` runs in each step **before** the physics, like the actuators and the belts.

---

## Part 2: Firing it

### 2.1 The sorter's pulse
The built-in sorter (lesson 5) fires a pusher at an encoder count and pulls it back when its *extended* sensor says it's out. A jet has no such sensor, so `SorterSpec::pulse` holds the valve open for a set time instead.

A PLC's timer is only looked at once a scan, so the pulse ends at the first scan after it's over: a 25 ms pulse is 30 ms with 10 ms scans. Real reject programs often use a timed output or a hardware pulse for that reason.

### 2.2 When to fire, and for how long
`sorterForJet` works out the timing from the layout. The air should meet the product's middle and blow while its middle third passes, for at most 80 ms. The sensor switches on the product's front edge, so:

    distance = (jet - sensor) + length / 2 - speed × (openTime + blow / 2)
    valve open for = blow - (closeTime - openTime)

### 2.3 Where the bin goes
A blown product keeps the belt's speed **along** the line while it flies and slides. At 0.4 m/s, a fish in the air or skating on the floor for half a second lands 20 cm downstream. Placed opposite the nozzles, the bin misses it. Real reject chutes are wide, long and downstream of the jet, and the demo's is too.

---

## Part 3: What the demo shows

Two lines, with the same products and a third of them bad. One rejects with a pusher, the other with a jet.

The jet's settings came from a scan: 36 combinations of bore, height, nozzles and blow, 40 s of line each. The outcome is what an integrator finds on a real line:

| Jet | Bad fish in the bin | Bad bags in the bin |
|---|---|---|
| 3 or 4 mm, 4 nozzles, 10 mm up | 0 of 3 | 6 of 7 |
| 5 mm, 6 nozzles, 10 mm up | 3 of 3 | 0 of 7: they fly over |
| any, 25 mm up | almost none | almost none |

- **A bag of crisps** (160 g, a big flat side) is thrown 1 to 3 m by a jet strong enough for a fish. A gentle jet drops it in the bin.
- **A frozen fish** (740 g, on a film of ice) needs a strong jet low down, under its rounded edge, where the air lifts it. A gentle jet nudges it a few centimetres, and the belt carries it on.
- **The box** (1.8 kg of cardboard) doesn't move for any of them. Its friction on the belt, μ × m × g ≈ 14 N, is more than the share of the thrust that reaches it.
- **Height matters more than blow length.** At 25 mm the jet hits the fish's edge near its middle, so it pushes without lifting, and the fish stays on the belt. Between 60 and 200 ms the results hardly change, because the product moves out of the jet anyway.

So the demo's jet is set for the fish (5 mm, 6 nozzles, 10 mm up). Watch it throw the bags over the bin, and leave the boxes alone.

The pusher takes everything, a box included. Its 30 cm paddle sometimes catches a good neighbour too, so its bin isn't all bad.

On a real line this is solved by:
- one product per line;
- jet settings per product (recipes);
- several jets selected by what the camera saw;
- a pusher for the heavy ones.

---

## Part 4: C++

### 4.1 A queue kept in order
The valve's changes don't arrive in the order the air changes: a close command with a 6 ms delay can be due after an open command given later with a 4 ms delay. Each change goes into the `std::deque` at its place, found with `std::upper_bound`, a binary search on the due times. `update` then only takes changes off the front.

### 4.2 Analytic rays
`rayBox` is the **slab method**. For each pair of parallel faces, it works out where the ray enters and leaves them. The ray is inside the box from the last entry to the first exit, and the face of the last entry is the one it hits.

`rayCylinder` solves x² + z² = r² for the side (a quadratic) and y = ±h for the lids, then keeps the nearest hit.

Both work in the item's own axes: the ray is turned by the transposed rotation (the inverse of a rotation matrix), and the normal is turned back.

### 4.3 A bug that wasn't
During the tuning, a test program crashed on some settings and not others, and not under gdb. The cause was the shell, not the code. In zsh, an unquoted `$args` stays **one** argument (bash would split it into words), so the program read null pointers for the arguments it didn't get. `${=args}` splits it.

Before blaming the code, check what it was given.

---

## Part 5: Testing

**`sim/air_jet_test.cpp`** (8 cases):
- the air starts 4 ms after the valve and stops 6 ms after it, and a pulse shorter than the opening delay still lets air through;
- the 37 streams are unit vectors inside the cone, and a 2 mm nozzle at 6 bar gives 2.36 N;
- a big flat face close to the nozzles takes all three nozzles' thrust, to 2 %;
- the same jet throws a bag off the belt and through the air, but can't move a 1.8 kg box;
- a jet under a fish's rounded edge lifts it, while a box's flat side isn't lifted at all;
- the sorter opens a jet's valve for its pulse, rounded up to whole scans;
- the demo: the pusher's bin gets at least 10 bad products, the jet's bin at least 3 fish, no boxes, and nothing good;
- air jets and a sorter's pulse survive a trip to JSON.

**By eye:** the jet's manifold, nozzles and valve LED beside the second line, its inspector, and the sorter list with both sorters.

---

## Review questions

**1.** A jet's valve opens in 4 ms and closes in 6 ms. The PLC sets the output for 10 ms. How long does the air flow, and when does it start?
<details><summary>Answer</summary>

From 4 ms after the output switches on, to 6 ms after it switches off (16 ms): 12 ms of air, starting 4 ms late. That 4 ms has to be allowed for when choosing when to fire: at 0.4 m/s it's 1.6 mm of travel.
</details>

**2.** Why does the jet lift a fish but not a box?
<details><summary>Answer</summary>

Each stream pushes at right angles to the surface it hits. A box's side is vertical, so the push is horizontal. Below its middle, a fish's edge is rounded and faces partly down, so the push there points partly up.
</details>

**3.** The same jet drops bags of crisps into the bin but leaves fish on the belt. What would you change for a line that carries both?
<details><summary>Answer</summary>

Something that tells them apart: two jets (or one jet with two valves and pressures) chosen by what the inspection camera saw, or a pusher for the heavy product. One air setting can't suit 160 g and 740 g products.
</details>

**4.** Why does the jet's bin sit downstream of the nozzles?
<details><summary>Answer</summary>

What it blows off keeps the belt's speed along the line while it flies, and a fish keeps sliding on the floor. It lands tens of centimetres further on, depending on the speed and how long it flies and slides.
</details>
