# Lesson 19: Odd shapes

Until now every item was a box or a tin. Real lines carry bottles that tip over, bags that are mostly air, parts with their weight off-centre, and frozen fish that slide on a film of ice. Each of these is seen differently by a sensor and moves differently when pushed. Milestone 19 adds them:
- **built-in shapes:** a bottle, a bag of crisps, a ball, an angle bracket and a frozen fish;
- **STL import:** any part a CAD program can export;
- **materials:** what each item slides, bounces and weighs like;
- **real outlines:** every sensor (photocells, proximity switches, lasers, light curtains, the line camera) sees an item's actual shape, not its bounding box.

It prepares milestone 20, air jets, where a product's shape and weight decide where it lands.

Try it: **File > New from a demo > Odd shapes**, then **View > Camera view** for the line camera's image.

---

## Part 1: Two shapes for one item

### 1.1 What's seen and what collides
A shape like a fish needs two descriptions:
- **its surface**, a mesh of triangles: what's drawn, and what the sensors see;
- **a few convex pieces** that together follow that surface: what the physics collides.

A **convex** shape has no dents: a straight line between any two points inside it stays inside. Collision detection between two convex shapes is fast and reliable, because each has one "furthest point" in any direction (its *support point*), and the algorithms Jolt uses (GJK and EPA) only need that. A fish or an L-shaped bracket isn't convex, so it's made of several convex pieces stuck together (a Jolt *compound shape*).

Real simulators and games do the same: a detailed mesh to look at, a simpler set of convex pieces to collide.

### 1.2 The built-in shapes
Most are built by **lofting**: a series of cross-sections (rings of points) joined into a surface.

| Shape | Cross-sections | Convex pieces |
|---|---|---|
| bottle | circles up its height: body, shoulder, neck | body and shoulder; neck |
| bag of crisps | rounded rectangles along it, flat at the seals | one |
| ball | circles of latitude | a true sphere |
| fish | ellipses from snout to tail, then the flat tail fin | one between each pair of cross-sections |
| angle bracket | two plates | the two plates |

Why one piece per pair of fish cross-sections? The hull of two ellipses is convex and follows the body between them. One hull over the whole fish would fill in the narrowing before the tail.

The bottle is two pieces because of the **neck**. Body plus shoulder is convex: the radius only ever narrows, faster and faster. Then it stops narrowing at the neck, and that corner turns inward, which a single convex piece can't follow.

### 1.3 Mass and where it is
Each material has a density, and Jolt works out each piece's volume. So an item's mass, and its **centre of mass**, come from its shape:
- the fish is thickest behind the head, so its centre of mass is 6 cm ahead of its middle;
- the bracket's centre of mass is towards its upright plate.

When a jet or a pusher hits away from that point, the item turns as well as moves.

| Material | Friction | Bounce | Density | Metal |
|---|---|---|---|---|
| cardboard | 0.6 | 0 | 1000 kg/m³ | |
| tinplate | 0.4 | 0 | 1000 | yes |
| glass (filled bottle) | 0.35 | 0.05 | 1050 | |
| film (bag of crisps) | 0.5 | 0 | 60 | |
| rubber (ball) | 0.8 | 0.6 | 150 | |
| steel | 0.4 | 0.1 | 7850 | yes |
| frozen fish | 0.06 | 0.25 | 1000 | |

The density is of the **whole item**: a bag of crisps is mostly air (60 kg/m³), a ball is hollow. The frozen fish's friction of 0.06 is the ice glaze. Jolt combines two frictions as the square root of their product, so on the rubber belt (1.0) a fish gets 0.24, against a box's 0.77. That's why fish keep sliding after a push.

**Metal** decides what an inductive proximity switch sees: the bracket, yes; the fish, no.

---

## Part 2: STL files

### 2.1 The format
STL is the simplest 3D format, and nearly every CAD program exports it. It's only a list of triangles: no units, no colours, no materials. There are two kinds:
- **text:** `solid name`, then for each triangle `facet normal ...`, `outer loop`, three `vertex x y z` lines, `endloop`, `endfacet`;
- **binary:** an 80-byte header, a 4-byte triangle count, then 50 bytes per triangle (a normal and three corners as 32-bit floats, plus 2 spare bytes).

Some binary headers start with "solid" too, so the reader decides by size: a binary file is exactly `84 + 50 × count` bytes. The stored normals are ignored and worked out again from the corners. Normals are often wrong in exported files, and the order of the corners already says which way a triangle faces.

`examples/u_channel.stl` is a small text STL: a U-shaped profile, 120 mm long, with 6 mm walls and its top open. It's concave, so it shows why the next part is needed.

### 2.2 What the file doesn't say
Since STL has none of these, the spawner asks for them:
- **the unit:** mm, cm, inch or m;
- **which way is up:** most CAD programs have **z** up, factory_sim has **y** up;
- **the material.**

The part is centred on its bounding box, like every item.

### 2.3 Splitting into convex pieces: V-HACD
An STL part can be any shape, so the convex pieces are made by **V-HACD** (Volumetric Hierarchical Approximate Convex Decomposition), a widely used open-source library that is a single C++ header:
1. It fills the mesh with small cubes (100,000 voxels here).
2. It cuts that volume with a plane where it's least convex, and repeats on each half.
3. It stops at the maximum number of pieces, or when each piece is close enough to its convex hull.

The U-channel comes out as 8 pieces along its base and walls, holding 92.9 cm³ against the part's true 92.2 cm³. One convex hull would have filled the channel's inside: 288 cm³, about 3 times too much. Splitting takes under a second.

The **pieces** setting (default 8) trades accuracy for speed: 1 gives the convex hull, quick but filling every hollow.

V-HACD runs on one thread here (`m_asyncACD = false`), so the same file always gives the same pieces, and the simulation stays repeatable. Its result is **cached** by file, unit, material and settings, and only made again when the file changes. So rebuilding the line after an edit, which happens often, doesn't split the part again.

### 2.4 Finding the file
A relative name is looked for next to the scene, then in the examples folder. When a scene is saved somewhere else, the STL path is rewritten to point at the same file from the new folder, as was already done for the soft_plc program.

---

## Part 3: Sensors see the real outline

### 3.1 Three questions
Each sensor asks one of three questions, which each shape answers:

| Question | Who asks |
|---|---|
| **span:** how far does it reach in a direction? | quick checks: could it be near this sensor at all? |
| **slice:** at this height, which stretches along the belt does it cover? | photocells, zone eyes |
| **ray:** where does a line from here first hit its surface? | lasers, line cameras, proximity switches, light curtains |

- **Span** uses the corners of the surface's convex hull: a few dozen points instead of hundreds of triangles.
- **Rays** use Jolt's mesh shape (the surface as a Jolt `MeshShape`, never added to the physics world). It sorts the triangles into a tree of boxes, so a ray only tests the few it passes near. A line camera casts hundreds of rays per line, a thousand lines a second, so that matters.
- **Slice** is the photocell's question, so it gets its own subsection.

### 3.2 The slice
A photocell's beam crosses the whole belt at one height. It's blocked wherever the item's **cross-section at that height** reaches along the belt. Each triangle that crosses the plane at the beam's height leaves a short segment, and the segments together are the cross-section's outline. Projected onto the belt's direction and merged, they give exactly the stretches the beam is blocked by:
- a gap stays a gap: a beam across the U-channel's walls is blocked twice;
- a thin tail lifted off the belt isn't seen by a beam under it.

### 3.3 What changes on a real line
A rigid frozen fish lying on its side doesn't lie flat. It rests on its thickest part and the tip of its tail fin, nose slightly up. So:
- a photocell at 20 mm switches **about 11 cm after the fish's nose** passes, when the thick part arrives. The Triggers window shows it in the *error mm* column;
- at 3, 25 and 45 mm the same fish blocks the beam for 207, 248 and 129 mm of its 400 mm length. A box of the same size blocks all three for 400 mm.

This is the everyday problem of triggering on natural products: the trigger point depends on the product's shape and how it lies, not just on where it is. That's why such lines often use a light curtain or a camera instead of a single beam.

### 3.4 Boxes and tins don't change
Boxes and tins keep their exact old formulas. Every earlier test and measurement, such as the tin study's 1.30 mm, comes out exactly as before.

---

## Part 4: A physics bug, and how it was found

The first fish sank 15 mm into the belt, nose up. The search went step by step:
1. **Was the physics shape where the surface is?** Rays cast at both, from below, along the fish: they agreed to 0.1 mm. So the shape was right.
2. **Was the body in the wrong place?** Bottles, bags, balls and brackets, built the same way, rested exactly on the belt. So placement was right.
3. **Was it Jolt's allowed sinking** ("penetration slop", 2 cm by default)? Set to 2 mm, nothing changed.
4. **Was it having many pieces?** One convex hull rested perfectly, 2 pieces sank 1 mm, 8 pieces sank 15 mm.

That led to **manifold reduction**. To solve fewer contacts, Jolt merges the contact points of a body's pieces when their normals agree. On a curved underside made of 8 pieces, all touching the belt with nearly the same normal, the points it dropped were the ones holding the fish up. Turned off for items made of pieces (`mUseManifoldReduction = false`), the fish rests exactly on the belt. The bracket never showed it: its two flat plates touch with clearly separate faces.

The lesson is less about Jolt than about method: each step ruled out one layer (shape, placement, solver setting, piece count) with a small program that printed numbers, before anything was changed.

One more bug only showed in the demo's screenshot: a bottle standing **inside** a fish. A spawner checks for room before putting an item down, but it assumed items of similar height: it only looked for items whose middle was within its own height of the new one's. A tall bottle's middle is 10 cm above a fish's, more than the fish's 5 cm height. Now the test is the two heights together.

---

## Part 5: Drawing meshes

Boxes and tins are one unit cube or cylinder, stretched and drawn many times in one call (instancing, lesson 12). A mesh is drawn the same way: each shape's surface is uploaded to the GPU **once**, the first time it's drawn, and kept by the shape's id. All the fish then go in one draw call, all the bottles in another.

The batches are kept in a `std::map` keyed by (see-through, mesh). Its order puts all solid batches before the see-through ones, which must be drawn last, over everything else.

Shapes are shared: one geometry per shape and size. A spawner varying its items' heights rounds them to whole millimetres, so it makes a few dozen geometries, not one per item.

---

## Part 6: C++

### 6.1 Keeping Jolt out of the header
`ItemGeometry` holds Jolt shapes, but its header doesn't include Jolt. It declares `struct Jolt;`, holds a `std::unique_ptr<Jolt>`, and defines the struct in the `.cpp`. This is the **pimpl** ("pointer to implementation") idiom.
- **Why:** everything that includes `sensors.hpp`, which is most of the program, would otherwise compile Jolt's headers.
- **The catch:** the destructor must be defined in the `.cpp` (`ItemGeometry::~ItemGeometry() = default;` there), where `Jolt` is a complete type. `unique_ptr` needs to see the whole type to delete it.

### 6.2 Shared and immutable
Geometries are handed around as `std::shared_ptr<const ItemGeometry>`: by the item, its trigger record, a camera shot, the drawing. `const` means none of them can change it, so sharing is safe. The last one to let go frees it.

### 6.3 A trap with brace initialisation
`std::vector{a, b}` with two `std::vector<glm::vec3>` makes a vector of vectors. But `std::vector{a}` with one makes **a copy of `a`**: class template argument deduction prefers the copy constructor. The bag, made of one piece, failed to compile until the type was written out: `std::vector<std::vector<glm::vec3>>{...}`.

### 6.4 A library that isn't ours
V-HACD's code is compiled in one file only (`convex_pieces.cpp`, which defines `ENABLE_VHACD_IMPLEMENTATION`) with warnings off. It's someone else's code, and our strict warnings (`-Wconversion`, `-Wshadow`...) would only flag their style. CMake fetches the single header with `DOWNLOAD_NO_EXTRACT`.

---

## Part 7: Testing

- **`sim/item_shape_test.cpp`** (7 cases):
  - every built-in shape fits its size, faces outward (its enclosed volume is positive), has a mass, and its pieces hold about the same volume as its surface;
  - the same shape and size gives the same shared geometry;
  - the fish's centre of mass is ahead of its middle, and slices and rays find its thick body and thin fin;
  - a fallen bottle reaches its diameter up;
  - the bracket's centre of mass is towards its upright;
  - an STL file reads the same as text and binary, scaled from mm with z up;
  - the U-channel splits into pieces holding much less than its hull, and a beam across it is blocked by its two walls with the gap between.
- **`sim/shapes_test.cpp`** (4 cases):
  - every shape comes to rest on the belt and rides it at its speed;
  - photocells at three heights see a fish for different stretches, and a box of the same size for its whole length;
  - a laser measures a standing bottle's height;
  - the demo runs 30 s with every product made and seen, and a missing STL file stops only its spawner, with the reason.
- **`edit/files_test.cpp`**: the browser lists `.stl` files in any case; a scene saved elsewhere still finds its STL part.
- **By eye:** the Odd shapes demo in 3D, with the line camera's image showing fish outlines, tail fins included.

---

## Review questions

**1.** Why does the physics need convex pieces, while the sensors use the surface mesh?
<details><summary>Answer</summary>

Collisions between convex shapes are fast and robust: each has one furthest point in any direction, which is all the collision algorithms need. A concave shape (a fish, a U-channel) is several convex pieces. The sensors only cast rays and cut slices, which work on any triangle mesh, so they can use the exact surface that's drawn.
</details>

**2.** A photocell 20 mm above the belt triggers about 11 cm late on a frozen fish. Why, and what would make the trigger point steadier?
<details><summary>Answer</summary>

The fish rests on its thick body and its tail fin, nose up, so its snout passes above the beam. Only the thick part behind the head, at 20 mm, blocks it. A beam lower down sees more of the fish, but the trigger point still depends on each fish's shape and how it lies. A light curtain, or a camera that finds the snout, is steadier.
</details>

**3.** An STL file of a part drawn in millimetres is loaded as if in metres. What happens?
<details><summary>Answer</summary>

It's 1000 times too big: a 120 mm part becomes 120 m long. STL files have no units, so the spawner has to be told which one the file was drawn in.
</details>

**4.** Why is V-HACD run on one thread, and its result cached?
<details><summary>Answer</summary>

One thread: the same file always gives the same pieces, so runs stay repeatable. Cached: splitting takes a noticeable moment, and the line is rebuilt after every edit, so it's only done again when the file or its settings change.
</details>
