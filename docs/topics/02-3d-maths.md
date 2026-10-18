# 2. The 3D maths

Every box on the screen goes through the same chain: a unit cube, stretched to its size, turned, moved into place, seen from the camera, squeezed into perspective. This note explains each link, with the code that does it. factory_sim uses **glm**, a library that copies GLSL's types (`vec3`, `mat4`, `quat`) so the maths reads the same in C++ and in shaders.

Conventions, everywhere: metres, **Y up**, the floor at y = 0, a right-handed system (X right, Y up, Z towards you).

---

## 1. Vectors, and two products

A `glm::vec3` is three floats: a position (a point in the world) or a direction (an arrow, no position). Two operations do most of the work:

**The dot product** `dot(a, b) = ax·bx + ay·by + az·bz = |a| |b| cos(angle)`. With a unit vector `u`, `dot(v, u)` is **how far `v` goes along `u`**. The photocell uses exactly that: how far along the belt a box is, is `dot(boxCentre − beltStart, beltDirection)`. Lighting too: a face facing the sun gets `dot(normal, sun)` = 1, a face at right angles gets 0.

**The cross product** `cross(a, b)` is a vector at right angles to both, with length `|a| |b| sin(angle)`. The orbit camera uses it to find "right" from "forward" and "up" (`cross(forward, up)`), and "up" from "right" and "forward".

A box's **reach** along a direction (`reach()` in `world.cpp`) combines both ideas: a turned box's three axes, each times half the box's size, projected onto the direction with a dot product, absolute values added. That's the box's half-extent along any direction, which is how the photocell knows where a tilted box's front edge is.

---

## 2. Matrices: model, view, projection

A 4×4 matrix can turn, stretch and move a point, in one multiplication. Three of them take a corner of the unit cube to the screen:

```
corner of the unit cube ──model──► world ──view──► seen from the camera ──projection──► screen
(−0.5..0.5)                (metres)          (camera at 0, looking down −Z)           (−1..1, with perspective)
```

**Model** (`renderer.cpp`): place × turn × stretch, built right to left:

```cpp
model = translate(box.center) * rotation * scale(box.size);
```

The order matters: stretching first (in the cube's own axes), then turning, then moving. Stretch after turning and a long belt would be stretched along the world's X instead of its own length.

**View** (`OrbitCamera::view`): `glm::lookAt(position, target, up)` builds the matrix that moves and turns the whole world so the camera sits at the origin looking down −Z. Moving the camera right is the same as moving the world left.

**Projection** (`OrbitCamera::projection`): `glm::perspective(fieldOfView, aspect, near, far)` makes far things smaller. Everything between the near plane (2 cm) and the far plane (200 m) inside the 46° field of view ends up in a box from −1 to 1 on each axis.

The GPU gets `projection × view` once per frame and each box's `model`, and the vertex shader multiplies the corner by them.

### Homogeneous coordinates
Why 4×4 for 3D? A 3×3 matrix can turn and stretch, but not **move**: moving isn't a multiplication. Adding a fourth coordinate `w = 1` to every point makes translation a multiplication too. The perspective then uses `w`: the projection puts the point's distance into `w`, and after it the GPU divides x, y and z by `w` (the **perspective divide**). Far points have a big `w`, so they shrink towards the centre. That's the whole trick of perspective, and why `rayThrough` divides by `w` when it goes backwards.

### Normals and stretching
A normal (the direction a face points) must turn with its box, but not move. The shader uses only the rotation for normals (`mat3(uRotation)`). It doesn't use the full model matrix, because stretching a normal would tilt it. In general a stretched object needs the "inverse transpose" of its matrix for normals; here the cube's normals point along its own axes, which stretching doesn't tilt, so the rotation alone is exactly right, and cheaper.

---

## 3. Turning: angles and quaternions

A conveyor's direction is one angle, its **heading** around Y: `direction = (cos h, 0, −sin h)`. 0 runs along +X, 90° along −Z (turning left, seen from above). One angle is all a conveyor needs.

A box tumbling off a belt turns around any axis, and Jolt describes that with a **quaternion** (`glm::quat`): four numbers that encode "turn by this angle around this axis". Why not three angles (yaw, pitch, roll)? Three angles applied one after another can line two axes up and lose a degree of freedom (*gimbal lock*), and combining two turns is messy. Quaternions combine with one multiplication, interpolate smoothly, and never lock. factory_sim mostly uses them through glm:

- `glm::angleAxis(heading, up)`: a turn around Y, for conveyor parts;
- `glm::mat4_cast(q)`: the same turn as a matrix, for the model matrix;
- `glm::conjugate(q)`: the opposite turn, to bring a ray into a box's own axes (section 5).

---

## 4. The orbit camera: spherical coordinates

The camera sits on a sphere around its target. Its position is the target plus a direction worked out from two angles, times the distance (`OrbitCamera::position`):

```cpp
offset = (sin(yaw)·cos(pitch),  sin(pitch),  cos(yaw)·cos(pitch))
position = target + offset × distance
```

That's **spherical coordinates**: yaw turns around the vertical, pitch raises, distance is the radius. Mouse drags change yaw and pitch; the wheel scales the distance by 0.9 per step (so zooming feels the same near and far); panning moves the target along the screen's own right and up vectors, scaled so the point under the mouse follows it.

Pitch stops at about ±86°. At exactly 90° the camera looks straight down and `lookAt`'s "up" direction becomes parallel to the view, so "which way is up on screen" has no answer, and the image flips. Stopping short avoids it.

---

## 5. Rays: clicking on a 3D scene

A click is a pixel; what's under it is somewhere along a line from the camera through that pixel, a **ray**. `rayThrough` runs the pipeline backwards:

1. pixel → normalized device coordinates: x from −1 (left) to 1 (right), y from −1 (bottom) to 1 (top, so the window's y is flipped);
2. two points with that x and y: depth −1 (the near plane) and +1 (the far plane);
3. multiply both by `inverse(projection × view)`, then divide by `w` (undoing the perspective);
4. the ray starts at the near point and points at the far one.

Then each selectable box is tested with the **slab method** (`hitDistance`). In the box's own axes (the ray turned by the box's inverse rotation and moved by its centre), the box is the space between three pairs of parallel planes ("slabs"). For each axis, the ray enters its slab at one distance and leaves at another; the ray is inside the box where it's inside all three slabs at once, from the **latest entry** to the **earliest exit**. If the latest entry comes after the earliest exit, it missed. A ray parallel to a pair of planes is inside that slab everywhere or nowhere. `pickAt` keeps the nearest hit.

---

## Review questions

**1.** A box's centre is at (2, 0.9, 0.3); the belt starts at (0, 0.8, 0) and runs along +X. How far along the belt is the box?
<details><summary>Answer</summary>

2 m: `dot((2, 0.1, 0.3), (1, 0, 0)) = 2`. The dot product with the belt's unit direction keeps only the part along it.
</details>

**2.** The model matrix is `translate × rotation × scale`. What goes wrong with `translate × scale × rotation`?
<details><summary>Answer</summary>

The box would be turned first and stretched afterwards along the world's axes, not its own: a long belt turned 90° would be stretched sideways, coming out the wrong shape.
</details>

**3.** Why do points need a fourth coordinate w?
<details><summary>Answer</summary>

So moving a point (translation) can be a matrix multiplication like turning and stretching, which lets one matrix do all three; and so perspective can divide by w, making far things smaller.
</details>

**4.** Why does the camera's pitch stop short of 90°?
<details><summary>Answer</summary>

Looking straight down, the view direction is parallel to "up", so `lookAt` can't tell which way is up on the screen and the image flips. Stopping at about 86° keeps up defined.
</details>

**5.** In the slab test, the ray enters the three slabs at distances 2, 3 and 1, and leaves them at 5, 2.5 and 4. Does it hit the box, and where?
<details><summary>Answer</summary>

No. The latest entry is 3 and the earliest exit is 2.5: it leaves one slab before it has entered another, so it's never inside all three at once.
</details>
