# 3. Shaders

A shader is a small program that runs **on the GPU**, written in GLSL (C-like, with vectors and matrices built in). factory_sim has two pairs of them, both in `src/render/renderer.cpp` as raw string literals: one for boxes, one for the floor. They're compiled by the graphics driver when the app starts (`Shader`'s constructor), and a compile error throws with the driver's message, since a broken shader is a bug in the program.

---

## 1. Two stages, two kinds of program

**The vertex shader** runs once per corner. Its main job is to say where the corner lands on screen (`gl_Position`). It can also compute values for the next stage:

```glsl
void main() {
    vNormal = mat3(uRotation) * aNormal;  // the face's direction, turned with the box
    vLocal = aPosition * uSize;           // where on the box this corner is, in metres
    gl_Position = uViewProjection * uModel * vec4(aPosition, 1.0);
}
```

**The fragment shader** runs once per pixel each triangle covers, and decides its colour (`fragColor`).

Between them, the GPU **interpolates**: every `out` of the vertex shader arrives in the fragment shader blended between the triangle's three corners, according to where the pixel is. That's how a pixel in the middle of a face knows its own position on the box (`vLocal`), though only the corners computed it. The variables passed this way are called *varyings*.

The scale is worth seeing: the cube has 36 corners, but a box filling a quarter of the screen covers hundreds of thousands of pixels. The fragment shader is where the GPU's work really is, and why per-pixel effects are cheap only on the GPU.

---

## 2. Lighting: two cheap tricks

```glsl
float sun = max(dot(normal, kSun), 0.0);   // a directional light
float sky = 0.35 + 0.25 * normal.y;        // softer light from everywhere, more from above
fragColor = vec4(color * (sky + 0.75 * sun), uAlpha);
```

- **Lambert's law** (`dot(normal, sun)`): a surface facing the sun gets full light, one at an angle gets less, one facing away gets none. It's the oldest and simplest lighting model, and right for matte things like cardboard and rubber.
- **A hemisphere light** (`0.35 + 0.25 × normal.y`): in a real room, light bounces off everything, so faces in shadow aren't black. Tops get a bit more than sides, sides more than bottoms. It's a crude stand-in for all that bounced light, at the cost of one multiply-add.

No shadows: computing them needs a second pass that draws the scene from the sun's point of view (*shadow mapping*). For a tool where you need to read the scene rather than admire it, plain lighting with clear edges does the job.

---

## 3. Detail without geometry

The belt's stripes, the darker box edges and the floor grid aren't triangles: they're computed per pixel from the interpolated position. That's **procedural** detail: no extra geometry, no textures, and it's exact at any zoom.

### Stripes that move
```glsl
float s = fract((vLocal.x + 0.5 * uSize.x - uStripeOffset) / uStripeSpacing);
if (s < 0.06) color = mix(color, vec3(0.75), 0.6);
```
`vLocal.x + 0.5 × size` is the distance from the belt's start, in metres. Dividing by the spacing and keeping the fractional part (`fract`) gives a sawtooth from 0 to 1 every 25 cm; the first 6 % of each tooth is a stripe. Subtracting the belt's travel (`uStripeOffset`) moves the sawtooth, so the stripes move exactly as far as the belt surface. Only the top face gets them (`vLocalNormal.y > 0.5`).

### Edges
```glsl
vec3 toEdge = 0.5 * uSize - abs(vLocal);  // distance to each pair of faces
float edge = ... the smaller of the two that matter for this face ...;
color *= mix(0.55, 1.0, smoothstep(0.0, 0.006, edge));
```
Within 6 mm of a face's border, the colour darkens smoothly (`smoothstep` is a soft step from 0 to 1). Two boxes of the same colour touching each other stay readable as two boxes. Because the distance is in metres on the box, the edge stays 6 mm wide whatever the box's size.

### A grid that stays sharp at any distance
```glsl
float line(vec2 p, float spacing) {
    vec2 g = abs(fract(p / spacing - 0.5) - 0.5) / fwidth(p / spacing);
    return 1.0 - min(min(g.x, g.y), 1.0);
}
```
`fract(p/spacing − 0.5) − 0.5` is the distance to the nearest grid line, in grid units. The key is `fwidth`: how much `p/spacing` changes from one pixel to the next. Dividing by it turns the distance into **pixels**, so a line is one pixel wide near the camera and far away. A fixed width in metres would make far lines disappear and near ones fat. The grid also fades with distance, where 10 cm lines would just turn into noise.

---

## 4. Unlit and see-through

Two uniforms switch features per box:
- `uUnlit`: the photocell's beam and LED are **lights**, not surfaces. They return their colour as it is, skipping all the lighting, so they don't go dark when seen from the side.
- `uAlpha`: below 1, the box is see-through (the beam, the ghost of a triggered box). The fragment's alpha is used by blending (topic 1, section 4).
- `uHighlight`: the selection gets a blue tint after the lighting, so it stays visible on dark and light faces alike.

These are uniforms rather than separate shader programs: switching programs between draw calls costs more than a branch every fragment of a draw takes the same way.

---

## Review questions

**1.** The vertex shader computes `vLocal` only for the 36 corners. How does a pixel in the middle of a face get its own `vLocal`?
<details><summary>Answer</summary>

The GPU interpolates every vertex shader output across each triangle, according to where the pixel lies between the three corners.
</details>

**2.** Why does `fwidth` keep the floor's grid lines one pixel wide?
<details><summary>Answer</summary>

`fwidth` is how much the grid coordinate changes between neighbouring pixels. Dividing the distance to the nearest line by it converts the distance into pixels, so the line's width is measured on screen, not in metres, near or far.
</details>

**3.** What makes the belt's stripes move at the belt's speed?
<details><summary>Answer</summary>

The stripe pattern is shifted by `uStripeOffset`, which the scene sets from how far the belt surface has travelled (kept within one stripe spacing). When the belt moves 5 cm, the stripes shift 5 cm.
</details>

**4.** Why is the beam drawn unlit?
<details><summary>Answer</summary>

It's a light, not a surface: shading it like one would make it darker on its sides and invisible when the sun is behind it. Unlit, it shows its colour from any angle.
</details>
