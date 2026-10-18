# 1. How the GPU draws a frame

Sixty times a second, factory_sim draws the floor, the conveyors, the boxes and the panels. The CPU doesn't draw any of it: it **describes** the frame to the GPU, a separate processor with its own memory, built to run the same small program on millions of pixels at once. OpenGL is the language for that description. This note explains what the CPU sends, what the GPU does with it, and how `src/render/` is organised around that.

---

## 1. The pipeline in one picture

```
CPU (renderer.cpp)                        GPU
──────────────────                        ───
vertices (once)  ──► buffer in GPU memory
per box: matrices,  ──► uniforms
colour, size
glDrawArrays ──────────► vertex shader      runs once per corner: where on screen is it?
                         │
                         ▼ rasterizer       which pixels does each triangle cover?
                         │                  (and interpolates the corners' outputs across it)
                         ▼ fragment shader  runs once per pixel: what colour is it?
                         │
                         ▼ depth test,      keep it only if it's nearer than what's there;
                           blending         mix it with what's there if it's see-through
                         │
                         ▼ framebuffer ───► the window
```

Everything the GPU draws is made of **triangles**. A box is 12 of them (2 per face); the floor is 2.

---

## 2. Data lives on the GPU

### Buffers
The GPU can't read the CPU's memory while it draws. So the cube's corners are copied to GPU memory **once**, at start-up (`Renderer::Renderer`):

```cpp
glGenBuffers(1, &cubeBuffer_);                       // ask for a buffer: you get a number
glBindBuffer(GL_ARRAY_BUFFER, cubeBuffer_);          // "the buffer I'm talking about now"
glBufferData(GL_ARRAY_BUFFER, sizeof cube, cube.data(), GL_STATIC_DRAW); // copy, once
```

`GL_STATIC_DRAW` is a hint: written once, drawn many times. The cube never changes: every box, belt and leg is that **same** cube, stretched and placed differently (topic 2). 36 corners, 6 floats each (position and normal): 864 bytes for the whole scene's geometry.

### Vertex arrays
A buffer is just bytes. A **vertex array object** (VAO) records how to read them: attribute 0 is three floats starting at byte 0, attribute 1 is three floats starting at byte 12, and a corner is 24 bytes long:

```cpp
glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);                       // position
glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(12));   // normal
```

Drawing then needs only `glBindVertexArray(cubeArray_)`: all of that comes back at once.

### Uniforms
What changes per box (its matrices, colour, size, whether it's see-through) is sent as **uniforms**: values the shaders read, the same for every corner and pixel of one draw call. `Shader::set("uColor", ...)` sends one.

---

## 3. One draw call per box

```cpp
for (const auto& box : boxes) {
    shader.set("uModel", model);     // where this box is
    shader.set("uColor", box.color);
    glDrawArrays(GL_TRIANGLES, 0, 36);
}
```

Each `glDrawArrays` is a **draw call**: "draw 36 corners as triangles, with the current state". A draw call has a fixed cost on the CPU side (the driver checks state, prepares commands). With a few hundred boxes that's nothing. With tens of thousands, it would be the bottleneck, and the fix would be **instancing**: put every box's matrix and colour in a buffer and draw all of them with one call (`glDrawArraysInstanced`). factory_sim doesn't need it yet; it's the first thing to change if scenes get big.

---

## 4. What decides which pixel wins

### The depth test
Two boxes can cover the same pixel. The GPU keeps a second image, the **depth buffer**, with each pixel's distance from the camera. With `glEnable(GL_DEPTH_TEST)`, a new pixel is kept only if it's nearer than the one already there. That's why the order of draw calls doesn't matter for solid objects: whatever is nearest wins, pixel by pixel. Each frame starts by clearing it (`glClear(... | GL_DEPTH_BUFFER_BIT)`).

### Back-face culling
With `glEnable(GL_CULL_FACE)`, triangles facing away from the camera are skipped before any pixel is computed: the inside faces of a closed box can never be seen anyway. The GPU decides which way a triangle faces from the **order** of its corners on screen: counter-clockwise is the front. That's why `cubeVertices()` builds each face's two triangles counter-clockwise as seen from outside, and reverses the order for the faces on the negative side of each axis (`const int k = sign > 0 ? c : 5 - c`). Get it wrong and the boxes look inside-out: you see their far faces through the near ones.

### Blending, and why see-through comes last
The photocell's beam and the trigger "ghost" are see-through. With blending on (`glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)`), a pixel's colour is mixed with what's already there: `new × alpha + old × (1 − alpha)`. That only works if "what's already there" is everything behind it, so:

1. draw every solid box first;
2. then the see-through ones, with depth **testing** on (a solid box in front still hides them) but depth **writing** off (`glDepthMask(GL_FALSE)`), so one see-through box doesn't hide another behind it.

Strictly, see-through boxes should also be drawn back to front among themselves. With two or three faint ones, the difference isn't visible; sorting is what a scene with many would need.

### Smoothing edges
The window asks for 4 samples per pixel (`glfwWindowHint(GLFW_SAMPLES, 4)` and `glEnable(GL_MULTISAMPLE)`): **multisample anti-aliasing**. Edges are sampled at four points inside each pixel, so a slanted belt edge gets in-between colours instead of a staircase.

---

## 5. OpenGL is a state machine

Most OpenGL calls don't take the object they act on: they act on whatever is **bound** right now. `glBufferData` fills the buffer bound to `GL_ARRAY_BUFFER`; `glDrawArrays` draws with the bound vertex array and the program set with `glUseProgram`. Enabling depth testing stays on until someone turns it off.

That's convenient and error-prone: a function that leaves blending on breaks the next thing drawn. `Renderer::draw` therefore turns everything it needs on at its start and **back off at its end** (`glDisable(GL_CULL_FACE)`, `glDisable(GL_DEPTH_TEST)`, `glBindVertexArray(0)`), because ImGui draws the panels right after it and expects plain 2D state.

---

## 6. Owning GPU objects in C++

`glCreateProgram` gives back a number; the GPU object lives until `glDeleteProgram`. That's a resource like heap memory, and it gets the same treatment: an owning class with a destructor (**RAII**).

`Shader` deletes its program in its destructor. Copying it would give two objects deleting the same number, so copying is deleted and **moving** transfers the number, leaving the moved-from shader holding 0 (which its destructor ignores):

```cpp
Shader::Shader(Shader&& other) noexcept : program_(std::exchange(other.program_, 0)) {}
```

`Renderer` owns its buffers and vertex arrays the same way, and is created **after** the OpenGL context and destroyed **before** it (`App` lives inside the `try` block in `main`, while the window still exists): GPU objects only make sense while their context does.

---

## 7. No loader library

On Windows, and in general, OpenGL functions newer than 1.1 must be looked up at run time, which is what loader libraries (glad, GLEW) do. On Linux, the system's `libGL` exports them directly, so declaring their prototypes is enough (`src/render/gl.hpp`):

```cpp
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
```

It's one less dependency, and it ties the build to Linux. A port to Windows would add a loader there.

---

## Review questions

**1.** The cube's 864 bytes are uploaded once. What is sent to the GPU for each box, every frame?
<details><summary>Answer</summary>

Uniforms: the box's model matrix, its rotation (for normals), size, colour, stripe settings, alpha and flags. Then a draw call of 36 corners that reuses the cube already in GPU memory.
</details>

**2.** Two solid boxes overlap on screen. Does it matter which is drawn first?
<details><summary>Answer</summary>

No: the depth test keeps, pixel by pixel, whichever is nearer to the camera. Order only matters for see-through boxes, which mix with what's already drawn.
</details>

**3.** A bug reverses the corner order of every triangle. What do you see?
<details><summary>Answer</summary>

Back-face culling then drops the faces that face the camera and keeps the far ones: boxes look inside-out, showing their insides.
</details>

**4.** Why is depth *writing* turned off while see-through boxes are drawn, but depth *testing* kept on?
<details><summary>Answer</summary>

Testing on: a solid box in front still hides them. Writing off: a see-through box doesn't stop another see-through box behind it from being drawn, which would cut holes in it.
</details>

**5.** Why may `Shader` be moved but not copied?
<details><summary>Answer</summary>

It owns a GPU program, deleted in its destructor. Two copies would both delete it. A move hands the program over and leaves the old object owning nothing (0), so exactly one object deletes it.
</details>
