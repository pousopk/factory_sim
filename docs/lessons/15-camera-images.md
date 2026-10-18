# Lesson 15: What the cameras see

Milestone 8 modelled cameras as timing: triggers, busy windows, which product was in the field of view. Milestone 15 shows **the images**:
- an area camera's picture, drawn from its own position and lens;
- a line-scan camera's image, built line by line.

Brightness (exposure as light level, gain) isn't modelled. What is modelled is what makes an image wrong for the job: **motion blur**, **noise**, and for line-scan cameras, **pixels that aren't square** because the line rate doesn't match the belt.

Try it: **File > New from a demo > Line-scan and area cameras**, then **View > Camera view**, and choose K1 (line scan) or K2 (area).

---

## Part 1: Area cameras

### 1.1 The camera's own view
`cameraView(spec, conveyor)` (in `render/scene`, tested without a window) builds the view and projection a camera would have:
- **Where:** the working distance from the middle of its field of view, turned by its tilt around the belt.
- **Which way up:** the picture's x along the travel, as cameras over conveyors are usually mounted.
- **The lens angle:** the field of view across (the field of view along × pixels across / pixels along) seen from the working distance: `2·atan(across / 2 / distance)`. A 0.3 m field of view at 1280 × 1024 pixels, from 0.5 m, is a lens of about 27° across.

The test checks the geometry against what an integrator would measure: the middle of the field of view lands in the middle of the picture, its ends at the left and right edges, and its sides at the top and bottom.

The renderer draws the scene with that view into an offscreen **framebuffer** instead of the window, and ImGui shows the result as an image. OpenGL's image rows go up from the bottom, so it's shown flipped.

### 1.2 The moment of the picture
**Live** is what the camera would see if it took a picture every frame. **Last picture** is the one it took when triggered. That picture is drawn a frame or so after the exposure, when the boxes have moved on: up to 8 mm at 0.5 m/s. So the items are put back where they were **at the exposure**, along their velocities, the same trick the photocells use for their trigger positions (lesson 2).

### 1.3 Motion blur
During the exposure, the product keeps moving. With a 2 ms exposure at 0.5 m/s it moves 1 mm, which is 4 pixels of 0.23 mm. A real sensor adds up all the light over that time, so edges smear.

The renderer does the same:
1. It draws the scene several times, with the items at moments spread over the exposure (two per pixel of movement, up to 12).
2. It adds the renders, each weighted by 1/n, into a **16-bit float** buffer, using additive blending.
3. A last pass adds the noise and writes the finished 8-bit image.

The float buffer matters. Twelve renders each worth 1/12 of a value, added up in 8 bits, would round away most of the gradient that makes the blur.

The inspector and the window show the blur in pixels, `speed × exposure / pixel size`. Vision people keep it under one pixel for measuring, and accept a few for presence checks. That's why fast lines use short exposures and strobes.

### 1.4 Noise
Each pixel gets a random amount added, with a normal distribution and the set standard deviation in grey levels. In the finishing shader, two pseudo-random numbers per pixel (a hash of its position and a seed) become a normal sample with the Box-Muller formula. A new seed per picture means new noise each time, as on a real sensor.

---

## Part 2: Line-scan cameras

### 2.1 One row, over and over
A line-scan camera has a single row of pixels across the belt. Each time it's triggered, it takes one line; the lines stack up, and the product moving past turns them into a 2D image. Line cameras are used for fast or wide products, and for anything round, because the image doesn't depend on the product's position.

In factory_sim a line is worked out in the simulation, not the renderer (`World::takeLine`). For each pixel there's a ray straight down onto the items, as they were **at that line's moment**, and the pixel takes the top of the highest item there, or the belt. Being simulation code, it's exact in time (a line at 1.6273 s sees the box where it was at 1.6273 s), deterministic, and tested.

### 2.2 What starts a line
- **Free-running:** the camera's own clock, say 1000 lines a second. Each line then covers `speed / rate` of belt: 0.5 mm at 0.5 m/s, 1 mm at 1 m/s.
- **From the encoder:** a line every N pulses. Each line covers N / pulses-per-metre of belt, at **any speed**.

The tests show the difference with the same 0.2 m box:
- free-running at 1000 lines/s, it's **400 lines** long at 0.5 m/s and **200** at 1 m/s;
- triggered every 13 encoder pulses, it's **100 lines** at both speeds.

A belt's speed is never quite constant (ramps, load, slip), so a free-running line camera on a conveyor gives products of varying length. That's why line cameras over conveyors are triggered from an encoder.

### 2.3 Square pixels
A pixel's size **across** is the line's width over its pixels: 0.45 m / 512 = 0.88 mm in the demo. Its size **along** is what the line trigger makes it: with 6519 pulses per metre, every 6 pulses = 0.92 mm. Close to square, so products keep their shape. When they don't match, the image is stretched or squashed, and the inspector says by how much and what would make it square: "for square pixels at 0.50 m/s: 0.57 kHz", or "5.7 pulses per line".

### 2.4 Blur on a line camera
A line's exposure has the same problem as an area camera's: the product moves during it, and the line averages what it saw. One test exposes each line for 4 ms at 1 m/s, which is 4 mm of travel for lines 1 mm apart, and finds the product's edges spread over several lines. With a 1 µs exposure, the edge falls in one line. The usual rule is an exposure no longer than the line period, preferably much shorter.

---

## Part 3: C++ and OpenGL

### 3.1 Offscreen rendering
A **framebuffer object** is a target to draw into other than the window: textures for colour (and a renderbuffer for depth) attached to it. `Picture` owns three:
- one render: colour and depth;
- the sum: 16-bit float colour;
- the finished image: 8-bit colour, the texture ImGui shows.

It owns OpenGL objects, so, like `Shader`, it can be moved but not copied: a copy would delete the same textures twice. Its move operations use `std::exchange` to take the other's handles and leave it empty.

### 3.2 A triangle with no vertex buffer
Adding renders together and finishing the image are each one draw over the whole target. The vertex shader makes a triangle big enough to cover it from `gl_VertexID` alone (0, 1, 2 → corners at (0,0), (2,0), (0,2)), so no vertex buffer is needed, only an empty vertex array, which OpenGL's core profile requires anyway.

### 3.3 The draw function, split
`Renderer::draw` used to draw the window. Now the scene drawing is `render(boxes, viewProjection, eye, framebuffer, width, height)`: the window passes framebuffer 0 and the orbit camera, a picture passes its own framebuffer and the camera's view. One drawing path, two destinations: the shadows, instancing and lighting are the same in the camera images as in the 3D view.

### 3.4 Uploading a line image
A line image is uploaded as an 8-bit RGB texture, all its lines in one buffer. RGB rows of an odd width aren't a multiple of 4 bytes long, so `GL_UNPACK_ALIGNMENT` is set to 1 for the upload. Without it, OpenGL would expect each row padded to 4 bytes and shear the image diagonally. Magnified, it's shown with `GL_NEAREST`, so each camera pixel stays a sharp square.

### 3.5 A bug only the Debug build found
The line camera made its noise generator, `std::normal_distribution<float>(0, noise)`, for every line, even with the noise set to 0. A normal distribution with a standard deviation of 0 is **undefined behaviour** in C++. The Release build ran it without complaint, and every test passed. The Debug build, with GCC's checked standard library, stopped on an assertion (`_M_stddev > 0`). Now the generator is only made when there's noise (a `std::optional`). Release is the everyday build since lesson 12, but this is why the Debug build, with its checks and sanitizers, still runs the tests before every commit: undefined behaviour that happens to work today is a bug waiting for another compiler.

---

## Part 4: Testing

- **`sim/linescan_test.cpp`** (4 cases):
  - free-running, a product's length in lines halves when the speed doubles, and its width in pixels doesn't change;
  - encoder-triggered, the same length at both speeds;
  - a long line exposure spreads the edges over several lines, a short one doesn't;
  - the noise's standard deviation, measured over the belt's pixels.
- **`render/scene_test.cpp`**: an area camera's view puts its field of view's middle, ends and sides on the picture's middle, edges, top and bottom, at any belt heading.
- **By eye:**
  - K1's image building up: a bad box as a red rectangle with the right proportions, a tin as a disc;
  - K2's live picture from above: the belt's stripe, a tin entering, the noise, and its blur in pixels in the header.

---

## Review questions

**1.** A free-running line camera takes 2000 lines a second, its pixels are 0.5 mm across, and the belt runs at 1.5 m/s. Are its pixels square? What line rate would make them so?
<details><summary>Answer</summary>

No: each line covers 1.5 / 2000 = 0.75 mm along, against 0.5 mm across, so products look 1.5 times shorter than they are. Square needs 1.5 / 0.0005 = 3000 lines a second.
</details>

**2.** Why do line cameras over conveyors usually take their lines from an encoder?
<details><summary>Answer</summary>

So each line covers the same length of belt whatever its speed: ramps, load changes and slip don't stretch or squash the products in the image.
</details>

**3.** An area camera exposes for 1 ms over a belt at 2 m/s, with 0.25 mm pixels. How many pixels of blur? What would bring it under one pixel?
<details><summary>Answer</summary>

2 m/s × 1 ms = 2 mm = 8 pixels. Under one pixel needs an exposure under 0.125 ms, with a strobe to give enough light in that time.
</details>

**4.** Why are the blurred renders added up in a 16-bit float buffer instead of 8 bits?
<details><summary>Answer</summary>

Each render is added with a weight of 1/n. In 8 bits, those small shares round to whole steps, so the sum loses much of the gradient that makes the blur, or gets the colour wrong. Floats keep the fractions until the finished image is written.
</details>
