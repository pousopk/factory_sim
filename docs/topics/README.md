# Topics: what factory_sim teaches beyond its milestones

The [milestone lessons](../lessons/README.md) follow the order things were built. These notes take one subject at a time, across the whole project, and explain it from the ground up, with factory_sim's code as the example. They're written for someone who knows C++ but has never drawn anything in 3D or plugged in a physics engine.

| # | Topic | You'll understand |
|---|---|---|
| 1 | [How the GPU draws a frame](01-gpu-and-opengl.md) | buffers, vertex arrays, draw calls, depth testing, culling, blending, OpenGL as a state machine, owning GPU objects in C++ |
| 2 | [The 3D maths](02-3d-maths.md) | vectors, dot and cross products, model/view/projection matrices, homogeneous coordinates, quaternions, the orbit camera, rays and picking |
| 3 | [Shaders](03-shaders.md) | what runs on the GPU, vertex and fragment shaders, interpolation, lighting, procedural detail (stripes, edges, a grid that stays sharp) |
| 4 | [Numbers and time](04-numbers-and-time.md) | float versus double, precision that runs out, why time is counted in steps, tolerances, determinism |
| 5 | [Plugging in a physics engine](05-physics-engine.md) | rigid bodies, static and dynamic, the broad phase, contact callbacks, what the engine does and what stays ours |
| 6 | [Design notes](06-design.md) | scene versus world, deriving instead of storing, keeping OpenGL thin, measuring before optimising |

Each one ends with review questions; the answers are folded underneath.
