#pragma once
// OpenGL 3.3 core. On Linux, the system's libGL exports the modern functions
// directly, so declaring their prototypes is enough: no loader library.
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
