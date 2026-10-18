#pragma once
#include "render/gl.hpp"
#include <glm/glm.hpp>
#include <string_view>

namespace fsim {

// A compiled and linked GLSL program. It owns the OpenGL object, so it
// can be moved but not copied (two copies would delete it twice).
class Shader {
public:
    // Compiles both stages and links them. Throws std::runtime_error with
    // the driver's message if either fails: shaders are part of the
    // program, so a broken one is a bug, found the first time it runs.
    Shader(std::string_view vertexSource, std::string_view fragmentSource);
    ~Shader();
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    void use() const;
    void set(const char* name, float value) const;
    void set(const char* name, int value) const; // also a sampler's texture unit
    void set(const char* name, const glm::vec3& value) const;
    void set(const char* name, const glm::mat4& value) const;

private:
    GLuint program_ = 0;
};

} // namespace fsim
