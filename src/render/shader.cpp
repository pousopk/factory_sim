#include "render/shader.hpp"
#include <glm/gtc/type_ptr.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace fsim {

namespace {

GLuint compile(GLenum stage, std::string_view source) {
    const GLuint shader = glCreateShader(stage);
    const char* text = source.data();
    const auto length = static_cast<GLint>(source.size());
    glShaderSource(shader, 1, &text, &length);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == GL_FALSE) {
        char log[2048];
        glGetShaderInfoLog(shader, sizeof log, nullptr, log);
        glDeleteShader(shader);
        throw std::runtime_error(std::string(stage == GL_VERTEX_SHADER ? "vertex" : "fragment") +
                                 " shader doesn't compile: " + log);
    }
    return shader;
}

} // namespace

Shader::Shader(std::string_view vertexSource, std::string_view fragmentSource) {
    const GLuint vertex = compile(GL_VERTEX_SHADER, vertexSource);
    const GLuint fragment = compile(GL_FRAGMENT_SHADER, fragmentSource);
    program_ = glCreateProgram();
    glAttachShader(program_, vertex);
    glAttachShader(program_, fragment);
    glLinkProgram(program_);
    glDeleteShader(vertex); // the program keeps what it needs
    glDeleteShader(fragment);
    GLint ok = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (ok == GL_FALSE) {
        char log[2048];
        glGetProgramInfoLog(program_, sizeof log, nullptr, log);
        glDeleteProgram(program_);
        throw std::runtime_error(std::string("shader program doesn't link: ") + log);
    }
}

Shader::~Shader() {
    if (program_ != 0) {
        glDeleteProgram(program_);
    }
}

Shader::Shader(Shader&& other) noexcept : program_(std::exchange(other.program_, 0)) {}

Shader& Shader::operator=(Shader&& other) noexcept {
    if (this != &other) {
        if (program_ != 0) {
            glDeleteProgram(program_);
        }
        program_ = std::exchange(other.program_, 0);
    }
    return *this;
}

void Shader::use() const {
    glUseProgram(program_);
}

void Shader::set(const char* name, float value) const {
    glUniform1f(glGetUniformLocation(program_, name), value);
}

void Shader::set(const char* name, const glm::vec3& value) const {
    glUniform3fv(glGetUniformLocation(program_, name), 1, glm::value_ptr(value));
}

void Shader::set(const char* name, const glm::mat4& value) const {
    glUniformMatrix4fv(glGetUniformLocation(program_, name), 1, GL_FALSE, glm::value_ptr(value));
}

} // namespace fsim
