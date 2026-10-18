// factory_sim: a 3D factory simulator.
//
//   factory_sim                 the scene in factory_sim_scene.json (a new one if it doesn't exist)
//   factory_sim my_line.json    another scene file
//
// The scene is saved automatically after every change.
#include "app/app.hpp"
#include "render/gl.hpp" // before GLFW, which would otherwise include the old OpenGL header
#include <GLFW/glfw3.h>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

int main(int argc, char** argv) {
    if (argc > 2) {
        std::fprintf(stderr, "usage: factory_sim [scene.json]\n");
        return 2;
    }
    const std::filesystem::path scenePath = argc == 2 ? argv[1] : "factory_sim_scene.json";
    glfwSetErrorCallback(
        [](int error, const char* description) { std::fprintf(stderr, "GLFW error %d: %s\n", error, description); });
    if (!glfwInit()) {
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // smooth edges
    GLFWwindow* window = glfwCreateWindow(1400, 900, "factory_sim", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glEnable(GL_MULTISAMPLE);

    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    int status = 0;
    try {
        fsim::App app(scenePath); // after the OpenGL context: it creates shaders and buffers
        double last = glfwGetTime();
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            const double now = glfwGetTime();
            int width = 0;
            int height = 0;
            glfwGetFramebufferSize(window, &width, &height);
            app.frame(now - last, width, height);
            last = now;

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window);
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "factory_sim: %s\n", error.what());
        status = 1;
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return status;
}
