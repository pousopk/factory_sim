// factory_sim: a 3D factory simulator.
//
//   factory_sim                 the start page: a new line, a scene to open, or a demo
//   factory_sim my_line.json    that scene file (a new, empty scene there if it doesn't exist)
//
// The scene is saved automatically after every change. factory_sim's own
// files (recent scenes, the window layout, the untitled scene) are in
// ~/.config/factory_sim.
#include "app/app.hpp"
#include "app/theme.hpp"
#include "render/gl.hpp" // before GLFW, which would otherwise include the old OpenGL header
#include <GLFW/glfw3.h>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include <optional>
#include <string>

int main(int argc, char** argv) {
    if (argc > 2) {
        std::fprintf(stderr, "usage: factory_sim [scene.json]\n");
        return 2;
    }
    std::optional<std::filesystem::path> scenePath;
    if (argc == 2) {
        scenePath = argv[1];
    }
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
    ImPlot::CreateContext(); // the logic analyser's plots
    // The window layout is remembered, in the config folder (ImGui keeps the
    // pointer: the string has to outlive the context).
    std::error_code ignored;
    std::filesystem::create_directories(fsim::configFolder(), ignored);
    const std::string layoutFile = (fsim::configFolder() / "layout.ini").string();
    ImGui::GetIO().IniFilename = layoutFile.c_str();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable; // panels dock around the 3D view
    // The font and style, at the screen's scale (2 on a HiDPI screen at 200 %).
    float scaleX = 1.0F;
    float scaleY = 1.0F;
    glfwGetWindowContentScale(window, &scaleX, &scaleY);
    fsim::applyTheme(scaleX);
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    int status = 0;
    try {
        fsim::App app(scenePath); // after the OpenGL context: it creates shaders and buffers
        double last = glfwGetTime();
        std::string title;
        while (!glfwWindowShouldClose(window) && !app.wantsToQuit()) {
            if (app.title() != title) {
                title = app.title();
                glfwSetWindowTitle(window, title.c_str());
            }
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
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return status;
}
