cmake_minimum_required(VERSION 3.21)
project(factory_sim CXX)

# Some fetched dependencies declare a cmake_minimum_required below 3.5, which
# CMake 4.x refuses. This is CMake's documented workaround.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

option(FACTORY_SIM_SANITIZERS "Build Debug builds with AddressSanitizer and UBSan" ON)
option(FACTORY_SIM_BUILD_APP "Build the 3D application (needs GLFW and OpenGL)" ON)

include(FetchContent)

function(factory_sim_mark_system target)
  get_target_property(dirs ${target} INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(${target} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${dirs}")
endfunction()

# --- {fmt} ---
FetchContent_Declare(fmt GIT_REPOSITORY https://github.com/fmtlib/fmt.git GIT_TAG 11.1.4)
FetchContent_MakeAvailable(fmt)
factory_sim_mark_system(fmt)

# --- glm: vectors, matrices and quaternions for 3D ---
FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.1)
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glm)
factory_sim_mark_system(glm-header-only) # glm is header-only; this target carries its include path

# --- Jolt Physics: rigid bodies and collisions ---
FetchContent_Declare(
  JoltPhysics
  GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
  GIT_TAG        v5.2.0
  SOURCE_SUBDIR  Build
)
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
set(CPP_RTTI_ENABLED ON CACHE BOOL "" FORCE)        # our code uses RTTI (e.g. std::function)
set(CPP_EXCEPTIONS_ENABLED ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(JoltPhysics)
factory_sim_mark_system(Jolt)
# Jolt is optimised even in Debug builds: we debug our code, not Jolt's, and
# unoptimised physics is too slow to keep up with real time.
target_compile_options(Jolt PRIVATE $<$<CONFIG:Debug>:-O2>)

# --- tl::expected: value-or-error results (std::expected is C++23) ---
FetchContent_Declare(tl-expected GIT_REPOSITORY https://github.com/TartanLlama/expected.git GIT_TAG v1.1.0)
set(EXPECTED_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(EXPECTED_BUILD_PACKAGE OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(tl-expected)
factory_sim_mark_system(expected)

# --- nlohmann/json: the scene file ---
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(json)
factory_sim_mark_system(nlohmann_json)

find_package(Threads REQUIRED)

# --- Shared compile settings for our own targets ---
add_library(factory_sim_options INTERFACE)
target_compile_options(factory_sim_options INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
)
if(FACTORY_SIM_SANITIZERS)
  target_compile_options(factory_sim_options INTERFACE
    $<$<CONFIG:Debug>:-fsanitize=address,undefined -fno-omit-frame-pointer>
  )
  target_link_options(factory_sim_options INTERFACE $<$<CONFIG:Debug>:-fsanitize=address,undefined>)
endif()

# --- The simulation: no graphics, so tests can run it headless ---
add_library(factory_sim_core STATIC
  src/sim/physics.cpp
  src/sim/world.cpp
  src/sim/signal.cpp
  src/sim/sensors.cpp
  src/sim/scene.cpp
  src/sim/actuators.cpp
  src/sim/io.cpp
  src/sim/control.cpp
)
target_include_directories(factory_sim_core PUBLIC src)
target_link_libraries(factory_sim_core
  PUBLIC  factory_sim_options glm::glm-header-only Jolt Threads::Threads tl::expected
  PRIVATE fmt::fmt nlohmann_json::nlohmann_json
)

# --- What to draw and where from: plain geometry, no OpenGL, so it's tested too ---
add_library(factory_sim_view STATIC
  src/render/scene.cpp
  src/render/orbit_camera.cpp
  src/edit/scene_history.cpp
)
target_include_directories(factory_sim_view PUBLIC src)
target_link_libraries(factory_sim_view PUBLIC factory_sim_core)

# --- The 3D application ---
if(FACTORY_SIM_BUILD_APP)
  find_package(glfw3 REQUIRED)
  find_package(OpenGL REQUIRED)

  FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git GIT_TAG v1.92.6)
  FetchContent_Declare(implot GIT_REPOSITORY https://github.com/epezent/implot.git GIT_TAG v1.0)
  FetchContent_MakeAvailable(imgui implot)

  add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
  )
  target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends ${imgui_SOURCE_DIR}/misc/cpp)
  target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
  add_library(implot STATIC ${implot_SOURCE_DIR}/implot.cpp ${implot_SOURCE_DIR}/implot_items.cpp)
  target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})
  target_link_libraries(implot PUBLIC imgui)
  factory_sim_mark_system(imgui)
  factory_sim_mark_system(implot)

  add_executable(factory_sim
    src/app/main.cpp
    src/app/app.cpp
    src/render/shader.cpp
    src/render/renderer.cpp
  )
  target_link_libraries(factory_sim PRIVATE factory_sim_view implot imgui fmt::fmt OpenGL::GL)
endif()

enable_testing()
add_subdirectory(tests)
