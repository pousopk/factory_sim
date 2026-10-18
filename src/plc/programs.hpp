#pragma once
#include "sim/scene.hpp"
#include <filesystem>
#include <memory>
#include <string>

// Makes the control program a scene asks for, whatever kind it is: the
// built-in sorting, soft_plc in lockstep, or a live S7 PLC. (The core
// library's makeProgram only knows the built-in one; this one links soft_plc
// and Snap7.)
namespace fsim {

struct MadeProgram {
    std::unique_ptr<ControlProgram> program; // null: manual control, or it failed
    std::string error;                       // why it failed (a compile error, a busy port, ...)
};

// `sceneFolder` is where the scene file is: a relative soft_plc program path
// is relative to it.
MadeProgram makeAnyProgram(const Scene& scene, const std::filesystem::path& sceneFolder);

} // namespace fsim
