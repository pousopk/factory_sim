#include "plc/programs.hpp"
#include "plc/s7_link.hpp"
#include "plc/soft_plc_program.hpp"

namespace fsim {

MadeProgram makeAnyProgram(const Scene& scene, const std::filesystem::path& sceneFolder) {
    switch (scene.control) {
    case Control::SoftPlc: {
        if (scene.plcProgram.empty()) {
            return {nullptr, "choose the ST file soft_plc should run"};
        }
        std::filesystem::path file = scene.plcProgram;
        if (file.is_relative()) {
            file = sceneFolder / file;
        }
        auto program = SoftPlcProgram::fromFile(file, static_cast<std::uint16_t>(scene.monitorPort));
        if (!program) {
            return {nullptr, program.error()};
        }
        return {std::move(*program), {}};
    }
    case Control::S7Plc:
        return {std::make_unique<S7Link>(scene.s7Host, static_cast<std::uint16_t>(scene.s7Port)), {}};
    case Control::Manual:
    case Control::BuiltInSorting:
    case Control::BuiltInCameras:
        break;
    }
    return {makeProgram(scene), {}};
}

} // namespace fsim
