#pragma once
#include "sim/scene.hpp"
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <tl/expected.hpp>
#include <vector>

// What factory_sim remembers between runs, apart from the scenes themselves:
// the scenes opened recently. It lives in its own folder, the way Linux
// programs keep their settings (see configFolder), next to the window
// layout and the untitled scene.
namespace fsim {

// $XDG_CONFIG_HOME/factory_sim, or ~/.config/factory_sim without it (or the
// current folder, if there's no home either). Not created here.
std::filesystem::path configFolder();
// Where an untitled scene is saved until Save as names it: in the config folder.
std::filesystem::path untitledFile();

struct Settings {
    std::vector<std::filesystem::path> recent; // most recent first, absolute
    static constexpr std::size_t kMaxRecent = 8;

    bool operator==(const Settings&) const = default;
};

// Puts `scene` first in the recent list: once, absolute, and the oldest
// falls off past kMaxRecent.
void addRecent(Settings& settings, const std::filesystem::path& scene);

// Missing or unreadable settings are no reason not to start: they give the
// defaults. Recent scenes that no longer exist are left out.
Settings loadSettings(const std::filesystem::path& file);
// Creates the folder if needed.
tl::expected<void, std::string> saveSettings(const Settings& settings, const std::filesystem::path& file);

// The untitled scene left in `file` last time, if there's anything in it to
// go back to: the start page offers it. Not if it's missing, unreadable or
// an empty floor.
std::optional<Scene> untitledWork(const std::filesystem::path& file);

} // namespace fsim
