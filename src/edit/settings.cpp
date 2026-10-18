#include "edit/settings.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace fsim {

std::filesystem::path configFolder() {
    if (const char* config = std::getenv("XDG_CONFIG_HOME"); config != nullptr && *config != '\0') {
        return std::filesystem::path(config) / "factory_sim";
    }
    if (const char* home = std::getenv("HOME"); home != nullptr && *home != '\0') {
        return std::filesystem::path(home) / ".config" / "factory_sim";
    }
    return std::filesystem::current_path() / ".factory_sim";
}

void addRecent(Settings& settings, const std::filesystem::path& scene) {
    // Absolute and without "a/../b", so the same file is always the same entry.
    const auto path = std::filesystem::absolute(scene).lexically_normal();
    std::erase(settings.recent, path);
    settings.recent.insert(settings.recent.begin(), path);
    if (settings.recent.size() > Settings::kMaxRecent) {
        settings.recent.resize(Settings::kMaxRecent);
    }
}

Settings loadSettings(const std::filesystem::path& file) {
    Settings settings;
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        return settings;
    }
    std::stringstream text;
    text << in.rdbuf();
    // Parsing without exceptions: a damaged file is "no settings", not an error.
    const auto data = nlohmann::json::parse(text.str(), nullptr, false);
    if (!data.is_object() || !data.contains("recent") || !data["recent"].is_array()) {
        return settings;
    }
    for (const auto& entry : data["recent"]) {
        if (entry.is_string() && settings.recent.size() < Settings::kMaxRecent) {
            const std::filesystem::path path = entry.get<std::string>();
            std::error_code error;
            if (std::filesystem::exists(path, error)) {
                settings.recent.push_back(path);
            }
        }
    }
    return settings;
}

tl::expected<void, std::string> saveSettings(const Settings& settings, const std::filesystem::path& file) {
    std::error_code error;
    std::filesystem::create_directories(file.parent_path(), error);
    nlohmann::json data = {{"recent", nlohmann::json::array()}};
    for (const auto& path : settings.recent) {
        data["recent"].push_back(path.string());
    }
    std::ofstream out(file, std::ios::binary);
    out << data.dump(2) << "\n";
    if (!out) {
        return tl::unexpected("can't write " + file.string());
    }
    return {};
}

} // namespace fsim
