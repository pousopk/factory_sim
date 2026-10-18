#include "edit/file_browser.hpp"
#include <algorithm>

namespace fsim {

FileBrowser::FileBrowser(std::filesystem::path folder) {
    go(std::move(folder));
}

void FileBrowser::go(std::filesystem::path folder) {
    folder_ = std::filesystem::absolute(folder).lexically_normal();
    // "a/b/" normalises to "a/b/" with an empty last part: drop it, so
    // parent_path() goes up a level.
    if (folder_.has_relative_path() && folder_.filename().empty()) {
        folder_ = folder_.parent_path();
    }
    refresh();
}

void FileBrowser::enter(const std::string& name) {
    go(folder_ / name);
}

void FileBrowser::up() {
    go(folder_.parent_path());
}

void FileBrowser::refresh() {
    entries_.clear();
    error_.clear();
    // The error_code forms: a folder we can't read is something to show,
    // not an exception to unwind through the frame.
    std::error_code error;
    std::filesystem::directory_iterator it(folder_, error);
    for (; !error && it != std::filesystem::directory_iterator(); it.increment(error)) {
        const auto name = it->path().filename().string();
        if (name.empty() || name.front() == '.') {
            continue;
        }
        std::error_code ignored;
        if (it->is_directory(ignored)) {
            entries_.push_back(Entry{name, true});
        } else if (it->path().extension() == ".json") {
            entries_.push_back(Entry{name, false});
        }
    }
    if (error) {
        error_ = "can't list " + folder_.string() + ": " + error.message();
    }
    std::sort(entries_.begin(), entries_.end(), [](const Entry& a, const Entry& b) {
        return a.folder != b.folder ? a.folder : a.name < b.name; // folders first
    });
}

std::optional<std::filesystem::path> FileBrowser::fileFor(std::string_view name) const {
    if (name.empty() || name.find('/') != std::string_view::npos || name == "." || name == "..") {
        return std::nullopt;
    }
    std::filesystem::path file = folder_ / std::filesystem::path(name);
    if (!file.has_extension()) {
        file += ".json";
    }
    return file;
}

} // namespace fsim
