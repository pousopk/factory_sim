#pragma once
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// The model behind the Open and Save As dialogs: one folder at a time, its
// sub-folders and files of one kind (scenes, or STL parts), and the file a
// typed name means. No ImGui
// here, so it's tested against a real folder; the dialog only draws it.
namespace fsim {

class FileBrowser {
public:
    // Lists the files ending in `extension` (in any case).
    explicit FileBrowser(std::filesystem::path folder, std::string extension = ".json");

    struct Entry {
        std::string name;
        bool folder = false;
        bool operator==(const Entry&) const = default;
    };

    const std::filesystem::path& folder() const { return folder_; }
    // Sub-folders first, then the files, each in alphabetical order.
    // Hidden ones (starting with '.') are left out.
    const std::vector<Entry>& entries() const { return entries_; }
    // Why the folder couldn't be listed (it was removed, no permission, ...).
    const std::string& error() const { return error_; }

    void go(std::filesystem::path folder);
    void enter(const std::string& name); // a sub-folder
    void up();                           // the parent folder
    void refresh();                      // read the folder again

    // The file a typed name means: in this folder, with the extension added
    // when it has none. Nothing for an empty name or one with a path
    // separator in it (pick folders by entering them).
    std::optional<std::filesystem::path> fileFor(std::string_view name) const;

private:
    std::filesystem::path folder_;
    std::string extension_;
    std::vector<Entry> entries_;
    std::string error_;
};

} // namespace fsim
