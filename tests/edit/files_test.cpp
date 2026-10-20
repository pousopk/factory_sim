#include "edit/file_browser.hpp"
#include "edit/scene_history.hpp"
#include "edit/settings.hpp"
#include <doctest/doctest.h>
#include <fstream>
#include <random>

using namespace fsim;

// Files: recent scenes, the folder browser behind Open and Save As, and
// scenes moving to another folder.

namespace {

// A fresh folder for one test, removed afterwards.
struct TemporaryFolder {
    std::filesystem::path path;
    TemporaryFolder() {
        std::random_device random;
        path = std::filesystem::temp_directory_path() / ("factory_sim_test_" + std::to_string(random()));
        std::filesystem::create_directories(path);
    }
    ~TemporaryFolder() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
    std::filesystem::path touch(const std::string& name) const {
        std::ofstream(path / name) << "{}";
        return path / name;
    }
};

} // namespace

TEST_CASE("Recent scenes: most recent first, each once, at most eight") {
    Settings settings;
    for (int i = 0; i < 10; ++i) {
        addRecent(settings, "/lines/scene" + std::to_string(i) + ".json");
    }
    REQUIRE(settings.recent.size() == Settings::kMaxRecent);
    CHECK(settings.recent.front() == "/lines/scene9.json");
    CHECK(settings.recent.back() == "/lines/scene2.json"); // 0 and 1 fell off
    addRecent(settings, "/lines/other/../scene5.json");    // the same file, written another way
    CHECK(settings.recent.front() == "/lines/scene5.json");
    CHECK(std::count(settings.recent.begin(), settings.recent.end(), "/lines/scene5.json") == 1);
    CHECK(settings.recent.size() == Settings::kMaxRecent);
}

TEST_CASE("Settings are saved and read back; missing, damaged or gone files don't stop anything") {
    TemporaryFolder folder;
    const auto file = folder.path / "config" / "settings.json"; // the folder is created
    Settings settings;
    addRecent(settings, folder.touch("a.json"));
    addRecent(settings, folder.touch("b.json"));
    REQUIRE(saveSettings(settings, file).has_value());
    CHECK(loadSettings(file) == settings);

    std::filesystem::remove(folder.path / "a.json"); // a scene deleted since
    CHECK(loadSettings(file).recent == std::vector<std::filesystem::path>{folder.path / "b.json"});
    CHECK(loadSettings(folder.path / "none.json").recent.empty());
    std::ofstream(file) << "{ not json";
    CHECK(loadSettings(file).recent.empty());
}

TEST_CASE("The file browser lists folders first, then scene files, without hidden ones") {
    TemporaryFolder folder;
    folder.touch("zeta.json");
    folder.touch("alpha.json");
    folder.touch("notes.txt");
    folder.touch(".hidden.json");
    std::filesystem::create_directory(folder.path / "lines");
    std::filesystem::create_directory(folder.path / ".git");
    FileBrowser browser(folder.path);
    CHECK(browser.error().empty());
    CHECK(browser.entries() ==
          std::vector<FileBrowser::Entry>{{"lines", true}, {"alpha.json", false}, {"zeta.json", false}});
    browser.enter("lines");
    CHECK(browser.folder() == folder.path / "lines");
    CHECK(browser.entries().empty());
    browser.up();
    CHECK(browser.folder() == folder.path);
    browser.go(folder.path / "gone");
    CHECK_FALSE(browser.error().empty()); // shown in the dialog, not thrown
}

TEST_CASE("The file browser can list STL parts instead, in any case") {
    TemporaryFolder folder;
    folder.touch("bracket.STL");
    folder.touch("fish.stl");
    folder.touch("line.json");
    FileBrowser browser(folder.path, ".stl");
    CHECK(browser.entries() == std::vector<FileBrowser::Entry>{{"bracket.STL", false}, {"fish.stl", false}});
    CHECK(browser.fileFor("hook") == folder.path / "hook.stl");
}

TEST_CASE("A typed name is a .json file in the browser's folder") {
    TemporaryFolder folder;
    FileBrowser browser(folder.path);
    CHECK(browser.fileFor("line") == folder.path / "line.json");
    CHECK(browser.fileFor("line.json") == folder.path / "line.json");
    CHECK_FALSE(browser.fileFor("").has_value());
    CHECK_FALSE(browser.fileFor("sub/line").has_value());
    CHECK_FALSE(browser.fileFor("..").has_value());
}

TEST_CASE("A scene saved in another folder still finds its STL parts") {
    TemporaryFolder lines;
    TemporaryFolder other;
    lines.touch("hook.stl");
    setShapeFolders({lines.path});
    Scene scene = defaultScene();
    SpawnerSpec spawner;
    spawner.shape = Shape::Stl;
    spawner.stl.file = "hook.stl";
    scene.spawners.push_back(spawner);
    spawner.stl.file = "missing.stl";
    scene.spawners.push_back(spawner);
    moveScene(scene, lines.path, other.path);
    CHECK(
        std::filesystem::absolute(other.path / scene.spawners[scene.spawners.size() - 2].stl.file).lexically_normal() ==
        std::filesystem::absolute(lines.path / "hook.stl").lexically_normal());
    CHECK(scene.spawners.back().stl.file == "missing.stl"); // not found: left as it was
    setShapeFolders({});
}

TEST_CASE("A scene saved in another folder still finds its soft_plc program") {
    Scene scene;
    scene.plcProgram = "programs/main.st"; // next to the scene, in /work/lines
    SUBCASE("relative from the new folder") {
        moveScene(scene, "/work/lines", "/work/other");
        CHECK(scene.plcProgram == "../lines/programs/main.st");
    }
    SUBCASE("deeper") {
        moveScene(scene, "/work/lines", "/work/lines/backup");
        CHECK(scene.plcProgram == "../programs/main.st");
    }
    SUBCASE("an absolute path stays as it is") {
        scene.plcProgram = "/opt/plc/main.st";
        moveScene(scene, "/work/lines", "/home/me");
        CHECK(scene.plcProgram == "/opt/plc/main.st");
    }
}

TEST_CASE("A change made everywhere in the history isn't undone by undo") {
    SceneHistory history(defaultScene());
    history.current().plcProgram = "main.st";
    history.commit();
    history.current().conveyors[0].spec.targetSpeed = 1.5F;
    history.commit();
    history.changeEverywhere([](Scene& s) { moveScene(s, "/a", "/b"); });
    CHECK_FALSE(history.commit()); // not an edit: nothing to save as a step
    CHECK(history.current().plcProgram == "../a/main.st");
    history.undo(); // the speed comes back; the program path stays rewritten
    CHECK(history.current().conveyors[0].spec.targetSpeed != 1.5F);
    CHECK(history.current().plcProgram == "../a/main.st");
}

TEST_CASE("The start page offers to continue the untitled scene only when there's something in it") {
    TemporaryFolder folder;
    const auto file = folder.path / "untitled.json";
    CHECK_FALSE(untitledWork(file).has_value()); // never made
    REQUIRE(saveScene(Scene{}, file).has_value());
    CHECK_FALSE(untitledWork(file).has_value()); // a new line, left empty
    REQUIRE(saveScene(defaultScene(), file).has_value());
    const auto work = untitledWork(file);
    REQUIRE(work.has_value());
    CHECK(*work == defaultScene());
    std::ofstream(file) << "{ not json";
    CHECK_FALSE(untitledWork(file).has_value()); // nothing to continue from a damaged file
}
