// Files: the menu bar, the Open and Save As dialog, recent scenes, and the
// untitled scene.
//
// Every change is still saved at once, as since milestone 3; what's new is
// where. A named scene is saved to its file. An untitled one (a new scene,
// or a demo) is saved to a scratch file in the config folder, so nothing is
// lost before it has a name, and Save As gives it one.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <fmt/format.h>
#include <imgui.h>
#include <imgui_stdlib.h>

namespace fsim {

namespace {

std::filesystem::path settingsFile() {
    return configFolder() / "settings.json";
}

} // namespace

std::string App::title() const {
    if (startPage_) {
        return "factory_sim";
    }
    return (scenePath_.empty() ? std::string("untitled") : scenePath_.filename().string()) + " - factory_sim";
}

std::filesystem::path App::savePath() const {
    return scenePath_.empty() ? untitledFile() : scenePath_;
}

std::filesystem::path App::sceneFolder() const {
    return scenePath_.empty() ? std::filesystem::current_path() : scenePath_.parent_path();
}

void App::remember(const std::filesystem::path& path) {
    addRecent(settings_, path);
    if (auto saved = saveSettings(settings_, settingsFile()); !saved) {
        fileStatus_ = saved.error();
    }
}

void App::replaceScene(Scene scene, std::filesystem::path path) {
    history_ = SceneHistory(std::move(scene)); // a new file starts a new undo history
    scenePath_ = path.empty() ? path : std::filesystem::absolute(path).lexically_normal();
    startPage_ = false;
    selected_.reset();
    firing_.clear();
    study_.clear();
    rebuildWorld();
}

bool App::openScene(const std::filesystem::path& path) {
    auto loaded = loadScene(path);
    if (!loaded) {
        fileStatus_ = loaded.error();
        return false;
    }
    replaceScene(std::move(*loaded), path);
    remember(path);
    fileStatus_ = "opened " + path.filename().string();
    return true;
}

void App::saveSceneAs(const std::filesystem::path& path) {
    // Relative paths in the scene start from its folder: keep them pointing
    // at the same files from the new one, in every undo step too.
    const auto from = sceneFolder();
    const auto to = path.parent_path();
    history_.changeEverywhere([&](Scene& s) { moveScene(s, from, to); });
    moveScene(applied_, from, to); // the same program: no restart
    if (auto saved = saveScene(scene(), path); !saved) {
        fileStatus_ = saved.error();
        return;
    }
    scenePath_ = std::filesystem::absolute(path).lexically_normal();
    remember(path);
    fileStatus_ = "saved as " + path.filename().string();
}

void App::startNew(Scene scene) {
    // The untitled scene exists only in the scratch file: replacing one that
    // was edited would lose it, so that asks first. From the start page, the
    // one left in the scratch file counts.
    const bool atRisk =
        startPage_ ? untitledLeft_ : scenePath_.empty() && history_.canUndo();
    if (atRisk) {
        pendingNew_ = std::move(scene);
        return;
    }
    beginNew(std::move(scene));
}

void App::beginNew(Scene scene) {
    replaceScene(std::move(scene), {});
    if (auto saved = saveScene(this->scene(), untitledFile()); !saved) {
        fileStatus_ = saved.error();
    } else {
        fileStatus_ = "a new, untitled scene";
    }
}

void App::continueUntitled() {
    if (auto untitled = untitledWork(untitledFile())) {
        replaceScene(std::move(*untitled), {});
        fileStatus_ = "untitled";
    }
}

void App::closeScene() {
    // Every change is already saved (an untitled scene in the scratch file,
    // which the start page offers to continue): nothing to ask.
    replaceScene(Scene{}, {});
    startPage_ = true;
    untitledLeft_ = untitledWork(untitledFile()).has_value();
    fileStatus_.clear();
}

void App::drawMenuBar() {
    const auto openDialog = [&](Dialog dialog) {
        dialog_ = dialog;
        browser_.emplace(scenePath_.empty() ? std::filesystem::current_path() : scenePath_.parent_path());
        dialogName_ = dialog == Dialog::SaveAs && !scenePath_.empty() ? scenePath_.filename().string() : "";
        dialogError_.clear();
    };
    // The shortcuts work whatever has the focus, except while typing.
    const auto& io = ImGui::GetIO();
    if (!io.WantTextInput && io.KeyCtrl) {
        if (ImGui::IsKeyPressed(ImGuiKey_N)) {
            startNew(Scene{});
        } else if (ImGui::IsKeyPressed(ImGuiKey_O)) {
            openDialog(Dialog::Open);
        } else if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_S) && !startPage_) {
            openDialog(Dialog::SaveAs);
        }
    }
    if (!ImGui::BeginMainMenuBar()) {
        return;
    }
    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("New", "Ctrl+N")) {
            startNew(Scene{});
        }
        ImGui::SetItemTooltip("An empty floor, untitled");
        if (ImGui::BeginMenu("New from a demo")) {
            for (const auto& demo : demos()) {
                if (ImGui::MenuItem(demo.name)) {
                    startNew(demo.make());
                }
                if (ImGui::BeginItemTooltip()) {
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0F);
                    ImGui::TextUnformatted(demo.description);
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
            }
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Open...", "Ctrl+O")) {
            openDialog(Dialog::Open);
        }
        if (ImGui::BeginMenu("Open recent", !settings_.recent.empty())) {
            std::optional<std::filesystem::path> chosen;
            for (const auto& path : settings_.recent) {
                if (ImGui::MenuItem(path.filename().string().c_str(), path.parent_path().string().c_str())) {
                    chosen = path; // not while iterating: opening changes the list
                }
            }
            ImGui::EndMenu();
            if (chosen) {
                openScene(*chosen);
            }
        }
        if (ImGui::MenuItem("Save as...", "Ctrl+Shift+S", false, !startPage_)) {
            openDialog(Dialog::SaveAs);
        }
        if (ImGui::MenuItem("Close scene", nullptr, false, !startPage_)) {
            closeScene();
        }
        ImGui::SetItemTooltip("Back to the start page (the scene is already saved)");
        ImGui::TextColored(kDim, "(every change is saved at once)");
        ImGui::Separator();
        if (ImGui::MenuItem("Quit")) {
            quit_ = true;
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edit", !startPage_)) {
        if (ImGui::MenuItem("Undo", "Ctrl+Z", false, history_.canUndo())) {
            history_.undo();
        }
        if (ImGui::MenuItem("Redo", "Ctrl+Y", false, history_.canRedo())) {
            history_.redo();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Copy", "Ctrl+C", false, selected_.has_value())) {
            copySelection();
        }
        if (ImGui::MenuItem("Paste", "Ctrl+V", false, clipboard_.has_value())) {
            pasteClip(std::nullopt);
        }
        if (ImGui::MenuItem("Duplicate", "Ctrl+D", false, selected_.has_value())) {
            copySelection();
            pasteClip(std::nullopt);
        }
        if (ImGui::MenuItem("Turn conveyor", "R / Shift+R", false,
                            selected_ && selected_->kind == Pick::Kind::Conveyor && canDrag(*selected_))) {
            turnSelection(1.0F);
        }
        ImGui::Separator();
        ImGui::TextDisabled("Drag the selection in the view: a device along its belt,");
        ImGui::TextDisabled("a conveyor across the floor. Shift: fine steps.");
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View", !startPage_)) {
        ImGui::MenuItem("Control", nullptr, &showControl_);
        ImGui::MenuItem("Line", nullptr, &showLine_);
        ImGui::MenuItem("Inspector", nullptr, &showInspector_);
        ImGui::MenuItem("Triggers", nullptr, &showTriggers_);
        ImGui::MenuItem("I/O", nullptr, &showIo_);
        ImGui::MenuItem("Operator panel", nullptr, &showControlPanel_);
        ImGui::MenuItem("PLC variables", nullptr, &showPlcVariables_);
        ImGui::MenuItem("Cameras", nullptr, &showCameras_);
        ImGui::MenuItem("Camera view", nullptr, &showCameraView_);
        ImGui::MenuItem("Logic analyser", nullptr, &showAnalyser_);
        ImGui::Separator();
        ImGui::MenuItem("Names in 3D", nullptr, &showNames_);
        ImGui::MenuItem("Shadows", nullptr, &renderer_.shadows);
        ImGui::Separator();
        if (ImGui::MenuItem("Reset layout")) {
            resetLayout_ = true;
        }
        ImGui::SetItemTooltip("The panels back where they start: building on the left, watching on the right,\n"
                              "time underneath");
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        ImGui::MenuItem("Mouse and keys", "F1", &showHelp_);
        ImGui::EndMenu();
    }
    // Which file this is, on the right.
    if (startPage_) {
        ImGui::EndMainMenuBar();
        return;
    }
    const auto name = scenePath_.empty() ? std::string("untitled (saved in the config folder)") : scenePath_.string();
    ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(name.c_str()).x - 16.0F);
    ImGui::TextColored(kDim, "%s", name.c_str());
    ImGui::EndMainMenuBar();
}

void App::drawNewConfirmation() {
    if (pendingNew_) {
        ImGui::OpenPopup("Replace the untitled scene?");
    }
    if (ImGui::BeginPopupModal("Replace the untitled scene?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("The untitled scene hasn't been saved under a name: it would be lost.");
        if (ImGui::Button("Save it first...")) {
            pendingNew_.reset();
            if (startPage_) {
                continueUntitled(); // the one to save is in the scratch file
            }
            dialog_ = Dialog::SaveAs;
            browser_.emplace(std::filesystem::current_path());
            dialogName_.clear();
            dialogError_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Replace it")) {
            auto scene = std::move(*pendingNew_);
            pendingNew_.reset();
            beginNew(std::move(scene));
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            pendingNew_.reset();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// One dialog for both: a folder, its sub-folders and scene files, and a name.
void App::drawFileDialog() {
    if (dialog_ == Dialog::None || !browser_) {
        return;
    }
    const bool opening = dialog_ == Dialog::Open;
    const char* title = opening ? "Open a scene" : "Save the scene as";
    ImGui::OpenPopup(title);
    ImGui::SetNextWindowSize(ImVec2(560, 420), ImGuiCond_Appearing);
    bool open = true;
    if (!ImGui::BeginPopupModal(title, &open)) {
        if (!open) {
            dialog_ = Dialog::None;
        }
        return;
    }
    auto& browser = *browser_;
    if (ImGui::Button("Up")) {
        browser.up();
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(browser.folder().string().c_str());
    if (!browser.error().empty()) {
        ImGui::TextColored(kProblem, "%s", browser.error().c_str());
    }

    // The folder: click a folder to go in, a file to pick its name (double
    // click to open or save it at once).
    std::optional<std::filesystem::path> chosen;
    std::optional<std::string> enter;
    ImGui::BeginChild("entries", ImVec2(0, -ImGui::GetFrameHeightWithSpacing() * 2.2F), ImGuiChildFlags_Borders);
    for (const auto& entry : browser.entries()) {
        const auto label = entry.folder ? entry.name + "/" : entry.name;
        if (ImGui::Selectable(label.c_str(), !entry.folder && entry.name == dialogName_,
                              ImGuiSelectableFlags_AllowDoubleClick)) {
            if (entry.folder) {
                enter = entry.name; // after the loop: entering changes the list
            } else {
                dialogName_ = entry.name;
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    chosen = browser.fileFor(entry.name);
                }
            }
        }
    }
    if (browser.entries().empty() && browser.error().empty()) {
        ImGui::TextColored(kDim, "No folders or scene files here.");
    }
    ImGui::EndChild();
    if (enter) {
        browser.enter(*enter);
    }

    ImGui::SetNextItemWidth(-160.0F);
    const bool typedEnter =
        ImGui::InputTextWithHint("##name", "name.json", &dialogName_, ImGuiInputTextFlags_EnterReturnsTrue);
    const auto file = browser.fileFor(dialogName_);
    std::error_code ignored;
    const bool exists = file && std::filesystem::exists(*file, ignored);
    ImGui::SameLine();
    ImGui::BeginDisabled(!file || (opening && !exists));
    const char* action = opening ? "Open" : exists ? "Replace" : "Save";
    if (ImGui::Button(action) || (typedEnter && file && (!opening || exists))) {
        chosen = file;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        open = false;
    }
    if (!opening && exists) {
        ImGui::TextColored(kProblem, "%s already exists: saving replaces it.", file->filename().string().c_str());
    } else if (opening && file && !exists) {
        ImGui::TextColored(kDim, "No such file here.");
    } else if (!dialogError_.empty()) {
        ImGui::TextColored(kProblem, "%s", dialogError_.c_str());
    }

    if (chosen) {
        if (opening) {
            open = !openScene(*chosen); // a file that can't be read leaves the dialog open, with the reason
        } else {
            saveSceneAs(*chosen);
            open = scenePath_ != std::filesystem::absolute(*chosen).lexically_normal(); // not if it couldn't be written
        }
        if (open) {
            dialogError_ = fileStatus_;
        }
    }
    if (!open) {
        dialog_ = Dialog::None;
        browser_.reset();
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

} // namespace fsim
