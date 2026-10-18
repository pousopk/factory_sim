#pragma once
#include "analysis/tin_study.hpp"
#include "edit/file_browser.hpp"
#include "edit/placing.hpp"
#include "edit/scene_history.hpp"
#include "edit/settings.hpp"
#include "render/orbit_camera.hpp"
#include "render/renderer.hpp"
#include "render/scene.hpp"
#include "sim/scene.hpp"
#include "sim/world.hpp"
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <set>
#include <string>

// The application: the scene being designed, the world running it, the
// camera, and the panels.
//
// The scene is the source of truth. The panels edit it (with undo), it's
// saved to its file after every change (an untitled one to a scratch file
// in the config folder, until Save As names it), and the world follows it: settings
// that don't move anything (speeds, sensors, spawners) are applied to the
// running world live; a change to the layout builds a new world.
//
// The world runs in simulated time, in 1 ms steps; each frame advances it
// by the real time since the last frame, times the time scale (0 when
// paused).
namespace fsim {

class App {
public:
    // Opens `scenePath` (a new scene there if it doesn't exist); without one,
    // the most recent scene, or the untitled one. Needs a current OpenGL context.
    explicit App(std::optional<std::filesystem::path> scenePath);

    // One frame: handles the mouse and keys, draws the panels, follows the
    // scene's changes, advances the world by `realSeconds` (scaled), draws.
    void frame(double realSeconds, int width, int height);

    // For the window's title bar: "conveyor_line.json - factory_sim".
    std::string title() const;
    bool wantsToQuit() const { return quit_; } // File > Quit

private:
    // Files (files.cpp): the menu bar, the Open and Save As dialog, and what
    // they do.
    void drawMenuBar();
    void drawFileDialog();
    void drawNewConfirmation();
    // Opens a scene file: a fresh undo history and a new world. False (and
    // the current scene kept) if it can't be read.
    bool openScene(const std::filesystem::path& path);
    void saveSceneAs(const std::filesystem::path& path);
    // A new, untitled scene (empty, or a demo). The untitled scene's only
    // copy is the scratch file, so replacing an edited one asks first.
    void startNew(Scene scene);
    void replaceScene(Scene scene, std::filesystem::path path); // and its file (empty: untitled)
    void remember(const std::filesystem::path& path);           // in the recent list
    std::filesystem::path savePath() const;                     // where autosave writes
    std::filesystem::path sceneFolder() const;                  // what relative paths in the scene start from

    void handleMouse(int width, int height);
    // Editing in the view (editing.cpp): dragging the selection, placing a new
    // device by clicking on a belt, copy and paste, turning a conveyor.
    void editInView(const OrbitCamera::Ray& ray, bool pressed, bool held, bool clicked);
    void copySelection();
    void pasteClip(std::optional<BeltHit> at);
    void turnSelection(float quarterTurns);
    bool canDrag(Pick pick) const; // a device on a belt, or a conveyor standing on its own
    std::vector<DrawBox> boxesToDraw();
    std::optional<OrbitCamera::Ray> mouseRay(int width, int height) const;
    void drawNames(int width, int height); // the devices' names, over the 3D view
    void handleKeys();
    void drawMainPanel();
    void drawLinePanel();
    void drawInspector();
    void drawConveyorInspector(std::size_t index);
    void drawDrive(std::size_t conveyor);             // its drive's telegram, state and settings (drives.cpp)
    void drawConveyorEquipment(std::size_t conveyor); // zones and carriage (equipment.cpp)
    void drawPhotocellInspector(std::size_t index);
    void drawSpawnerInspector(std::size_t index);
    void drawPusherInspector(std::size_t index);
    void drawDiverterInspector(std::size_t index);
    void drawStopperInspector(std::size_t index);
    void drawBinInspector(std::size_t index);
    // The cameras (cameras.cpp): a camera's settings, the camera triggering
    // in the main panel, and the Cameras window (which camera took which
    // item, and the gap study).
    void drawCameraInspector(std::size_t index);
    void drawLineCameraSettings(CameraSpec& spec);
    // What a camera sees (camera_view.cpp): an area camera's live view and
    // last picture, a line-scan camera's growing image.
    void drawCameraView();
    std::vector<std::vector<DrawBox>> exposureMoments(const Camera& camera, double middle) const;
    // Proximity switches, lasers and light curtains (sensors.cpp).
    void drawProximityInspector(std::size_t index);
    void drawLaserInspector(std::size_t index);
    void drawCurtainInspector(std::size_t index);
    void drawCameraTriggering();
    void drawCameras();
    void drawTinTable();
    void drawGapStudy();
    // The manual controls of an actuator, and its sensors' LEDs.
    void drawStrokeControls(Stroke& stroke, Pick pick);
    void finishFiring(); // pushers fired once retract when they get there
    void drawTriggers();
    void drawIoTable();
    void drawControlPanel();
    void drawControl(); // the control section of the main panel
    void drawPlcStatus();
    void drawPlcVariables();
    // The logic analyser (analyser.cpp): timelines, histograms, event log.
    void drawAnalyser();
    void drawTimelines();
    void drawHistograms();
    void drawEventLog();
    // A new world (or program) for the scene, the PLC kinds included. The old
    // one goes first: its soft_plc may hold the monitoring port the new one needs.
    void rebuildWorld();
    void replaceProgram();
    void followScene(); // the world catches up with the scene
    void restartRun();  // a new world from the scene: no boxes, time 0
    void deleteSelected();
    void keepSelectionValid();

    Scene& scene() { return history_.current(); }

    std::filesystem::path scenePath_; // empty: untitled
    Settings settings_;
    SceneHistory history_;
    Scene applied_; // the scene the world was last made to follow
    std::unique_ptr<World> world_;
    std::optional<Pick> selected_;
    OrbitCamera camera_;
    Renderer renderer_;
    std::string fileStatus_;
    std::string programError_; // why the scene's control program couldn't start
    std::string variableFilter_;

    // Files: the dialog open (if any), its folder and typed name, and a new
    // scene waiting for "replace the untitled one?".
    enum class Dialog { None, Open, SaveAs };
    Dialog dialog_ = Dialog::None;
    std::optional<FileBrowser> browser_;
    std::string dialogName_;
    std::string dialogError_; // why the last Open or Save didn't work
    std::optional<Scene> pendingNew_;
    bool quit_ = false;

    // Which windows are shown (the View menu).
    bool showLine_ = true;
    bool showInspector_ = true;
    bool showTriggers_ = true;
    bool showIo_ = true;
    bool showControlPanel_ = true;
    bool showPlcVariables_ = true;
    bool showCameras_ = true;

    // The logic analyser's state.
    bool showAnalyser_ = false;
    float analyserSeconds_ = 5.0F; // how much time the timelines show
    bool followTime_ = true;       // the timelines follow the simulation (off: look around freely)
    double viewFrom_ = 0.0;        // the time span shown, shared by the traces and the speed plot
    double viewTo_ = 5.0;
    double markerA_ = 0.0; // two markers to measure a time difference
    double markerB_ = 0.0;
    bool markersPlaced_ = false;
    std::set<std::string> hiddenTraces_;
    int histogramPhotocell_ = 0;
    std::string eventFilter_;
    std::string exportStatus_;

    // The cameras: the trigger being added, and the gap study, which runs
    // on another thread so the window keeps drawing meanwhile.
    int newTriggerSensor_ = 0;
    int newTriggerCamera_ = 0;
    float studyUpTo_ = 0.25F; // m: the largest gap studied
    std::future<std::vector<GapOutcome>> studyRunning_;
    std::vector<GapOutcome> study_;
    Scene studied_; // the scene the study is for

    bool paused_ = false;
    float timeScale_ = 1.0F;
    double frameSeconds_ = 0.0; // the last frame's real duration, for the panel
    int stepsLastFrame_ = 0;
    double stepMilliseconds_ = 0.1; // what a simulation step has cost lately (real time)
    bool behind_ = false;           // the last frame ran out of time for the simulation
    bool showNames_ = true;         // device names over the view (View menu)

    // Editing in the view. A drag holds what's dragged and where it was
    // grabbed; meanwhile the scene changes but the world doesn't follow (a
    // moved conveyor needs a new world): a preview of the layout is drawn
    // instead, and letting go makes it one undo step.
    struct Drag {
        Pick what;
        float grab = 0.0F;     // a device: the mouse's place along its belt, less the device's
        glm::vec3 grabPoint{}; // a conveyor: where the mouse met the belt's level
        glm::vec3 startedAt{}; // and where the conveyor started then
    };
    std::optional<Drag> drag_;
    std::unique_ptr<World> preview_;    // the layout being dragged, to draw
    Scene previewed_;                   // the scene it was built from
    std::optional<Pick::Kind> placing_; // a new device waiting for a click on a belt
    std::optional<Pick> hovered_;       // the belt under the mouse while placing
    std::optional<Clip> clipboard_;

    // The Camera view window.
    bool showCameraView_ = true;
    int viewedCamera_ = 0;
    Picture livePicture_;
    Picture lastPicture_;
    double lastPictureAt_ = -1.0; // the exposure of the shot in lastPicture_
    LineTexture lineTexture_;
    std::uint64_t linesShown_ = 0;
    float scanMilliseconds_ = 10.0F; // a PLC reading the photocell this often (for the trigger table)
    int newSorterSensor_ = 0;        // the sensor and pusher chosen for the next sorter
    int newSorterPusher_ = 0;
    std::vector<Pick> firing_; // actuators fired once by hand: they retract when fully out
};

} // namespace fsim
