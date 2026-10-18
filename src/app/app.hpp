#pragma once
#include "edit/scene_history.hpp"
#include "render/orbit_camera.hpp"
#include "render/renderer.hpp"
#include "render/scene.hpp"
#include "sim/scene.hpp"
#include "sim/world.hpp"
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <string>

// The application: the scene being designed, the world running it, the
// camera, and the panels.
//
// The scene is the source of truth. The panels edit it (with undo), it's
// saved to a file after every change, and the world follows it: settings
// that don't move anything (speeds, sensors, spawners) are applied to the
// running world live; a change to the layout builds a new world.
//
// The world runs in simulated time, in 1 ms steps; each frame advances it
// by the real time since the last frame, times the time scale (0 when
// paused).
namespace fsim {

class App {
public:
    // Loads the scene from `scenePath` if it exists (a new one otherwise),
    // and saves it there after every change. Needs a current OpenGL context.
    explicit App(std::filesystem::path scenePath);

    // One frame: handles the mouse and keys, draws the panels, follows the
    // scene's changes, advances the world by `realSeconds` (scaled), draws.
    void frame(double realSeconds, int width, int height);

private:
    void handleMouse(int width, int height);
    void handleKeys();
    void drawMainPanel();
    void drawLinePanel();
    void drawInspector();
    void drawConveyorInspector(std::size_t index);
    void drawPhotocellInspector(std::size_t index);
    void drawSpawnerInspector(std::size_t index);
    void drawPusherInspector(std::size_t index);
    void drawDiverterInspector(std::size_t index);
    void drawStopperInspector(std::size_t index);
    void drawBinInspector(std::size_t index);
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
    // The conveyor something new should go on: the selected one, or the one
    // the selected sensor or spawner is on.
    std::optional<std::size_t> selectedConveyor() const;

    Scene& scene() { return history_.current(); }

    std::filesystem::path scenePath_;
    SceneHistory history_;
    Scene applied_; // the scene the world was last made to follow
    std::unique_ptr<World> world_;
    std::optional<Pick> selected_;
    OrbitCamera camera_;
    Renderer renderer_;
    std::string fileStatus_;
    std::string programError_; // why the scene's control program couldn't start
    std::string variableFilter_;

    // The logic analyser's state.
    bool showAnalyser_ = false;
    float analyserSeconds_ = 5.0F;  // how much time the timelines show
    bool followTime_ = true;        // the timelines follow the simulation (off: look around freely)
    double viewFrom_ = 0.0;         // the time span shown, shared by the traces and the speed plot
    double viewTo_ = 5.0;
    double markerA_ = 0.0;          // two markers to measure a time difference
    double markerB_ = 0.0;
    bool markersPlaced_ = false;
    std::set<std::string> hiddenTraces_;
    int histogramPhotocell_ = 0;
    std::string eventFilter_;
    std::string exportStatus_;

    bool paused_ = false;
    float timeScale_ = 1.0F;
    double frameSeconds_ = 0.0; // the last frame's real duration, for the panel
    int stepsLastFrame_ = 0;
    float scanMilliseconds_ = 10.0F; // a PLC reading the photocell this often (for the trigger table)
    int newSorterSensor_ = 0;        // the sensor and pusher chosen for the next sorter
    int newSorterPusher_ = 0;
    std::vector<Pick> firing_; // actuators fired once by hand: they retract when fully out
};

} // namespace fsim
