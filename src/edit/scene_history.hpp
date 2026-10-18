#pragma once
#include "sim/scene.hpp"
#include <cstddef>
#include <vector>

// Undo and redo for the scene, by snapshots: a Scene is a small value, so
// each step keeps the whole scene as it was (the same idea as the undo of
// ladder_editor's rungs, without any command classes).
//
// The panels edit `current()` in place, every frame while a slider moves.
// commit() is called when no edit is in progress (the slider was let go):
// if the scene changed since the last commit, that's one step. So dragging
// a slider from 1 to 3 m is one undo step, not sixty.
namespace fsim {

class SceneHistory {
public:
    explicit SceneHistory(Scene initial) : current_(initial), committed_(std::move(initial)) {}

    Scene& current() { return current_; }
    const Scene& current() const { return current_; }

    // Records the changes since the last commit as one step. Returns true if
    // there were any (so the caller can save the scene).
    bool commit();

    bool canUndo() const { return !undo_.empty() || current_ != committed_; }
    bool canRedo() const { return !redo_.empty(); }
    void undo();
    void redo();

    static constexpr std::size_t kLimit = 200;

private:
    Scene current_;   // what the panels edit
    Scene committed_; // the scene as of the last step
    std::vector<Scene> undo_;
    std::vector<Scene> redo_;
};

} // namespace fsim
