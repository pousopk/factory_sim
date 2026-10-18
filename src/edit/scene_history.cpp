#include "edit/scene_history.hpp"

namespace fsim {

bool SceneHistory::commit() {
    if (current_ == committed_) {
        return false;
    }
    undo_.push_back(committed_);
    if (undo_.size() > kLimit) {
        undo_.erase(undo_.begin());
    }
    redo_.clear(); // a new change: what was undone can't come back
    committed_ = current_;
    return true;
}

void SceneHistory::undo() {
    commit(); // an edit still in progress becomes a step first, so it's what gets undone
    if (undo_.empty()) {
        return;
    }
    redo_.push_back(committed_);
    committed_ = undo_.back();
    undo_.pop_back();
    current_ = committed_;
}

void SceneHistory::redo() {
    if (redo_.empty()) {
        return;
    }
    undo_.push_back(committed_);
    committed_ = redo_.back();
    redo_.pop_back();
    current_ = committed_;
}

void SceneHistory::changeEverywhere(const std::function<void(Scene&)>& change) {
    change(current_);
    change(committed_);
    for (auto& scene : undo_) {
        change(scene);
    }
    for (auto& scene : redo_) {
        change(scene);
    }
}

} // namespace fsim
