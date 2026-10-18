#include "edit/scene_history.hpp"
#include <doctest/doctest.h>

using namespace fsim;

TEST_CASE("Changes made between two commits are one undo step") {
    SceneHistory history(defaultScene());
    for (int frame = 1; frame <= 60; ++frame) { // a slider dragged over 60 frames
        history.current().conveyors[0].spec.length = 3.0F + static_cast<float>(frame) / 60.0F;
    }
    CHECK(history.commit());
    CHECK_FALSE(history.commit()); // nothing new
    history.undo();
    CHECK(history.current().conveyors[0].spec.length == 3.0F);
    CHECK_FALSE(history.canUndo());
    history.redo();
    CHECK(history.current().conveyors[0].spec.length == 4.0F);
}

TEST_CASE("Undo first records an edit still in progress, then undoes it") {
    SceneHistory history(defaultScene());
    addPhotocell(history.current(), 0);
    CHECK(history.canUndo()); // not committed yet, but there's something to undo
    history.undo();
    CHECK(history.current().photocells.size() == 1);
    CHECK(history.canRedo());
}

TEST_CASE("A new change after an undo forgets what was undone") {
    SceneHistory history(defaultScene());
    addPhotocell(history.current(), 0);
    history.commit();
    history.undo();
    addSpawner(history.current(), 0);
    history.commit();
    CHECK_FALSE(history.canRedo());
    CHECK(history.current().photocells.size() == 1);
    CHECK(history.current().spawners.size() == 2);
}
