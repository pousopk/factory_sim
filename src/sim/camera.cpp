#include "sim/camera.hpp"

namespace fsim {

bool CameraTiming::trigger(double time, double busyTime) {
    if (busyAt(time)) {
        return false; // still on the last picture: the trigger is lost
    }
    busyUntil_ = time + busyTime;
    return true;
}

} // namespace fsim
