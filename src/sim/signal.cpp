#include "sim/signal.hpp"
#include <algorithm>

namespace fsim {

bool Signal::valueAt(double t) const {
    // The last change at or before t.
    const auto after = std::upper_bound(edges_.begin(), edges_.end(), t,
                                        [](double time, const Edge& edge) { return time < edge.time; });
    if (after == edges_.begin()) {
        return edges_.empty() ? value_ : !edges_.front().value; // before the oldest remembered change
    }
    return std::prev(after)->value;
}

void Signal::set(double time, bool value) {
    if (value == value_) {
        return;
    }
    value_ = value;
    edges_.push_back(Edge{time, value});
    if (edges_.size() > kHistory) {
        edges_.pop_front();
    }
}

} // namespace fsim
