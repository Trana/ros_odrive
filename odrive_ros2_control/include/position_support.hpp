#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

namespace odrive_ros2_control {

// Serialized by the transport owner. ROS claims and native receipt storage stay
// outside this value. Only a complete submitted native round can replace its hold.
class PositionSupport {
public:
    enum class Source { inactive, measured, native, retained };

    void configure(size_t axes) { targets_.resize(axes); }
    void begin(std::span<const double> measured) {
        std::copy(measured.begin(), measured.end(), targets_.begin());
        source_ = Source::measured;
        first_native_ = last_native_ = 0;
    }
    void admit_native(uint64_t generation) {
        if (source_ == Source::inactive || source_ == Source::retained)
            return;
        if (!first_native_)
            first_native_ = generation;
        last_native_ = generation;
        source_ = Source::native;
    }
    void retain() {
        if (source_ == Source::native)
            source_ = Source::retained;
    }
    void completed(uint64_t generation, std::span<const double> positions) {
        if (!first_native_ || generation < first_native_ || generation > last_native_
            || positions.size() != targets_.size()
            || !std::all_of(positions.begin(), positions.end(), [](double value) { return std::isfinite(value); }))
            return;
        std::copy(positions.begin(), positions.end(), targets_.begin());
    }
    void stop() { source_ = Source::inactive; }
    Source source() const { return source_; }
    bool producing_hold() const { return source_ == Source::measured || source_ == Source::retained; }
    double target(size_t axis) const { return targets_.at(axis); }

private:
    Source source_ = Source::inactive;
    std::vector<double> targets_;
    uint64_t first_native_ = 0, last_native_ = 0;
};
} // namespace odrive_ros2_control
