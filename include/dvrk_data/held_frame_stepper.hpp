#ifndef DVRK_DATA_HELD_FRAME_STEPPER_HPP
#define DVRK_DATA_HELD_FRAME_STEPPER_HPP

#include <algorithm>
#include <cmath>

namespace dc {
// Counts encoded frames to seek, using elapsed time rather than key repeats.
class HeldFrameStepper {
public:
    void reset(double now) {
        start_ = last_ = now;
        credit_ = 0.0;
    }

    long long advance(double now, double fps, double max_speed) {
        if (now <= last_) return 0;
        const double begin = std::max(0.0, last_ - start_ - repeat_delay);
        const double end = std::max(0.0, now - start_ - repeat_delay);
        last_ = now;
        const double cap = std::max(0.0, fps * max_speed);
        const double initial = std::min(2.0, cap);
        auto distance = [initial, cap](double t) {
            const double ramp = std::min(t, ramp_duration);
            return initial * ramp + (cap - initial) * ramp * ramp / (2.0 * ramp_duration)
                   + cap * std::max(0.0, t - ramp_duration);
        };
        credit_ += distance(end) - distance(begin);
        const auto frames = static_cast<long long>(std::floor(credit_ + 1e-9));
        credit_ -= frames;
        return frames;
    }

    static constexpr double repeat_delay = 0.25;
    static constexpr double ramp_duration = 3.0;

private:
    double start_ = 0.0;
    double last_ = 0.0;
    double credit_ = 0.0;
};
} // namespace dc

#endif
