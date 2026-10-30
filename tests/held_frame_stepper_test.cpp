#include <dvrk_data/held_frame_stepper.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    try {
        dc::HeldFrameStepper stepper;
        stepper.reset(0);
        require(stepper.advance(0.24, 30, 5) == 0, "repeat delay");
        require(stepper.advance(0.25, 30, 5) == 0, "repeat delay boundary");
        require(stepper.advance(0.26, 30, 5) == 0, "slow initial repeats");

        for (double speed : {0.1, 0.25, 0.5, 1., 2., 3., 4., 5.}) {
            stepper.reset(0);
            long long count = 0;
            for (int i = 1; i <= 81; ++i) {
                const double t = i * 0.04;
                count += stepper.advance(t, 30, speed);
                const double elapsed = std::max(0., t - dc::HeldFrameStepper::repeat_delay);
                const double initial = std::min(2., 30 * speed);
                const double expected = initial * elapsed
                    + (30 * speed - initial) * elapsed * elapsed / (2 * dc::HeldFrameStepper::ramp_duration);
                require(std::abs(count - expected) < 1.000001, "smooth ramp and fractional frames");
                require(count <= 30 * speed * elapsed + 1e-9, "selected speed cap");
            }
            stepper.advance(3.25, 30, speed);
            for (int i = 1; i <= 5; ++i) {
                const auto count_at_cap = stepper.advance(3.25 + i, 30, speed);
                require(std::abs(count_at_cap - 30 * speed) <= 1, "holds selected maximum");
            }
        }

        // A speed change applies to subsequent ticks, not retroactively.
        stepper.reset(0);
        stepper.advance(3.25, 30, 5);
        require(stepper.advance(4.25, 30, 0.1) == 3, "lowered maximum applies immediately");
        require(stepper.advance(5.25, 30, 5) == 150, "raised maximum applies immediately");

        dc::HeldFrameStepper coarse, fine;
        coarse.reset(0);
        fine.reset(0);
        long long fine_count = 0;
        for (int i = 1; i <= 500; ++i) fine_count += fine.advance(i * 0.01, 30, 5);
        require(coarse.advance(5, 30, 5) == fine_count, "independent of timer scheduling");
        stepper.reset(100);
        require(stepper.advance(100.24, 30, 5) == 0, "new hold resets acceleration");
        require(stepper.advance(100, 30, 5) == 0, "ignore stale timer ticks");
        std::cout << "Held-frame acceleration tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
