#include "view_transform.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, const char* message) {
    check(std::abs(actual - expected) < 1e-8, message);
}

int main() {
    try {
        VideoViewTransform view;
        auto initial = view.placement(800, 450, 3840, 1080);
        near(initial.scale, 800. / 3840, "fit stereo image width");
        near(initial.x, 0, "horizontal fit");
        near(initial.y, 112.5, "letterbox centered");
        const double mouse_x = 210, mouse_y = 170;
        const double image_x = (mouse_x - initial.x) / initial.scale;
        const double image_y = (mouse_y - initial.y) / initial.scale;
        view.scroll(-1, mouse_x, mouse_y, 800, 450, 3840, 1080);
        auto zoomed = view.placement(800, 450, 3840, 1080);
        near(zoomed.x + image_x * zoomed.scale, mouse_x, "mouse image x stays fixed");
        near(zoomed.y + image_y * zoomed.scale, mouse_y, "mouse image y stays fixed");
        view.pan_x += 35;
        view.pan_y -= 20;
        auto panned = view.placement(800, 450, 3840, 1080);
        near(panned.x - zoomed.x, 35, "drag x in display pixels");
        near(panned.y - zoomed.y, -20, "drag y in display pixels");
        view.scroll(-1000, 200, 200, 800, 450, 3840, 1080);
        near(view.zoom, 32, "upper zoom bound");
        view.scroll(1000, 200, 200, 800, 450, 3840, 1080);
        near(view.zoom, 1, "lower zoom bound");
        near(view.pan_x, 0, "zooming out resets pan x");
        near(view.pan_y, 0, "zooming out resets pan y");
        view.scroll(std::numeric_limits<double>::quiet_NaN(), 0, 0, 800, 450, 3840, 1080);
        near(view.zoom, 1, "invalid scroll ignored");
        view.scroll(-1, 0, 0, 0, 0, 3840, 1080);
        near(view.zoom, 1, "zero viewport ignored");
        view.scroll(-1, 400, 225, 800, 450, 3840, 1080);
        view.reset();
        auto fit = view.placement(800, 450, 3840, 1080);
        near(fit.scale, initial.scale, "reset fits again");
        near(fit.x, initial.x, "reset centers x");
        near(fit.y, initial.y, "reset centers y");
        std::cout << "Zoom/pan transform tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
