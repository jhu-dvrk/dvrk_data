#ifndef DVRK_DATA_VIDEO_TAG_VIEW_TRANSFORM_HPP
#define DVRK_DATA_VIDEO_TAG_VIEW_TRANSFORM_HPP

#include <algorithm>
#include <cmath>

struct VideoViewTransform {
    struct Placement { double scale, x, y; };
    double zoom = 1.0, pan_x = 0.0, pan_y = 0.0;

    void reset() { zoom = 1.0; pan_x = pan_y = 0.0; }

    Placement placement(double width, double height, double image_width, double image_height) const {
        if (image_width <= 0 || image_height <= 0) return {0, 0, 0};
        const double scale = std::min(width / image_width, height / image_height) * zoom;
        return {scale, (width - image_width * scale) / 2 + pan_x,
                (height - image_height * scale) / 2 + pan_y};
    }

    void scroll(double delta, double mouse_x, double mouse_y,
                double width, double height, double image_width, double image_height) {
        if (!std::isfinite(delta) || delta == 0) return;
        const auto old = placement(width, height, image_width, image_height);
        if (old.scale <= 0) return;
        zoom = std::clamp(zoom * std::pow(1.2, std::clamp(-delta, -20.0, 20.0)), 1.0, 32.0);
        if (zoom == 1.0) { reset(); return; }
        const auto next = placement(width, height, image_width, image_height);
        pan_x += mouse_x - ((mouse_x - old.x) * next.scale / old.scale + next.x);
        pan_y += mouse_y - ((mouse_y - old.y) * next.scale / old.scale + next.y);
    }
};

#endif
