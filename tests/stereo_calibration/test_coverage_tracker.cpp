// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/coverage_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using dvrk_data::calibration::CoverageTracker;

namespace {

const cv::Size image_size(640, 480);

struct View {
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    std::vector<cv::Point3f> object_points;
};

View board_view() {
    View v;
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 6; ++x) {
            v.corners.emplace_back(200 + 30 * x, 150 + 30 * y);
            v.ids.push_back(y * 6 + x);
            v.object_points.emplace_back(5 * x, 5 * y, 0);
        }
    }
    return v;
}

View select(const View &v, int first, int last) {
    View subset;
    for (int i = first; i < last; ++i) {
        subset.corners.push_back(v.corners[i]);
        subset.ids.push_back(v.ids[i]);
        subset.object_points.push_back(v.object_points[i]);
    }
    return subset;
}

void add(CoverageTracker &tracker, const View &v,
         cv::Vec3d rvec = cv::Vec3d(0, 0, 0)) {
    tracker.add_frame(v.corners, v.ids, v.object_points, image_size, rvec, true);
}

bool novel(const CoverageTracker &tracker, const View &v) {
    return tracker.is_novel_coverage(v.corners, v.ids, v.object_points, image_size);
}

void expect(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

} // namespace

int main() {
    try {
        const View original = board_view();
        CoverageTracker tracker;
        expect(novel(tracker, original), "First valid view must be accepted");
        add(tracker, original, cv::Vec3d(CV_PI - 0.001, 0, 0));
        for (int i = 0; i < 20; ++i) {
            expect(!novel(tracker, original), "Coverage counts must never admit identical views");
        }
        expect(tracker.get_stats().frame_count == 1, "One static view should remain one sample");
        View bad_corner = original;
        bad_corner.corners[0].x += 100;
        expect(!novel(tracker, bad_corner), "One bad corner must not create a new view");

        cv::RNG rng(42);
        for (int i = 0; i < 300; ++i) {
            View noisy = original;
            for (auto &p : noisy.corners) {
                p.x += rng.gaussian(0.15);
                p.y += rng.gaussian(0.15);
            }
            expect(!novel(tracker, noisy), "Small corner noise must not admit a stationary view");
        }

        View reordered = original;
        std::reverse(reordered.corners.begin(), reordered.corners.end());
        std::reverse(reordered.ids.begin(), reordered.ids.end());
        std::reverse(reordered.object_points.begin(), reordered.object_points.end());
        expect(!novel(tracker, reordered), "Corner order must not change novelty");
        const auto upper = select(original, 0, 12);
        const auto lower = select(original, 12, 24);
        for (int i = 0; i < 20; ++i) {
            expect(!novel(tracker, i % 2 ? lower : upper),
                   "Detected subsets of a stationary board must remain duplicates");
        }
        CoverageTracker partial;
        add(partial, upper);
        expect(!novel(partial, lower), "Disjoint corner IDs at the same pose must remain duplicates");
        View shifted_partial = lower;
        for (auto &p : shifted_partial.corners) p.x += 40;
        expect(novel(partial, shifted_partial), "A moved partial view must still be accepted");

        View translated = original;
        for (auto &p : translated.corners) p.x += 40;
        expect(novel(tracker, translated), "New image position must be accepted");
        add(tracker, translated);
        expect(!novel(tracker, original), "A-B-A must reject A against the full sample history");
        expect(!novel(tracker, translated), "The most recent view must also remain a duplicate");

        View scaled = original, rotated = original, tilted = original;
        const cv::Point2f center(275, 195);
        const double angle = 15 * CV_PI / 180;
        for (size_t i = 0; i < original.corners.size(); ++i) {
            auto p = original.corners[i] - center;
            scaled.corners[i] = center + p * 1.25f;
            rotated.corners[i] = center + cv::Point2f(
                std::cos(angle) * p.x - std::sin(angle) * p.y,
                std::sin(angle) * p.x + std::cos(angle) * p.y);
            tilted.corners[i] = center + cv::Point2f(0.65f * p.x, p.y);
        }
        expect(novel(tracker, scaled), "New distance/scale must be accepted");
        expect(novel(tracker, rotated), "In-plane rotation must be accepted");
        expect(novel(tracker, tilted), "Changed board tilt/shape must be accepted");

        CoverageTracker equivalent_pose;
        add(equivalent_pose, original, cv::Vec3d(-CV_PI + 0.001, 0, 0));
        expect(!novel(equivalent_pose, original), "Rotation-vector wrapping must not create novelty");
        expect(!tracker.is_novel_coverage(original.corners, original.ids, original.object_points,
                                          cv::Size(320, 240)), "Resolution changes must be rejected");

        View invalid = original;
        invalid.ids.pop_back();
        expect(!novel(tracker, invalid), "Mismatched correspondence counts must be rejected");
        invalid = original;
        invalid.ids[1] = invalid.ids[0];
        expect(!novel(tracker, invalid), "Duplicate corner IDs must be rejected");
        invalid = original;
        invalid.corners[0].x = std::numeric_limits<float>::quiet_NaN();
        expect(!novel(tracker, invalid), "Non-finite corners must be rejected");
        invalid = original;
        for (auto &p : invalid.object_points) p.y = 0;
        expect(!novel(tracker, invalid), "Collinear board correspondences must be rejected");

        tracker.reset();
        expect(tracker.get_stats().frame_count == 0, "Reset must clear sample history");
        expect(novel(tracker, original), "Reset must allow a previously collected view");
        add(tracker, translated);
        expect(novel(tracker, original), "Rebuilding with retained samples must forget deleted views");

        std::cout << "Coverage novelty regression checks PASSED\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Coverage novelty regression FAILED: " << e.what() << '\n';
        return 1;
    }
}
