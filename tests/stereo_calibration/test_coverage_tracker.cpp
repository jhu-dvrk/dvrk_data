// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/coverage_tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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

        CoverageTracker coverage;
        coverage.add_frame(original.corners, original.ids, original.object_points, image_size,
                           {0,0,0}, true, 7, original.corners, {-15,-10,120}, true);
        auto stats = coverage.get_stats();
        expect(stats.sampled_cells == 0, "One frame must not count as repeated spatial coverage");
        std::string reason;
        coverage.is_novel_coverage(original.corners, original.ids, original.object_points, image_size, &reason);
        expect(reason.find("#7") != std::string::npos, "Duplicate reason must identify sample ID");
        coverage.add_frame(original.corners, original.ids, original.object_points, image_size,
                           {0,0,0}, true, 8, original.corners, {-15,-10,120}, true);
        expect(coverage.get_stats().sampled_cells > 0 && coverage.get_stats().right_spatial_coverage_ratio > 0,
               "Repeated samples must update coverage in both cameras");
        const auto known_samples = coverage.nested_coverage().samples;
        coverage.add_frame(translated.corners, translated.ids, translated.object_points, image_size,
                           {}, false, 9, translated.corners);
        expect(coverage.nested_coverage().samples == known_samples,
               "Unknown poses must not invent depth or orientation coverage");
        coverage.reset();
        expect(coverage.get_stats().frame_count == 0 && coverage.get_stats().sampled_cells == 0 &&
               coverage.get_stats().right_spatial_coverage_ratio == 0,
               "Reset must clear coverage in both cameras");
        // Corner-based joint coverage: multiple corners in one full bin count as
        // one image, while edge corners contribute even if the board center does not.
        CoverageTracker joint;
        View edge = original;
        edge.corners.back() = {639,479};
        auto add_joint = [&](const View &v, int id, double z, cv::Vec3d angle = {0,0,0}) {
            joint.add_frame(v.corners,v.ids,v.object_points,image_size,angle,true,id,v.corners,
                            {0,0,z},true,{{0,0.4,0},{0,0,z+10},true});
        };
        add_joint(edge,10,100);
        add_joint(original,20,300,{0.4,0,0});
        const auto both = joint.nested_coverage();
        const auto right = joint.nested_coverage(true);
        expect(both.range.valid && both.range.near_mm < 100 && both.range.far_mm > 300,
               "Auto range must include corner depths from both cameras");
        expect(right.range.near_mm == both.range.near_mm && right.range.far_mm == both.range.far_mm,
               "Both cameras must use the same depth scale");
        const int near_depth = (100-both.range.near_mm)*both.depth_bins/(both.range.far_mm-both.range.near_mm);
        expect(both.count(5,3,near_depth,1,1) == 1,
               "Border corner must contribute without the board center");
        expect(*std::max_element(both.corners.begin(),both.corners.end()) > 1 &&
               *std::max_element(both.samples.begin(),both.samples.end()) == 1,
               "Multiple corners must not inflate the image count within a full bin");
        expect(std::accumulate(right.samples.begin(),right.samples.end(),0) > 0,
               "Right-eye joint coverage missing");
        joint.reset(); // Rebuild exactly as the GUI does after removing an outlier image.
        add_joint(edge,10,100);
        const auto retained = joint.nested_coverage();
        expect(retained.range.far_mm < both.range.far_mm && retained.range.far_mm < 150,
               "Removing far sample must shrink the auto depth range");
        expect(std::accumulate(retained.corners.begin(),retained.corners.end(),0) == static_cast<int>(edge.ids.size()),
               "Removed sample corners remain in coverage");
        edge.corners.pop_back(); edge.ids.pop_back(); edge.object_points.pop_back();
        joint.reset(); // Rebuild after excluding an individual matching corner.
        add_joint(edge,10,100);
        auto edited = joint.nested_coverage();
        int edge_count = 0;
        for (int d = 0; d < edited.depth_bins; ++d) for (int p = 0; p < 3; ++p) for (int t = 0; t < 3; ++t)
            edge_count += edited.count(5,3,d,p,t);
        expect(edge_count == 0, "Excluded corner left a stale edge contribution");
        expect(std::accumulate(edited.corners.begin(),edited.corners.end(),0) == static_cast<int>(edge.ids.size()),
               "Corner exclusion must update raw counts too");
        View depth_edge = original;
        depth_edge.object_points.back().x = 100;
        joint.reset();
        add_joint(depth_edge,40,100,{0,-0.7,0});
        const auto with_extreme_corner = joint.nested_depth_range();
        depth_edge.corners.pop_back(); depth_edge.ids.pop_back(); depth_edge.object_points.pop_back();
        joint.reset();
        add_joint(depth_edge,40,100,{0,-0.7,0});
        expect(joint.nested_depth_range().far_mm < with_extreme_corner.far_mm,
               "Removing an extreme-depth corner must shrink the auto range");
        cv::Mat camera(image_size,CV_8UC3,cv::Scalar(80,120,160));
        cv::rectangle(camera,{130,90,250,240},cv::Scalar(220,220,220),cv::FILLED);
        const auto camera_before = camera.clone();
        const auto overlay = CoverageTracker::draw_nested_overlay(camera,both);
        expect(overlay.image.size() == image_size && cv::norm(camera,camera_before) == 0,
               "Live overlay changed source pixels or image dimensions");
        cv::Mat difference, mask;
        cv::absdiff(camera,overlay.image,difference);
        cv::cvtColor(difference,mask,cv::COLOR_BGR2GRAY);
        expect(cv::countNonZero(mask) > 0 && cv::mean(overlay.image)[0] > 20,
               "Translucent depth bands must preserve visible camera content");
        const auto right_overlay = CoverageTracker::draw_nested_overlay(camera,right,true);
        expect(overlay.tiles.size() == 24*4 && right_overlay.tiles.front().right_eye,
               "Overlay bucket layout or per-camera hover metadata missing");
        for (const auto &tile : overlay.tiles) {
            expect(tile.bounds.width > 0 && tile.bounds.height > 0, "Invisible coverage band");
            expect(CoverageTracker::describe_nested_tile(both,tile).find("retained images") != std::string::npos,
                   "Hover details missing image counts");
        }
        if (std::getenv("DVRK_CALIBRATION_PREVIEWS")) cv::imwrite("/tmp/charuco-build/overlay.png",overlay.image);
        // Frontal edge midpoints and four tilted corners must blend smoothly around each band.
        auto directional = both;
        std::fill(directional.samples.begin(),directional.samples.end(),0);
        directional.samples[directional.index(5,3,0,0,0)] = 1;
        directional.samples[directional.index(5,3,0,2,0)] = 3;
        directional.samples[directional.index(5,3,0,0,2)] = 8;
        const cv::Mat gradient_background(1000,1440,CV_8UC3,cv::Scalar(19,21,24));
        const auto gradient_chart = CoverageTracker::draw_nested_overlay(gradient_background,directional,false,0.6);
        const auto &near_band = gradient_chart.tiles.back();
        std::set<std::array<int,3>> colors;
        const int gy = near_band.bounds.y+3;
        for (int gx = near_band.bounds.x+3; gx < near_band.bounds.br().x-3; ++gx) {
            const auto c = gradient_chart.image.at<cv::Vec3b>(gy,gx);
            colors.insert({c[0],c[1],c[2]});
        }
        expect(colors.size() > 20, "Depth band still uses discrete orientation cells instead of a smooth gradient");
        auto frontal = directional;
        std::fill(frontal.samples.begin(),frontal.samples.end(),0);
        frontal.samples[frontal.index(5,3,0,1,1)] = 20;
        const auto frontal_chart = CoverageTracker::draw_nested_overlay(gradient_background,frontal,false,0.6);
        const auto &fb = frontal_chart.tiles.back().bounds;
        const auto empty_gradient = CoverageTracker::draw_nested_overlay(gradient_background,
            [&] { auto empty = frontal; std::fill(empty.samples.begin(),empty.samples.end(),0); return empty; }(),false,0.6);
        const int cx=fb.x+(fb.width-1)/2, cy=fb.y+(fb.height-1)/2;
        for (const auto &point : std::vector<cv::Point>{{cx,fb.y},{cx,fb.br().y-1},{fb.x,cy},{fb.br().x-1,cy}}) {
            const auto color=frontal_chart.image.at<cv::Vec3b>(point);
            expect(color[1]>color[2]+100, "Front-facing density must be green at N/S/E/W edge midpoints");
        }
        for (const auto &point : std::vector<cv::Point>{fb.tl(),{fb.br().x-1,fb.y},
                {fb.x,fb.br().y-1},{fb.br().x-1,fb.br().y-1}}) {
            expect(frontal_chart.image.at<cv::Vec3b>(point)==empty_gradient.image.at<cv::Vec3b>(point),
                   "Frontal observations must not clear the tilted diagonal corners");
        }
        // Verify the same representation in the live overlay, including removal.
        auto frontal_live=CoverageTracker::draw_nested_overlay(camera,frontal);
        const auto &live_band=frontal_live.tiles.back().bounds;
        const cv::Point north(live_band.x+(live_band.width-1)/2,live_band.y);
        auto empty=frontal; std::fill(empty.samples.begin(),empty.samples.end(),0);
        const auto removed_live=CoverageTracker::draw_nested_overlay(camera,empty);
        expect(frontal_live.image.at<cv::Vec3b>(north)[1] > removed_live.image.at<cv::Vec3b>(north)[1]+50,
               "Live frontal density must disappear when its retained observations are removed");
        auto angled=empty;
        for (int pan : {0,2}) for (int tilt : {0,2})
            angled.samples[angled.index(5,3,0,pan,tilt)]=angled.target_samples;
        const auto angled_chart=CoverageTracker::draw_nested_overlay(gradient_background,angled,false,0.6);
        expect(angled_chart.image.at<cv::Vec3b>(fb.tl())[1]>angled_chart.image.at<cv::Vec3b>(fb.tl())[2]+100 &&
               angled_chart.image.at<cv::Vec3b>(fb.y,cx)[2]>angled_chart.image.at<cv::Vec3b>(fb.y,cx)[1]+100,
               "Tilted density must fill diagonal corners while frontal midpoints stay missing");
        auto uniform = directional;
        uniform.target_samples = 8;
        std::fill(uniform.samples.begin(),uniform.samples.end(),0);
        const auto missing = CoverageTracker::draw_nested_overlay(camera,uniform);
        std::fill(uniform.samples.begin(),uniform.samples.end(),4);
        const auto half = CoverageTracker::draw_nested_overlay(camera,uniform);
        std::fill(uniform.samples.begin(),uniform.samples.end(),uniform.target_samples);
        const auto complete = CoverageTracker::draw_nested_overlay(camera,uniform);
        expect(cv::norm(complete.opacity,missing.opacity,cv::NORM_INF) == 0 &&
               cv::norm(half.opacity,missing.opacity,cv::NORM_INF) == 0 &&
               std::abs(complete.opacity.at<float>(20,50)-0.30) < 1e-6,
               "Coverage opacity must stay at 30% for all sample densities");
        const cv::Point probe(50,20);
        expect(missing.image.at<cv::Vec3b>(probe)[2] > half.image.at<cv::Vec3b>(probe)[2] &&
               half.image.at<cv::Vec3b>(probe)[2] > complete.image.at<cv::Vec3b>(probe)[2] &&
               missing.image.at<cv::Vec3b>(probe)[1] < half.image.at<cv::Vec3b>(probe)[1] &&
               half.image.at<cv::Vec3b>(probe)[1] < complete.image.at<cv::Vec3b>(probe)[1],
               "Coverage must progress from red to green as sample density increases");
        expect(half.image.at<cv::Vec3b>(13,50) == half.image.at<cv::Vec3b>(14,50) &&
               half.image.at<cv::Vec3b>(25,50) == half.image.at<cv::Vec3b>(26,50),
               "Depth-band separators remain visible with uniform coverage");
        const auto saved_counts = uniform.samples;
        const auto saved_range = uniform.range;
        auto blue = complete.image.clone();
        CoverageTracker::draw_live_coverage_tint(blue,uniform,complete,original.corners,
            original.object_points,{{0,0,0},{0,0,100},true});
        expect(cv::norm(blue,complete.image) > 0 && uniform.samples == saved_counts &&
               uniform.range.near_mm == saved_range.near_mm && uniform.range.far_mm == saved_range.far_mm,
               "Blue live footprint must be visible without altering saved coverage or depth bounds");
        auto moved_corners = original.corners;
        for (auto &point : moved_corners) point.x += 180;
        auto moved_blue = complete.image.clone();
        CoverageTracker::draw_live_coverage_tint(moved_blue,uniform,complete,moved_corners,
            original.object_points,{{0,0,0},{0,0,100},true});
        expect(cv::norm(moved_blue,blue) > 0, "Blue footprint did not follow board movement");
        for (const auto &tile : complete.tiles) if (tile.x == 1 && tile.y == 1 && tile.depth == 0) {
            const int thickness=std::min(tile.bounds.width,tile.bounds.height)/2;
            const cv::Point point(tile.bounds.x+tile.bounds.width/2,tile.bounds.y+thickness/2);
            const cv::Point center(tile.bounds.x+tile.bounds.width/2,tile.bounds.y+tile.bounds.height/2);
            expect(blue.at<cv::Vec3b>(center)==complete.image.at<cv::Vec3b>(center),
                   "The center of the nearest density bucket must stay visible");
            expect(blue.at<cv::Vec3b>(point)[0] > complete.image.at<cv::Vec3b>(point)[0] &&
                   moved_blue.at<cv::Vec3b>(point) == complete.image.at<cv::Vec3b>(point),
                   "Blue tint did not clear the previous board position");
        }
        // A normal depth ring keeps both red/green edges untouched, with blue
        // only in its middle third on all four sides.
        auto middle_blue=complete.image.clone();
        const double middle_z=uniform.range.near_mm+2.5*(uniform.range.far_mm-uniform.range.near_mm)/uniform.depth_bins;
        CoverageTracker::draw_live_coverage_tint(middle_blue,uniform,complete,original.corners,
            original.object_points,{{0,0,0},{0,0,middle_z},true});
        for (const auto &tile:complete.tiles) if (tile.x==1 && tile.y==1 && tile.depth==2) {
            const auto &b=tile.bounds; const auto &hole=tile.inner_bounds;
            const int t=hole.x-b.x, cx=b.x+b.width/2, cy=b.y+b.height/2;
            const std::vector<cv::Point> blue_points{{cx,b.y+t/2},{cx,b.br().y-1-t/2},
                {b.x+t/2,cy},{b.br().x-1-t/2,cy}};
            for (const auto &p:blue_points) expect(middle_blue.at<cv::Vec3b>(p)!=complete.image.at<cv::Vec3b>(p),
                "Live marker must remain visible in the middle third of each side");
            const std::vector<cv::Point> density_points{{cx,b.y+1},{cx,hole.y-1},
                {cx,hole.br().y},{cx,b.br().y-2},{b.x+1,cy},{hole.x-1,cy},
                {hole.br().x,cy},{b.br().x-2,cy}};
            for (const auto &p:density_points) expect(middle_blue.at<cv::Vec3b>(p)==complete.image.at<cv::Vec3b>(p),
                "Blue live tint must not cover either density edge");
        }
        auto lost = complete.image.clone();
        CoverageTracker::draw_live_coverage_tint(lost,uniform,complete,{},original.object_points,{});
        expect(cv::norm(lost,complete.image) == 0, "Lost detection must leave no blue footprint");
        if (std::getenv("DVRK_CALIBRATION_PREVIEWS")) {
            cv::imwrite("/tmp/charuco-build/gradient-detail.png",gradient_chart.image);
            cv::imwrite("/tmp/charuco-build/blue-live.png",blue);
            cv::imwrite("/tmp/charuco-build/blue-middle-band.png",middle_blue);
            cv::imwrite("/tmp/charuco-build/half-red.png",half.image);
            cv::imwrite("/tmp/charuco-build/gradient-overlay.png",CoverageTracker::draw_nested_overlay(camera,directional).image);
        }
        // Density is a display/collection target: changing it must recolor existing
        // observations without modifying counts, depth range, or opacity.
        auto lower_target = uniform;
        std::fill(lower_target.samples.begin(),lower_target.samples.end(),2);
        lower_target.target_samples = 2;
        auto higher_target = lower_target; higher_target.target_samples = 4;
        const auto low_overlay = CoverageTracker::draw_nested_overlay(camera,lower_target);
        const auto high_overlay = CoverageTracker::draw_nested_overlay(camera,higher_target);
        expect(cv::norm(low_overlay.image,high_overlay.image) > 0 &&
               cv::norm(low_overlay.opacity,high_overlay.opacity) == 0 && lower_target.samples == higher_target.samples,
               "Changing density must recolor retained data at fixed opacity");
        using dvrk_data::calibration::NestedCoverageData;
        NestedCoverageData small;
        small.cols=1; small.rows=1; small.range={true,50,150}; small.target_samples=2;
        small.samples.assign(4*9,0); small.corners.assign(4*9,0);
        for (int z=0; z<4; ++z) for (int pan : {0,2}) for (int tilt : {0,2})
            small.samples[small.index(0,0,z,pan,tilt)] = 1;
        for (int z=0; z<4; ++z) small.samples[small.index(0,0,z,1,1)] = 1;
        const auto density = CoverageTracker::density_summary(small,small);
        expect(density.left_fraction == 0.5 && density.right_fraction == 0.5,
               "Density credit is incorrect");
        auto frontal_only=small;
        std::fill(frontal_only.samples.begin(),frontal_only.samples.end(),0);
        for (int z=0;z<4;++z) frontal_only.samples[frontal_only.index(0,0,z,1,1)]=100;
        const auto frontal_progress=CoverageTracker::density_summary(frontal_only,frontal_only);
        expect(std::abs(frontal_progress.left_fraction-0.2)<1e-9,
               "Repeated frontal edge midpoints must count as one of five orientation bins");
        auto higher = small; higher.target_samples=4;
        const auto more = CoverageTracker::density_summary(higher,higher);
        expect(more.left_fraction == 0.25 && more.right_fraction == 0.25,
               "Increasing density must lower progress");
        auto fulfilled = small; fulfilled.target_samples=1;
        const auto complete_density = CoverageTracker::density_summary(fulfilled,fulfilled);
        expect(complete_density.left_fraction == 1 && complete_density.right_fraction == 1,
               "Completed density must reach 100 percent");
        auto excess = fulfilled;
        excess.samples[excess.index(0,0,0,0,0)]=100;
        expect(CoverageTracker::density_summary(excess,excess).left_fraction == 1,
               "Excess observations must not increase density beyond 100 percent");
        auto removed = small;
        std::fill(removed.samples.begin(),removed.samples.end(),0);
        const auto removed_summary = CoverageTracker::density_summary(removed,removed);
        expect(removed_summary.left_fraction == 0 && removed_summary.right_fraction == 0,
               "Removing samples must reset density progress");
        // Per-corner Z, rather than one board-center Z, must span depth layers.
        joint.reset();
        add_joint(original,30,100,{0,0.7,0});
        edited = joint.nested_coverage();
        std::set<int> occupied_depths;
        for (int y = 0; y < edited.rows; ++y) for (int x = 0; x < edited.cols; ++x)
            for (int d = 0; d < edited.depth_bins; ++d) for (int p = 0; p < 3; ++p) for (int t = 0; t < 3; ++t)
                if (edited.count(x,y,d,p,t)) occupied_depths.insert(d);
        expect(occupied_depths.size() > 1, "Tilted board corners must contribute at their individual depths");
        joint.reset();
        expect(!joint.nested_coverage().range.valid, "Empty collection must not retain a depth range");
        CoverageTracker density_gate;
        auto add_gate = [&](const View &v,int id) {
            density_gate.add_frame(v.corners,v.ids,v.object_points,image_size,{},true,id,v.corners,{0,0,100},true,{{0,0,0},{0,0,100},true});
        };
        auto useful = [&](const View &v,int target) {
            return density_gate.is_useful_capture(v.corners,v.corners,v.ids,v.object_points,image_size,
                {{0,0,0},{0,0,100},true},{{0,0,0},{0,0,100},true},target);
        };
        add_gate(original,1);
        expect(!useful(original,20), "High density target must not admit a stationary duplicate");
        View shifted=original;
        for(auto &p:shifted.corners) p.x+=4;
        expect(!novel(density_gate,shifted) && useful(shifted,2),
               "A small real pose change must be allowed to fill missing density");
        add_gate(shifted,2);
        View shifted_again=original;
        for(auto &p:shifted_again.corners) p.x+=8;
        expect(!useful(shifted_again,2) && useful(shifted_again,3),
               "Similar views must stop at the density target and resume when it increases");
        View jitter=original;
        for(auto &p:jitter.corners) { p.x+=0.2; p.y-=0.1; }
        expect(!useful(jitter,20), "Subpixel jitter must not manufacture density");
        std::cout << "Coverage novelty regression checks PASSED\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Coverage novelty regression FAILED: " << e.what() << '\n';
        return 1;
    }
}
