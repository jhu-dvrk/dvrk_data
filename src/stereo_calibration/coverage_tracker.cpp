// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/coverage_tracker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <iomanip>
#include <limits>
#include <tuple>

namespace dvrk_data::calibration {

CoverageTracker::CoverageTracker(int grid_cols, int grid_rows)
    : m_grid_cols(std::max(2, grid_cols)),
      m_grid_rows(std::max(2, grid_rows)) {
    reset();
}

void CoverageTracker::reset() {
    m_cell_counts.assign(m_grid_rows, std::vector<int>(m_grid_cols, 0));
    m_right_cell_counts.assign(m_grid_rows, std::vector<int>(m_grid_cols, 0));
    m_views.clear();
    m_last_image_size = cv::Size();
}

namespace {

float footprint(const std::vector<cv::Point2f> &points) {
    cv::Point2f lo = points.front(), hi = lo;
    for (const auto &p : points) {
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
    }
    return cv::norm(hi - lo);
}

bool has_2d_spread(const std::vector<cv::Point2f> &points) {
    if (points.size() < 4) return false;
    std::vector<cv::Point2f> hull;
    cv::convexHull(points, hull);
    return std::abs(cv::contourArea(hull)) > 1e-6;
}

bool similar_projections(const std::vector<cv::Point2f> &a,
                         const std::vector<cv::Point2f> &b, bool noise_floor) {
    std::vector<float> squared_distances;
    for (size_t i = 0; i < a.size(); ++i) {
        if (!std::isfinite(a[i].x) || !std::isfinite(a[i].y) ||
            !std::isfinite(b[i].x) || !std::isfinite(b[i].y)) {
            return true; // Uncertain geometry cannot establish a novel view.
        }
        auto delta = a[i] - b[i];
        squared_distances.push_back(delta.dot(delta));
    }
    // Require movement above both a pixel-noise floor (3.2 px at 640-wide)
    // and 4% of the observed board footprint. Translation, scale, in-plane
    // rotation and perspective changes all contribute to this distance.
    const double threshold = std::max(noise_floor ? 0.002f : 0.005f,
        (noise_floor ? 0.01f : 0.04f)*std::max(footprint(a),footprint(b)));
    // One mislocalized corner must not turn a static board into a new view.
    auto middle = squared_distances.begin() + squared_distances.size() / 2;
    std::nth_element(squared_distances.begin(), middle, squared_distances.end());
    double median_squared = *middle;
    if (squared_distances.size() % 2 == 0) {
        median_squared = 0.5 * (median_squared +
            *std::max_element(squared_distances.begin(), middle));
    }
    return std::sqrt(median_squared) < threshold;
}

} // namespace

bool CoverageTracker::make_view(const std::vector<cv::Point2f> &corners,
                                const std::vector<int> &corner_ids,
                                const std::vector<cv::Point3f> &object_points,
                                const cv::Size &image_size,
                                CapturedView &view) {
    if (corners.size() < 6 || corners.size() != corner_ids.size() ||
        corners.size() != object_points.size() ||
        image_size.width <= 0 || image_size.height <= 0) {
        return false;
    }
    view = CapturedView();
    const float scale = 1.0f / std::max(image_size.width, image_size.height);
    std::set<int> ids;
    for (size_t i = 0; i < corners.size(); ++i) {
        const auto &p = corners[i];
        const auto &obj = object_points[i];
        if (corner_ids[i] < 0 || !ids.insert(corner_ids[i]).second ||
            !std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(obj.x) || !std::isfinite(obj.y) ||
            !std::isfinite(obj.z) || std::abs(obj.z) > 1e-5f) {
            return false;
        }
        view.corners.push_back(p * scale);
        view.board_points.emplace_back(obj.x, obj.y);
    }
    if (!has_2d_spread(view.board_points) || !has_2d_spread(view.corners)) return false;
    view.ids = corner_ids;
    view.board_to_image = cv::findHomography(view.board_points, view.corners, cv::RANSAC, 0.005);
    return !view.board_to_image.empty() && cv::checkRange(view.board_to_image);
}

bool CoverageTracker::views_are_close(const CapturedView &a, const CapturedView &b, bool noise_floor) {
    std::map<int, size_t> by_id;
    for (size_t i = 0; i < a.ids.size(); ++i) by_id[a.ids[i]] = i;
    std::vector<cv::Point2f> shared_board, shared_a, shared_b;
    for (size_t i = 0; i < b.ids.size(); ++i) {
        auto it = by_id.find(b.ids[i]);
        if (it != by_id.end()) {
            shared_board.push_back(b.board_points[i]);
            shared_a.push_back(a.corners[it->second]);
            shared_b.push_back(b.corners[i]);
        }
    }
    if (has_2d_spread(shared_board)) {
        // Compare the same physical corners, even if other detections appeared
        // or disappeared. Estimated 3D poses do not enter this comparison.
        return similar_projections(shared_a, shared_b, noise_floor);
    }

    // Partial endoscope views can have no common corner IDs. Compare their
    // board-to-image mappings at the same board coordinates instead of using
    // each detected subset's centroid or bounding box.
    std::vector<cv::Point2f> board = a.board_points;
    board.insert(board.end(), b.board_points.begin(), b.board_points.end());
    cv::Point2f lo = board.front(), hi = lo;
    for (const auto &p : board) {
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
    }
    std::vector<cv::Point2f> reference{{lo.x, lo.y}, {hi.x, lo.y},
                                      {hi.x, hi.y}, {lo.x, hi.y}};
    std::vector<cv::Point2f> projected_a, projected_b;
    cv::perspectiveTransform(reference, projected_a, a.board_to_image);
    cv::perspectiveTransform(reference, projected_b, b.board_to_image);
    return similar_projections(projected_a, projected_b, noise_floor);
}

void CoverageTracker::add_frame(const std::vector<cv::Point2f> &corners,
                               const std::vector<int> &corner_ids,
                               const std::vector<cv::Point3f> &object_points,
                               const cv::Size &image_size,
                               const cv::Vec3d &rvec, bool has_pose, int sample_id,
                               const std::vector<cv::Point2f> &right_corners,
                               const cv::Vec3d &tvec, bool has_translation,
                               const CoveragePose &right_pose) {
    CapturedView view;
    if (!make_view(corners, corner_ids, object_points, image_size, view)) return;
    if (!m_views.empty() && image_size != m_last_image_size) return;
    view.sample_id = sample_id;
    view.left_pixels = corners; view.right_pixels = right_corners;
    view.object_points = object_points;
    view.right_pose = right_pose;
    view.right_pose.valid = right_pose.valid && cv::checkRange(cv::Mat(right_pose.rvec)) &&
                            cv::checkRange(cv::Mat(right_pose.tvec)) && right_pose.tvec[2] > 0;
    view.rvec = rvec; view.tvec = tvec;
    view.has_translation = has_pose && has_translation && cv::checkRange(cv::Mat(rvec)) &&
                           cv::checkRange(cv::Mat(tvec)) && tvec[2] > 0;
    auto cells = [&](const std::vector<cv::Point2f> &points,
                     std::vector<std::vector<int>> &counts) {
        std::set<int> visited;
        for (const auto &p : points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
                p.x >= image_size.width || p.y >= image_size.height) continue;
            int c = static_cast<int>(p.x * m_grid_cols / image_size.width);
            int r = static_cast<int>(p.y * m_grid_rows / image_size.height);
            visited.insert(r*m_grid_cols+c);
        }
        // Count samples per cell, not individual corners from the same sample.
        for (int cell : visited) ++counts[cell/m_grid_cols][cell%m_grid_cols];
    };
    cells(corners, m_cell_counts);
    if (right_corners.size() == corners.size()) cells(right_corners, m_right_cell_counts);
    m_views.push_back(std::move(view));
    m_last_image_size = image_size;
}

bool CoverageTracker::is_novel_coverage(const std::vector<cv::Point2f> &corners,
                                       const std::vector<int> &corner_ids,
                                       const std::vector<cv::Point3f> &object_points,
                                       const cv::Size &image_size,
                                       std::string *reason_out) const {
    CapturedView candidate;
    if (!make_view(corners, corner_ids, object_points, image_size, candidate)) {
        if (reason_out) *reason_out = "Need at least 6 valid corners spanning the board";
        return false;
    }
    if (!m_views.empty() && image_size != m_last_image_size) {
        if (reason_out) *reason_out = "Resolution changed; clear samples first";
        return false;
    }
    for (size_t i = 0; i < m_views.size(); ++i) {
        if (views_are_close(candidate, m_views[i])) {
            if (reason_out) *reason_out = m_views[i].sample_id > 0
                ? "Similar to sample #" + std::to_string(m_views[i].sample_id)
                : "Similar to an existing sample";
            return false;
        }
    }
    if (reason_out) *reason_out = m_views.empty() ? "First frame" : "New board view";
    return true;
}

CoverageStats CoverageTracker::get_stats(int target_samples) const {
    CoverageStats stats;
    stats.total_cells = m_grid_cols * m_grid_rows;
    stats.sampled_cells = 0;
    stats.frame_count = static_cast<int>(m_views.size());

    for (int r = 0; r < m_grid_rows; ++r) {
        for (int c = 0; c < m_grid_cols; ++c) {
            if (m_cell_counts[r][c] >= std::max(1,target_samples)) {
                stats.sampled_cells++;
            }
        }
    }

    stats.spatial_coverage_ratio = (stats.total_cells > 0)
        ? static_cast<float>(stats.sampled_cells) / stats.total_cells
        : 0.0f;

    int right_cells = 0;
    for (const auto &row : m_right_cell_counts) for (int count : row) if (count >= std::max(1,target_samples)) ++right_cells;
    stats.right_spatial_coverage_ratio = static_cast<float>(right_cells) / stats.total_cells;

    return stats;
}

int NestedCoverageData::index(int x, int y, int depth, int pan, int tilt) const {
    if (x < 0 || x >= cols || y < 0 || y >= rows || depth < 0 || depth >= depth_bins ||
        pan < 0 || pan >= 3 || tilt < 0 || tilt >= 3) return -1;
    return (((y*cols+x)*depth_bins+depth)*3+tilt)*3+pan;
}

int NestedCoverageData::count(int x, int y, int depth, int pan, int tilt, bool raw_corners) const {
    const int i = index(x,y,depth,pan,tilt);
    const auto &values = raw_corners ? corners : samples;
    return i < 0 || static_cast<size_t>(i) >= values.size() ? 0 : values[i];
}

namespace {
std::vector<cv::Vec3d> corner_positions(const std::vector<cv::Point3f> &points,
                                       const CoveragePose &pose) {
    std::vector<cv::Vec3d> result;
    if (!pose.valid) return result;
    cv::Matx33d R; cv::Rodrigues(pose.rvec,R);
    for (const auto &p : points) result.push_back(R*cv::Vec3d(p.x,p.y,p.z)+pose.tvec);
    return result;
}

std::pair<int,int> pan_tilt_bins(const CoveragePose &pose) {
    cv::Matx33d R; cv::Rodrigues(pose.rvec,R);
    cv::Vec3d n(R(0,2),R(1,2),R(2,2));
    if (n[2] < 0) n = -n;
    auto bin = [](double angle) {
        angle *= 180/CV_PI;
        return angle < -NestedCoverageData::angle_threshold_deg ? 0 :
               angle > NestedCoverageData::angle_threshold_deg ? 2 : 1;
    };
    return {bin(std::atan2(n[0],n[2])), bin(std::atan2(n[1],n[2]))};
}

bool visible_corner(const cv::Point2f &p, const cv::Size &size) {
    return std::isfinite(p.x) && std::isfinite(p.y) && p.x >= 0 && p.y >= 0 &&
           p.x < size.width && p.y < size.height;
}

double missing_coverage(int count, int target) {
    return std::clamp(1.0-static_cast<double>(count)/std::max(1,target),0.0,1.0);
}

int band_count(const NestedCoverageData &data, int x, int y, int depth, bool raw = false) {
    int sum = 0;
    for (int tilt = 0; tilt < 3; ++tilt) for (int pan = 0; pan < 3; ++pan)
        sum += data.count(x,y,depth,pan,tilt,raw);
    return sum;
}

// Visit only the visible portions of a rectangular ring. Avoid scanning its hole.
template<class F>
void ring_spans(const cv::Rect &outer, const cv::Rect &inner, const cv::Size &size, F render) {
    const auto clipped = outer & cv::Rect(0,0,size.width,size.height);
    for (int y = clipped.y; y < clipped.br().y; ++y) {
        if (!inner.empty() && y >= inner.y && y < inner.br().y) {
            if (clipped.x < inner.x) render(y,clipped.x,std::min(clipped.br().x,inner.x));
            if (clipped.br().x > inner.br().x) render(y,std::max(clipped.x,inner.br().x),clipped.br().x);
        } else render(y,clipped.x,clipped.br().x);
    }
}

void draw_gradient_band(cv::Mat &image, const NestedCoverageData &data,
                        int x, int y, int depth, const cv::Rect &outer,
                        const cv::Rect &inner, double max_alpha, cv::Mat &opacity) {
    const double tl = missing_coverage(data.count(x,y,depth,0,0),data.target_samples);
    const double tr = missing_coverage(data.count(x,y,depth,2,0),data.target_samples);
    const double bl = missing_coverage(data.count(x,y,depth,0,2),data.target_samples);
    const double br = missing_coverage(data.count(x,y,depth,2,2),data.target_samples);
    const double frontal = missing_coverage(data.count(x,y,depth,1,1),data.target_samples);
    const double inv_width = 1.0/std::max(1,outer.width-1);
    const double inv_height = 1.0/std::max(1,outer.height-1);
    ring_spans(outer,inner,image.size(),[&](int py,int start,int end) {
        const double v = (py-outer.y)*inv_height;
        // A 3x3 interpolation surface: the four edge midpoints (and center)
        // share the frontal count, while each diagonal retains its angled bin.
        // Render two linear spans per row to keep the cached overlay inexpensive.
        const double vertical = std::abs(2*v-1);
        const double left = frontal+((v<0.5 ? tl : bl)-frontal)*vertical;
        const double right = frontal+((v<0.5 ? tr : br)-frontal)*vertical;
        const int middle = outer.x+(outer.width-1)/2+1;
        const double retained = 1-max_alpha;
        auto *alpha_row = opacity.ptr<float>(py);
        auto blend_span = [&](int first,int last,bool left_half) {
            if (first>=last) return;
            const double edge = left_half ? left : right;
            const double step = (left_half ? -2 : 2)*(edge-frontal)*inv_width;
            const double u = (first-outer.x)*inv_width;
            double missing = frontal+(edge-frontal)*(left_half ? 1-2*u : 2*u-1);
            auto *pixel = image.ptr<uchar>(py)+3*first;
            for (int px=first;px<last;++px,pixel+=3,missing+=step) {
                pixel[0] = static_cast<uchar>(pixel[0]*retained+0.5);
                pixel[1] = static_cast<uchar>(pixel[1]*retained+255*(1-missing)*max_alpha+0.5);
                pixel[2] = static_cast<uchar>(pixel[2]*retained+255*missing*max_alpha+0.5);
                alpha_row[px] = max_alpha;
            }
        };
        blend_span(start,std::min(end,middle),true);
        blend_span(std::max(start,middle),end,false);
    });
    // Deliberately no outlines or separators between depth bands.
}
}

NestedCoverageRange CoverageTracker::nested_depth_range() const {
    NestedCoverageRange range;
    double lo = std::numeric_limits<double>::infinity(), hi = -lo;
    for (const auto &v : m_views) for (int eye = 0; eye < 2; ++eye) {
        const auto &pixels = eye ? v.right_pixels : v.left_pixels;
        const CoveragePose pose = eye ? v.right_pose : CoveragePose{v.rvec,v.tvec,v.has_translation};
        const auto positions = corner_positions(v.object_points,pose);
        if (positions.size() != pixels.size()) continue;
        for (size_t i = 0; i < positions.size(); ++i) {
            const double z = positions[i][2];
            if (!visible_corner(pixels[i],m_last_image_size) || !std::isfinite(z) || z <= 0 || z >= 1e7) continue;
            lo = std::min(lo,z); hi = std::max(hi,z);
        }
    }
    if (!std::isfinite(lo)) return range;
    const double span = std::max(hi-lo, 4.0);
    const double step = std::pow(10.0,std::floor(std::log10(span)))/10.0;
    const double margin = std::max(0.5,span*0.025);
    range.near_mm = std::max(0.0,std::floor((lo-margin)/step)*step);
    range.far_mm = std::ceil((hi+margin)/step)*step;
    range.valid = true;
    return range;
}

NestedCoverageData CoverageTracker::nested_coverage(bool right_eye, int target_samples) const {
    NestedCoverageData data;
    data.cols = m_grid_cols; data.rows = m_grid_rows;
    data.target_samples = std::clamp(target_samples,1,20);
    data.image_size = m_last_image_size;
    data.range = nested_depth_range();
    data.samples.assign(data.cols*data.rows*data.depth_bins*9,0);
    data.corners.assign(data.samples.size(),0);
    for (const auto &v : m_views) {
        const CoveragePose pose = right_eye ? v.right_pose : CoveragePose{v.rvec,v.tvec,v.has_translation};
        const auto &pixels = right_eye ? v.right_pixels : v.left_pixels;
        const auto positions = corner_positions(v.object_points,pose);
        if (!pose.valid || positions.size() != pixels.size()) continue;
        const auto [pan,tilt] = pan_tilt_bins(pose);
        std::set<int> occupied;
        for (size_t i = 0; i < pixels.size(); ++i) {
            const double z = positions[i][2];
            if (!visible_corner(pixels[i],data.image_size) || !std::isfinite(z) || z <= 0 ||
                z < data.range.near_mm || z >= data.range.far_mm) continue;
            const int x = pixels[i].x*data.cols/data.image_size.width;
            const int y = pixels[i].y*data.rows/data.image_size.height;
            const int depth = (z-data.range.near_mm)*data.depth_bins/(data.range.far_mm-data.range.near_mm);
            const int index = data.index(x,y,depth,pan,tilt);
            if (index < 0) continue;
            ++data.corners[index]; occupied.insert(index);
        }
        for (int index : occupied) ++data.samples[index];
    }
    return data;
}

std::vector<int> CoverageTracker::corner_coverage_bins(const NestedCoverageData &data,
                                         const std::vector<cv::Point2f> &pixels,
                                         const std::vector<cv::Point3f> &objects,
                                         const CoveragePose &pose) {
    std::vector<int> bins(pixels.size(),-1);
    if (!data.range.valid || !pose.valid || data.image_size.empty()) return bins;
    const auto positions = corner_positions(objects,pose);
    if (positions.size() != pixels.size()) return bins;
    const auto [pan,tilt] = pan_tilt_bins(pose);
    for (size_t i=0; i<pixels.size(); ++i) {
        if (!visible_corner(pixels[i],data.image_size)) continue;
        const double z = positions[i][2];
        if (!std::isfinite(z) || z <= 0 || z < data.range.near_mm || z >= data.range.far_mm) continue;
        const int x = pixels[i].x*data.cols/data.image_size.width;
        const int y = pixels[i].y*data.rows/data.image_size.height;
        const int depth = (z-data.range.near_mm)*data.depth_bins/(data.range.far_mm-data.range.near_mm);
        bins[i] = data.index(x,y,depth,pan,tilt);
    }
    return bins;
}

int CoverageTracker::sample_density_gain(const NestedCoverageData &data,
                                         const std::vector<cv::Point2f> &pixels,
                                         const std::vector<cv::Point3f> &objects,
                                         const CoveragePose &pose) {
    std::set<int> useful;
    for (int index : corner_coverage_bins(data,pixels,objects,pose))
        if (index>=0 && data.samples[index]<data.target_samples) useful.insert(index);
    return useful.size();
}

bool CoverageTracker::is_useful_capture(const std::vector<cv::Point2f> &left,
                                        const std::vector<cv::Point2f> &right,
                                        const std::vector<int> &ids,
                                        const std::vector<cv::Point3f> &objects,
                                        const cv::Size &size, const CoveragePose &left_pose,
                                        const CoveragePose &right_pose, int target,
                                        std::string *reason) const {
    if (right.size()!=left.size()) {
        if (reason) *reason="Stereo correspondence counts do not match";
        return false;
    }
    if (is_novel_coverage(left,ids,objects,size,reason)) return true;
    CapturedView candidate;
    if (size != m_last_image_size ||
        !make_view(left,ids,objects,size,candidate)) return false;
    for (const auto &view : m_views) if (views_are_close(candidate,view,true)) {
        if (reason) *reason = "Too close to sample #"+std::to_string(view.sample_id)+" (move slightly before another capture)";
        return false;
    }
    if (sample_density_gain(nested_coverage(false,target),left,objects,left_pose)+
        sample_density_gain(nested_coverage(true,target),right,objects,right_pose) > 0) {
        if (reason) *reason = "Small pose change adds missing density";
        return true;
    }
    return false;
}

DensitySummary CoverageTracker::density_summary(const NestedCoverageData &left,
                                                  const NestedCoverageData &right) {
    DensitySummary result;
    auto credits = [](const NestedCoverageData &data) {
        double credited = 0;
        for (int y = 0; y < data.rows; ++y) for (int x = 0; x < data.cols; ++x)
            for (int z = 0; z < data.depth_bins; ++z) {
                for (int tilt : {0,2}) for (int pan : {0,2})
                    credited += std::min(std::max(0,data.count(x,y,z,pan,tilt)),std::max(1,data.target_samples));
                // Frontal density appears at four edge midpoints, but is one bin.
                credited += std::min(std::max(0,data.count(x,y,z,1,1)),std::max(1,data.target_samples));
            }
        return credited;
    };
    auto goal = [](const NestedCoverageData &data) {
        return static_cast<double>(data.cols)*data.rows*data.depth_bins*5*std::max(1,data.target_samples);
    };
    const double l_goal = goal(left), r_goal = goal(right);
    const double l_credit = credits(left), r_credit = credits(right);
    result.left_fraction = l_goal > 0 ? l_credit/l_goal : 0;
    result.right_fraction = r_goal > 0 ? r_credit/r_goal : 0;
    return result;
}

NestedCoverageChart CoverageTracker::draw_nested_overlay(const cv::Mat &image,
                                                         const NestedCoverageData &data,
                                                         bool right_eye, double alpha) {
    NestedCoverageChart chart;
    if (image.empty() || data.cols <= 0 || data.rows <= 0) return chart;
    if (image.channels() == 1) cv::cvtColor(image,chart.image,cv::COLOR_GRAY2BGR);
    else chart.image = image.clone();
    chart.opacity = cv::Mat::zeros(image.size(),CV_32FC1);
    for (int y = 0; y < data.rows; ++y) for (int x = 0; x < data.cols; ++x) {
        const int x0 = x*image.cols/data.cols, x1 = (x+1)*image.cols/data.cols;
        const int y0 = y*image.rows/data.rows, y1 = (y+1)*image.rows/data.rows;
        const cv::Rect bucket(x0,y0,x1-x0,y1-y0);
        const int inset = std::max(2,(std::min(bucket.width,bucket.height)-4)/(2*data.depth_bins));
        for (int depth = data.depth_bins-1; depth >= 0; --depth) {
            const int offset = 2+(data.depth_bins-1-depth)*inset;
            const cv::Rect band(x0+offset,y0+offset,bucket.width-2*offset,bucket.height-2*offset);
            if (band.width < 2 || band.height < 2) continue;
            cv::Rect inner;
            if (depth > 0 && band.width > 2*inset && band.height > 2*inset)
                inner = {band.x+inset,band.y+inset,band.width-2*inset,band.height-2*inset};
            draw_gradient_band(chart.image,data,x,y,depth,band,inner,std::clamp(alpha,0.0,1.0),chart.opacity);
            chart.tiles.push_back({band,x,y,depth,right_eye,inner});
        }
    }
    return chart;
}

void CoverageTracker::draw_live_coverage_tint(cv::Mat &image, const NestedCoverageData &data,
                                              const NestedCoverageChart &layout,
                                              const std::vector<cv::Point2f> &corners,
                                              const std::vector<cv::Point3f> &object_points,
                                              const CoveragePose &pose) {
    if (image.empty() || corners.empty() || data.cols <= 0 || data.rows <= 0) return;
    const auto positions = corner_positions(object_points,pose);
    const bool has_depth = pose.valid && positions.size() == corners.size() && data.range.valid;
    std::set<std::tuple<int,int,int>> occupied;
    for (size_t i = 0; i < corners.size(); ++i) {
        if (!visible_corner(corners[i],image.size())) continue;
        const int x = corners[i].x*data.cols/image.cols, y = corners[i].y*data.rows/image.rows;
        int depth = -1; // No retained range/pose: indicate only the observed X/Y footprint.
        if (has_depth) {
            const double z = positions[i][2];
            if (!std::isfinite(z) || z <= 0) continue;
            depth = std::clamp(static_cast<int>(std::floor((z-data.range.near_mm)*data.depth_bins/
                                      (data.range.far_mm-data.range.near_mm))),0,data.depth_bins-1);
        }
        occupied.emplace(x,y,depth);
    }
    double u = 0.5, v = 0.5;
    if (pose.valid) {
        cv::Matx33d R; cv::Rodrigues(pose.rvec,R);
        cv::Vec3d n(R(0,2),R(1,2),R(2,2));
        if (n[2] < 0) n = -n;
        u = std::clamp(0.5+std::atan2(n[0],n[2])/(CV_PI/2),0.0,1.0);
        v = std::clamp(0.5+std::atan2(n[1],n[2])/(CV_PI/2),0.0,1.0);
    }
    const std::array<double,4> weights{{(1-u)*(1-v),u*(1-v),(1-u)*v,u*v}};
    const double peak = *std::max_element(weights.begin(),weights.end());
    for (const auto &tile : layout.tiles) {
        if (!occupied.count({tile.x,tile.y,tile.depth}) && !occupied.count({tile.x,tile.y,-1})) continue;
        // Keep both edges of the density band visible. The central, filled
        // depth bucket is treated as a ring around its center for the live tint.
        const auto &outer=tile.bounds;
        const auto &inner=tile.inner_bounds;
        const int thickness=inner.empty() ? std::min(outer.width,outer.height)/2 :
            std::min({inner.x-outer.x,inner.y-outer.y,outer.br().x-inner.br().x,outer.br().y-inner.br().y});
        if (thickness<3) continue; // At tiny image sizes prioritize density visibility.
        const int outer_inset=(thickness+2)/3;
        const int inner_inset=(2*thickness+2)/3;
        const cv::Rect blue_outer(outer.x+outer_inset,outer.y+outer_inset,
                                  outer.width-2*outer_inset,outer.height-2*outer_inset);
        const cv::Rect blue_inner(outer.x+inner_inset,outer.y+inner_inset,
                                  outer.width-2*inner_inset,outer.height-2*inner_inset);
        const double inv_width = 1.0/std::max(1,tile.bounds.width-1);
        const double inv_height = 1.0/std::max(1,tile.bounds.height-1);
        ring_spans(blue_outer,blue_inner,image.size(),[&](int py,int start,int end) {
            const double ty = (py-tile.bounds.y)*inv_height;
            const double left = weights[0]*(1-ty)+weights[2]*ty;
            const double right = weights[1]*(1-ty)+weights[3]*ty;
            const double step = 0.4*(right-left)*inv_width/peak;
            double a = 0.4*left/peak+(start-tile.bounds.x)*step;
            auto *pixel = image.ptr<uchar>(py)+3*start;
            for (int px = start; px < end; ++px,pixel+=3,a+=step) {
                const double retained = 1-a;
                pixel[0] = static_cast<uchar>(pixel[0]*retained+255*a+0.5);
                pixel[1] = static_cast<uchar>(pixel[1]*retained+0.5);
                pixel[2] = static_cast<uchar>(pixel[2]*retained+0.5);
            }
        });
    }
}

std::string CoverageTracker::describe_nested_tile(const NestedCoverageData &data,
                                                  const NestedCoverageTile &tile) {
    const double step = (data.range.far_mm-data.range.near_mm)/data.depth_bins;
    std::ostringstream ss;
    ss << "X bucket " << tile.x+1 << ", Y bucket " << tile.y+1;
    if (data.image_size.width > 0) ss << " (pixels X " << tile.x*data.image_size.width/data.cols << "-"
        << (tile.x+1)*data.image_size.width/data.cols << ", Y " << tile.y*data.image_size.height/data.rows
        << "-" << (tile.y+1)*data.image_size.height/data.rows << ")";
    if (data.range.valid) ss << "\nDepth [" << std::fixed << std::setprecision(1) << data.range.near_mm+tile.depth*step
        << ", " << data.range.near_mm+(tile.depth+1)*step << ") mm";
    else ss << "\nDepth unavailable";
    ss << "\n" << band_count(data,tile.x,tile.y,tile.depth) << " retained images, "
       << band_count(data,tile.x,tile.y,tile.depth,true) << " detected corners in X/Y/Z";
    ss << "\nGradient corner counts (both angles beyond +/-15 deg):";
    ss << "\nLeft/up " << data.count(tile.x,tile.y,tile.depth,0,0)
       << "; right/up " << data.count(tile.x,tile.y,tile.depth,2,0)
       << "; left/down " << data.count(tile.x,tile.y,tile.depth,0,2)
       << "; right/down " << data.count(tile.x,tile.y,tile.depth,2,2);
    ss << "\nN/S/E/W frontal (both within +/-15 deg): " << data.count(tile.x,tile.y,tile.depth,1,1);
    ss << "\nRed to green at " << data.target_samples << " retained images per orientation bin; constant opacity. Frontal density is repeated at all four edge midpoints.";
    return ss.str();
}

} // namespace dvrk_data::calibration
