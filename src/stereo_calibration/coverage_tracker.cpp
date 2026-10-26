// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/coverage_tracker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace dvrk_data::calibration {

CoverageTracker::CoverageTracker(int grid_cols, int grid_rows)
    : m_grid_cols(std::max(2, grid_cols)),
      m_grid_rows(std::max(2, grid_rows)) {
    reset();
}

void CoverageTracker::reset() {
    m_cell_counts.assign(m_grid_rows, std::vector<int>(m_grid_cols, 0));
    for (int &b : m_tilt_bins) {
        b = 0;
    }
    m_views.clear();
    m_last_image_size = cv::Size();
    m_total_frames = 0;
}

static int get_tilt_bin(const cv::Vec3d &rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    double nx = R.at<double>(0, 2);
    double ny = R.at<double>(1, 2);
    double nz = R.at<double>(2, 2);

    double angle_deg = std::acos(std::clamp(std::abs(nz), 0.0, 1.0)) * 180.0 / M_PI;
    if (angle_deg < 10.0) {
        return 0; // Frontal
    }

    if (std::abs(nx) > std::abs(ny)) {
        return (nx > 0) ? 1 : 2; // Right vs Left tilt
    } else {
        return (ny > 0) ? 3 : 4; // Down vs Up tilt
    }
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
                         const std::vector<cv::Point2f> &b) {
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
    const double threshold = std::max(0.005f, 0.04f * std::max(footprint(a), footprint(b)));
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

bool CoverageTracker::views_are_close(const CapturedView &a, const CapturedView &b) {
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
        return similar_projections(shared_a, shared_b);
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
    return similar_projections(projected_a, projected_b);
}

void CoverageTracker::add_frame(const std::vector<cv::Point2f> &corners,
                               const std::vector<int> &corner_ids,
                               const std::vector<cv::Point3f> &object_points,
                               const cv::Size &image_size,
                               const cv::Vec3d &rvec,
                               bool has_pose) {
    CapturedView view;
    if (!make_view(corners, corner_ids, object_points, image_size, view)) return;
    m_views.push_back(std::move(view));

    m_last_image_size = image_size;
    float cell_w = static_cast<float>(image_size.width) / m_grid_cols;
    float cell_h = static_cast<float>(image_size.height) / m_grid_rows;

    for (const auto &pt : corners) {
        int c = std::clamp(static_cast<int>(pt.x / cell_w), 0, m_grid_cols - 1);
        int r = std::clamp(static_cast<int>(pt.y / cell_h), 0, m_grid_rows - 1);
        m_cell_counts[r][c]++;
    }

    if (has_pose) {
        int bin = get_tilt_bin(rvec);
        if (bin >= 0 && bin < 5) {
            m_tilt_bins[bin]++;
        }
    }

    m_total_frames++;
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
            if (reason_out) *reason_out = "Similar to an existing sample";
            return false;
        }
    }
    if (reason_out) *reason_out = m_views.empty() ? "First frame" : "New board view";
    return true;
}

CoverageStats CoverageTracker::get_stats() const {
    CoverageStats stats;
    stats.total_cells = m_grid_cols * m_grid_rows;
    stats.sampled_cells = 0;
    stats.frame_count = m_total_frames;

    for (int r = 0; r < m_grid_rows; ++r) {
        for (int c = 0; c < m_grid_cols; ++c) {
            if (m_cell_counts[r][c] >= 2) {
                stats.sampled_cells++;
            }
        }
    }

    stats.spatial_coverage_ratio = (stats.total_cells > 0)
        ? static_cast<float>(stats.sampled_cells) / stats.total_cells
        : 0.0f;

    // Real tilt diversity across 5 orientation bins (Frontal, Left, Right, Down, Up)
    int sampled_tilt_bins = 0;
    for (int b : m_tilt_bins) {
        if (b > 0) sampled_tilt_bins++;
    }

    stats.tilt_coverage_ratio = static_cast<float>(sampled_tilt_bins) / 5.0f;

    return stats;
}

void CoverageTracker::draw_heatmap_overlay(cv::Mat &image, double alpha) const {
    if (image.empty() || m_grid_cols <= 0 || m_grid_rows <= 0) return;

    cv::Mat overlay = image.clone();
    float cell_w = static_cast<float>(image.cols) / m_grid_cols;
    float cell_h = static_cast<float>(image.rows) / m_grid_rows;

    for (int r = 0; r < m_grid_rows; ++r) {
        for (int c = 0; c < m_grid_cols; ++c) {
            int count = m_cell_counts[r][c];
            cv::Scalar color;

            if (count == 0) {
                color = cv::Scalar(40, 40, 40); // Dark Gray (unvisited)
            } else if (count < 3) {
                color = cv::Scalar(0, 165, 255); // Orange (low samples)
            } else if (count < 8) {
                color = cv::Scalar(0, 220, 0);   // Green (good samples)
            } else {
                color = cv::Scalar(220, 200, 0); // Cyan/Yellow (saturated)
            }

            cv::Rect rect(static_cast<int>(c * cell_w),
                          static_cast<int>(r * cell_h),
                          static_cast<int>(cell_w),
                          static_cast<int>(cell_h));

            cv::rectangle(overlay, rect, color, cv::FILLED);
            cv::rectangle(overlay, rect, cv::Scalar(80, 80, 80), 1);
        }
    }

    cv::addWeighted(overlay, alpha, image, 1.0 - alpha, 0, image);
}

} // namespace dvrk_data::calibration
