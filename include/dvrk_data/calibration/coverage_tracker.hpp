// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#ifndef DVRK_DATA_CALIBRATION_COVERAGE_TRACKER_HPP
#define DVRK_DATA_CALIBRATION_COVERAGE_TRACKER_HPP

#include <vector>
#include <string>
#include <opencv2/core.hpp>

namespace dvrk_data::calibration {

struct CoverageStats {
    float spatial_coverage_ratio = 0.0f; // 0.0 to 1.0 (fraction of grid cells with samples)
    float tilt_coverage_ratio = 0.0f;    // 0.0 to 1.0 (diversity of board tilt angles or orientations)
    int total_cells = 0;
    int sampled_cells = 0;
    int frame_count = 0;
};

class CoverageTracker {
public:
    CoverageTracker(int grid_cols = 6, int grid_rows = 4);
    ~CoverageTracker() = default;

    /// Reset all coverage history
    void reset();

    /// Add a captured view, preserving corner identities for duplicate checks.
    void add_frame(const std::vector<cv::Point2f> &corners,
                   const std::vector<int> &corner_ids,
                   const std::vector<cv::Point3f> &object_points,
                   const cv::Size &image_size,
                   const cv::Vec3d &rvec = cv::Vec3d(0, 0, 0),
                   bool has_pose = false);

    /// Reject views close to any retained sample. Pose estimates and heatmap
    /// counts do not establish novelty; corner identities distinguish motion
    /// from changes in which part of the board was detected.
    bool is_novel_coverage(const std::vector<cv::Point2f> &corners,
                           const std::vector<int> &corner_ids,
                           const std::vector<cv::Point3f> &object_points,
                           const cv::Size &image_size,
                           std::string *reason_out = nullptr) const;

    /// Get current coverage statistics
    CoverageStats get_stats() const;

    /// Render a semi-transparent coverage heatmap overlay onto an image
    void draw_heatmap_overlay(cv::Mat &image, double alpha = 0.35) const;

private:
    int m_grid_cols;
    int m_grid_rows;
    cv::Size m_last_image_size{0, 0};

    // Cell hit count matrix [row][col]
    std::vector<std::vector<int>> m_cell_counts;

    // Tilt angle histogram bins
    int m_tilt_bins[5] = {0, 0, 0, 0, 0};

    struct CapturedView {
        std::vector<int> ids;
        std::vector<cv::Point2f> corners; // normalized by the longest image side
        std::vector<cv::Point2f> board_points;
        cv::Mat board_to_image;
    };

    static bool make_view(const std::vector<cv::Point2f> &corners,
                          const std::vector<int> &corner_ids,
                          const std::vector<cv::Point3f> &object_points,
                          const cv::Size &image_size,
                          CapturedView &view);
    static bool views_are_close(const CapturedView &a, const CapturedView &b);
    std::vector<CapturedView> m_views;

    int m_total_frames = 0;
};

} // namespace dvrk_data::calibration

#endif // DVRK_DATA_CALIBRATION_COVERAGE_TRACKER_HPP
