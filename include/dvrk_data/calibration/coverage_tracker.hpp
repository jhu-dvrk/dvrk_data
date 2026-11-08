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
    int total_cells = 0;
    int sampled_cells = 0;
    int frame_count = 0;
    float right_spatial_coverage_ratio = 0.0f;
};

struct CoveragePose {
    cv::Vec3d rvec{0, 0, 0}, tvec{0, 0, 0};
    bool valid = false;
};

struct NestedCoverageRange {
    bool valid = false;
    double near_mm = 0.0, far_mm = 200.0;
};

struct NestedCoverageData {
    int cols = 0, rows = 0;
    static constexpr int depth_bins = 4;
    static constexpr double overlay_opacity = 0.30;
    static constexpr int default_target_samples = 2;
    int target_samples = default_target_samples;
    static constexpr double angle_threshold_deg = 15.0;
    NestedCoverageRange range;
    cv::Size image_size;
    std::vector<int> samples, corners;
    int count(int x, int y, int depth, int pan, int tilt, bool raw_corners = false) const;
    int index(int x, int y, int depth, int pan, int tilt) const;
};

struct DensitySummary {
    double left_fraction = 0.0, right_fraction = 0.0;
};

struct NestedCoverageTile {
    cv::Rect bounds;
    int x, y, depth;
    bool right_eye = false;
    cv::Rect inner_bounds; // Hole belonging to nearer depth bands.
};

struct NestedCoverageChart {
    cv::Mat image;
    cv::Mat opacity; // CV_32FC1; coverage alpha for each pixel.
    std::vector<NestedCoverageTile> tiles;
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
                   bool has_pose = false, int sample_id = 0,
                   const std::vector<cv::Point2f> &right_corners = {},
                   const cv::Vec3d &tvec = cv::Vec3d(0, 0, 0),
                   bool has_translation = false,
                   const CoveragePose &right_pose = CoveragePose());

    /// Reject views close to any retained sample. Pose estimates and heatmap
    /// counts do not establish novelty; corner identities distinguish motion
    /// from changes in which part of the board was detected.
    bool is_novel_coverage(const std::vector<cv::Point2f> &corners,
                           const std::vector<int> &corner_ids,
                           const std::vector<cv::Point3f> &object_points,
                           const cv::Size &image_size,
                           std::string *reason_out = nullptr) const;

    /// Accept smaller, real pose changes only while they fill missing density.
    /// The tighter image-space noise floor still rejects stationary duplicates.
    bool is_useful_capture(const std::vector<cv::Point2f> &left,
                            const std::vector<cv::Point2f> &right,
                            const std::vector<int> &ids,
                            const std::vector<cv::Point3f> &objects,
                            const cv::Size &size, const CoveragePose &left_pose,
                            const CoveragePose &right_pose, int target,
                            std::string *reason = nullptr) const;
    /// Frozen-bin per-corner indices, or -1 for invisible/unusable corners.
    static std::vector<int> corner_coverage_bins(const NestedCoverageData &data,
                                    const std::vector<cv::Point2f> &pixels,
                                    const std::vector<cv::Point3f> &objects,
                                    const CoveragePose &pose);
    static int sample_density_gain(const NestedCoverageData &data,
                                    const std::vector<cv::Point2f> &pixels,
                                    const std::vector<cv::Point3f> &objects,
                                    const CoveragePose &pose);

    /// Get current coverage statistics
    CoverageStats get_stats(int target_samples = NestedCoverageData::default_target_samples) const;

    /// Both eyes share an auto depth range, derived only from retained corners.
    NestedCoverageRange nested_depth_range() const;
    NestedCoverageData nested_coverage(bool right_eye = false,
                                        int target_samples = NestedCoverageData::default_target_samples) const;
    /// Full-map heuristic: capped credits for one frontal and four tilted bins
    /// per X/Y/Z cell (frontal counted once) / goal.
    static DensitySummary density_summary(const NestedCoverageData &left,
                                          const NestedCoverageData &right);
    static NestedCoverageChart draw_nested_overlay(const cv::Mat &image,
                                                   const NestedCoverageData &data,
                                                   bool right_eye = false, double alpha = NestedCoverageData::overlay_opacity);
    /// Transient live footprint; does not modify retained coverage or its depth range.
    static void draw_live_coverage_tint(cv::Mat &image, const NestedCoverageData &data,
                                        const NestedCoverageChart &layout,
                                        const std::vector<cv::Point2f> &corners,
                                        const std::vector<cv::Point3f> &object_points,
                                        const CoveragePose &pose);
    static std::string describe_nested_tile(const NestedCoverageData &data,
                                            const NestedCoverageTile &tile);

private:
    int m_grid_cols;
    int m_grid_rows;
    cv::Size m_last_image_size{0, 0};

    // Cell hit count matrix [row][col]
    std::vector<std::vector<int>> m_cell_counts;

    std::vector<std::vector<int>> m_right_cell_counts;

    struct CapturedView {
        int sample_id = 0;
        cv::Vec3d rvec{0, 0, 0}, tvec{0, 0, 0};
        bool has_translation = false;
        CoveragePose right_pose;
        std::vector<cv::Point2f> left_pixels, right_pixels;
        std::vector<cv::Point3f> object_points;
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
    static bool views_are_close(const CapturedView &a, const CapturedView &b, bool noise_floor = false);
    std::vector<CapturedView> m_views;
};

} // namespace dvrk_data::calibration

#endif // DVRK_DATA_CALIBRATION_COVERAGE_TRACKER_HPP
