// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#ifndef DVRK_DATA_CALIBRATION_STEREO_CALIBRATOR_HPP
#define DVRK_DATA_CALIBRATION_STEREO_CALIBRATOR_HPP

#include "dvrk_data/calibration/charuco_detector.hpp"

#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <limits>
#include <opencv2/core.hpp>

namespace dvrk_data::calibration {

enum class DistortionModel {
    PLUMB_BOB,          // 5-parameter model (k1, k2, p1, p2, k3) - ROS standard
    RATIONAL_POLYNOMIAL // 8-parameter model (k1, k2, p1, p2, k3, k4, k5, k6)
};

struct CalibrationFrame {
    int id = 0;
    cv::Mat left_image;
    cv::Mat right_image;
    std::vector<int> charuco_ids;
    std::vector<cv::Point2f> left_corners;
    std::vector<cv::Point2f> right_corners;
    std::vector<cv::Point3f> object_points;
    cv::Vec3d rvec{0, 0, 0};
    bool pose_valid = false;
    double epipolar_rms_px = std::numeric_limits<double>::quiet_NaN();
};

struct CalibratorOptions {
    DistortionModel model = DistortionModel::PLUMB_BOB;
    bool fix_intrinsics = true;       // Fix monocular intrinsics during stereo baseline optimization (fast & robust)
    bool fix_aspect_ratio = false;
    bool zero_tangent_dist = false;
    double alpha_rectification = 0.2; // 0.0: maximize zoom / no black border, 1.0: keep all pixels
};

struct StereoCalibrationResult {
    bool success = false;
    // UTC ISO 8601 time of successful calibration; required for YAML export.
    std::string calibration_date;
    cv::Size image_size{0, 0};
    DistortionModel model = DistortionModel::PLUMB_BOB;

    // Intrinsics
    cv::Mat K1; // 3x3 Camera matrix Left
    cv::Mat D1; // Distortion coefficients Left
    cv::Mat K2; // 3x3 Camera matrix Right
    cv::Mat D2; // Distortion coefficients Right

    // Extrinsics
    cv::Mat R; // 3x3 Rotation matrix from Left to Right
    cv::Mat T; // 3x1 Translation vector (mm)
    cv::Mat E; // Essential matrix
    cv::Mat F; // Fundamental matrix

    // Rectification
    cv::Mat R1, R2; // 3x3 Rectification transforms
    cv::Mat P1, P2; // 3x4 Projection matrices in new rectified coords
    cv::Mat Q;      // 4x4 Disparity-to-depth mapping matrix
    cv::Rect roi1, roi2;

    // Fast undistort & rectify lookup maps
    cv::Mat map1_l, map2_l;
    cv::Mat map1_r, map2_r;

    // Quality metrics
    double rms_error = 0.0;
    double baseline_mm = 0.0;
    double avg_epipolar_error_px = 0.0;
    double max_epipolar_error_px = 0.0;
    std::vector<double> per_frame_epipolar_rms_px;
    std::map<int, double> epipolar_rms_by_id;
};

// A missing metric is NaN with a reason; it must never appear as a perfect fit.
struct CornerResidual {
    int id = -1;
    cv::Point2f detected, projected, delta; // projected - detected, pixels
    double magnitude_px = 0.0;
};

struct ProjectionDiagnostics {
    bool valid = false;
    std::string reason;
    double rms_px = std::numeric_limits<double>::quiet_NaN();
    cv::Vec3d rvec{0, 0, 0}, tvec{0, 0, 0};
    std::vector<CornerResidual> corners;
};

struct SampleDiagnostics {
    ProjectionDiagnostics left_fit, right_fit, left_to_right, right_to_left;
    double epipolar_rms_px = std::numeric_limits<double>::quiet_NaN();
    std::string epipolar_reason;
    std::map<int,double> epipolar_by_corner_id;
};

struct OutlierFilterReport {
    int removed_samples = 0, removed_corners = 0;
    int protected_samples = 0, protected_corners = 0, review_samples = 0;
    bool widespread_disagreement = false;
    std::vector<std::string> decisions;
    std::string summary() const;
};

enum class ProjectionMode { LEFT_TO_RIGHT, INDEPENDENT, RIGHT_TO_LEFT };

struct GridReprojection {
    cv::Mat left_overlay, right_overlay;
    double left_rms_px = std::numeric_limits<double>::quiet_NaN();
    double right_rms_px = std::numeric_limits<double>::quiet_NaN();
};

class StereoCalibrator {
public:
    StereoCalibrator() = default;
    ~StereoCalibrator() = default;

    /// Add a frame pair with detected stereo corners to collection
    int add_frame(const cv::Mat &left_img,
                  const cv::Mat &right_img,
                  const StereoDetection &detection);

    /// Restore a frame with its archived ID; 0 assigns a new ID.
    int add_frame(const cv::Mat &left_img,
                  const cv::Mat &right_img,
                  const StereoDetection &detection, int frame_id);

    /// Remove a frame by its unique ID
    bool remove_frame(int frame_id);

    /// Clear all collected frames
    void clear_frames();

    /// Get total number of collected frames
    size_t frame_count() const { return m_frames.size(); }

    /// Access collected frames
    const std::vector<CalibrationFrame> &get_frames() const { return m_frames; }

    /// Return expected image size of current collection, or (0, 0) if empty
    cv::Size expected_image_size() const {
        return m_frames.empty() ? cv::Size(0, 0) : m_frames[0].left_image.size();
    }

    /// Progress callback reporting completion fraction (0.0 .. 1.0) and description of current stage
    using ProgressCallback = std::function<void(double fraction, const std::string &stage)>;

    /// Run calibration computation
    bool calibrate(const CalibratorOptions &opts,
                   StereoCalibrationResult &result,
                   std::string &error_msg,
                   ProgressCallback progress_cb = nullptr);

    /// Compute all metrics without rendering; safe to call after any sample/model change.
    static SampleDiagnostics diagnose_sample(const CalibrationFrame &frame,
                                             const StereoCalibrationResult &calib, bool robust_pose = false);

    /// One conservative pass with frozen coverage bins and paired corner removal.
    OutlierFilterReport filter_outliers(const StereoCalibrationResult &calib, int target_density = 2);

    /// Render previously computed diagnostics, with one common residual color scale.
    static bool render_grid(const CalibrationFrame &frame, const CharucoConfig &board,
                            const StereoCalibrationResult &calib,
                            const SampleDiagnostics &diagnostics, ProjectionMode mode,
                            double color_max_px, int selected_corner_id,
                            GridReprojection &output, std::string &error_msg);

    /// Fit board pose in the left eye and project the grid into both raw images.
    /// The right pose uses calibrated R/T, so its residual also checks stereo geometry.
    static bool reproject_grid(const CalibrationFrame &frame,
                               const CharucoConfig &board,
                               const StereoCalibrationResult &calib,
                               GridReprojection &output,
                               std::string &error_msg);

    /// Rectify a live stereo image pair using precomputed maps
    static void rectify_stereo(const cv::Mat &in_left,
                               const cv::Mat &in_right,
                               cv::Mat &out_left,
                               cv::Mat &out_right,
                               const StereoCalibrationResult &calib);

    /// Create red-cyan 3D anaglyph visualization from rectified stereo images
    static cv::Mat make_anaglyph(const cv::Mat &rect_left, const cv::Mat &rect_right);

    /// Create side-by-side visualization with horizontal epipolar alignment lines
    static cv::Mat make_side_by_side_epipolar(const cv::Mat &rect_left,
                                              const cv::Mat &rect_right,
                                              int line_spacing = 32);

private:
    std::vector<CalibrationFrame> m_frames;
    int m_next_frame_id = 1;
};

} // namespace dvrk_data::calibration

#endif // DVRK_DATA_CALIBRATION_STEREO_CALIBRATOR_HPP
