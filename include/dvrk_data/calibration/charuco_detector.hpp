// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#ifndef DVRK_DATA_CALIBRATION_CHARUCO_DETECTOR_HPP
#define DVRK_DATA_CALIBRATION_CHARUCO_DETECTOR_HPP

#include <vector>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

namespace dvrk_data::calibration {

struct CharucoConfig {
    int squares_x = 8;
    int squares_y = 6;
    float square_length_mm = 5.0f;
    float marker_length_mm = 3.5f;
    int dictionary_id = cv::aruco::DICT_4X4_50;
    int min_corners = 6;               // Minimum corner count (must be >= 6 for DLT pose estimation and calibration)
    float motion_threshold_px = 4.5f;  // Pixel movement threshold accommodating handheld endoscope tremor
    bool refine_subpix = true;
};

struct MonoDetection {
    bool valid = false;
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> charuco_ids;
    std::vector<cv::Point2f> charuco_corners;
    cv::Vec3d rvec{0, 0, 0};
    cv::Vec3d tvec{0, 0, 0};
    bool pose_valid = false;
};

struct StereoDetection {
    MonoDetection left;
    MonoDetection right;
    std::vector<int> common_ids;
    std::vector<cv::Point2f> left_common_corners;
    std::vector<cv::Point2f> right_common_corners;
    std::vector<cv::Point3f> object_points;
    bool is_stereo_valid = false;
    float motion_speed = 0.0f;     // average pixel movement since last frame
    bool is_stationary = false;    // motion_speed < motion_threshold_px
};

class CharucoDetector {
public:
    explicit CharucoDetector(const CharucoConfig &config = CharucoConfig());
    ~CharucoDetector() = default;

    /// Update configuration and recreate board/dictionary
    void update_config(const CharucoConfig &config);
    const CharucoConfig &get_config() const { return m_config; }

    /// Detect markers and interpolate ChArUco corners for a single image
    bool detect_mono(const cv::Mat &image,
                     MonoDetection &out,
                     const cv::Mat &camera_matrix = cv::Mat(),
                     const cv::Mat &dist_coeffs = cv::Mat());

    /// Detect in stereo pair, match common corners, and compute motion/stationarity
    bool detect_stereo(const cv::Mat &left_img,
                       const cv::Mat &right_img,
                       StereoDetection &out,
                       const cv::Mat &k_l = cv::Mat(),
                       const cv::Mat &d_l = cv::Mat(),
                       const cv::Mat &k_r = cv::Mat(),
                       const cv::Mat &d_r = cv::Mat());

    /// Annotate single image with detected markers, charuco corners, and optional coordinate frame
    void draw_mono_detection(cv::Mat &image,
                             const MonoDetection &detection,
                             bool draw_axes = false,
                             const cv::Mat &camera_matrix = cv::Mat(),
                             const cv::Mat &dist_coeffs = cv::Mat());

    /// Annotate both left and right images with common corners and status
    void draw_stereo_detection(cv::Mat &left_img,
                               cv::Mat &right_img,
                               const StereoDetection &detection);

    /// Reset motion history (e.g. after capture or pause)
    void reset_motion_tracking();

    /// Get OpenCV CharucoBoard pointer
    cv::Ptr<cv::aruco::CharucoBoard> get_board() const { return m_board; }

    /// Get OpenCV Dictionary pointer
    cv::Ptr<cv::aruco::Dictionary> get_dictionary() const { return m_dict; }

    /// Lookup 3D object point for a specific ChArUco corner ID (in board frame, mm or meters)
    cv::Point3f get_corner_3d(int corner_id) const;

private:
    void init_board();

    CharucoConfig m_config;
    cv::Ptr<cv::aruco::Dictionary> m_dict;
    cv::Ptr<cv::aruco::CharucoBoard> m_board;
    cv::Ptr<cv::aruco::DetectorParameters> m_detector_params;

    // Previous frame common corner locations for velocity tracking
    std::map<int, cv::Point2f> m_prev_left_corners;
    bool m_has_prev_corners = false;
    float m_smoothed_speed = 0.0f;
};

} // namespace dvrk_data::calibration

#endif // DVRK_DATA_CALIBRATION_CHARUCO_DETECTOR_HPP
