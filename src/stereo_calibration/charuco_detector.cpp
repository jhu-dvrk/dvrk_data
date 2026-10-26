// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/charuco_detector.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <iostream>

namespace dvrk_data::calibration {

CharucoDetector::CharucoDetector(const CharucoConfig &config)
    : m_config(config) {
    init_board();
}

void CharucoDetector::init_board() {
    m_dict = cv::aruco::getPredefinedDictionary(
        static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(m_config.dictionary_id));

    m_board = cv::aruco::CharucoBoard::create(
        m_config.squares_x,
        m_config.squares_y,
        m_config.square_length_mm,
        m_config.marker_length_mm,
        m_dict);

    m_detector_params = cv::aruco::DetectorParameters::create();
    if (m_config.refine_subpix) {
        m_detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        m_detector_params->cornerRefinementWinSize = 5;
        m_detector_params->cornerRefinementMaxIterations = 30;
        m_detector_params->cornerRefinementMinAccuracy = 0.05;
    }

    reset_motion_tracking();
}

void CharucoDetector::update_config(const CharucoConfig &config) {
    m_config = config;
    init_board();
}

void CharucoDetector::reset_motion_tracking() {
    m_prev_left_corners.clear();
    m_has_prev_corners = false;
    m_smoothed_speed = 0.0f;
}

cv::Point3f CharucoDetector::get_corner_3d(int corner_id) const {
    if (m_board && corner_id >= 0 && corner_id < static_cast<int>(m_board->chessboardCorners.size())) {
        return m_board->chessboardCorners[corner_id];
    }
    return cv::Point3f(0.0f, 0.0f, 0.0f);
}

bool CharucoDetector::detect_mono(const cv::Mat &image,
                                 MonoDetection &out,
                                 const cv::Mat &camera_matrix,
                                 const cv::Mat &dist_coeffs) {
    out.valid = false;
    out.pose_valid = false;
    out.marker_ids.clear();
    out.marker_corners.clear();
    out.charuco_ids.clear();
    out.charuco_corners.clear();

    if (image.empty()) return false;

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = image;
    }

    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(gray, m_dict, out.marker_corners, out.marker_ids, m_detector_params, rejected);

    if (out.marker_ids.empty()) {
        return false;
    }

    // Refine rejected markers using board layout
    cv::aruco::refineDetectedMarkers(gray, m_board, out.marker_corners, out.marker_ids, rejected);

    // Interpolate ChArUco corners
    int n_corners = 0;
    if (!camera_matrix.empty() && !dist_coeffs.empty()) {
        n_corners = cv::aruco::interpolateCornersCharuco(
            out.marker_corners, out.marker_ids, gray, m_board,
            out.charuco_corners, out.charuco_ids, camera_matrix, dist_coeffs);
    } else {
        n_corners = cv::aruco::interpolateCornersCharuco(
            out.marker_corners, out.marker_ids, gray, m_board,
            out.charuco_corners, out.charuco_ids);
    }

    if (n_corners >= m_config.min_corners) {
        out.valid = true;

        if (!camera_matrix.empty() && !dist_coeffs.empty()) {
            out.pose_valid = cv::aruco::estimatePoseCharucoBoard(
                out.charuco_corners, out.charuco_ids, m_board,
                camera_matrix, dist_coeffs, out.rvec, out.tvec);
        } else if (out.charuco_corners.size() >= 6) {
            // Uncalibrated stream: estimate board pose using canonical pinhole approximation
            // to enable real-time 3D tilt tracking and angular coverage binning
            double f = std::max(gray.cols, gray.rows);
            cv::Mat K_approx = (cv::Mat_<double>(3, 3) <<
                f, 0.0, gray.cols / 2.0,
                0.0, f, gray.rows / 2.0,
                0.0, 0.0, 1.0);
            cv::Mat D_approx = cv::Mat::zeros(1, 5, CV_64F);
            try {
                out.pose_valid = cv::aruco::estimatePoseCharucoBoard(
                    out.charuco_corners, out.charuco_ids, m_board,
                    K_approx, D_approx, out.rvec, out.tvec);
            } catch (...) {
                out.pose_valid = false;
            }
        }
    }

    return out.valid;
}

bool CharucoDetector::detect_stereo(const cv::Mat &left_img,
                                   const cv::Mat &right_img,
                                   StereoDetection &out,
                                   const cv::Mat &k_l,
                                   const cv::Mat &d_l,
                                   const cv::Mat &k_r,
                                   const cv::Mat &d_r) {
    out.common_ids.clear();
    out.left_common_corners.clear();
    out.right_common_corners.clear();
    out.object_points.clear();
    out.is_stereo_valid = false;
    out.is_stationary = false;
    out.motion_speed = 0.0f;

    bool left_ok = detect_mono(left_img, out.left, k_l, d_l);
    bool right_ok = detect_mono(right_img, out.right, k_r, d_r);

    if (!left_ok || !right_ok) {
        reset_motion_tracking();
        return false;
    }

    // Match common corners by ID
    std::map<int, cv::Point2f> left_map;
    for (size_t i = 0; i < out.left.charuco_ids.size(); ++i) {
        left_map[out.left.charuco_ids[i]] = out.left.charuco_corners[i];
    }

    for (size_t i = 0; i < out.right.charuco_ids.size(); ++i) {
        int id = out.right.charuco_ids[i];
        auto it = left_map.find(id);
        if (it != left_map.end()) {
            out.common_ids.push_back(id);
            out.left_common_corners.push_back(it->second);
            out.right_common_corners.push_back(out.right.charuco_corners[i]);
            out.object_points.push_back(get_corner_3d(id));
        }
    }

    int required_corners = std::max(6, m_config.min_corners);
    if (static_cast<int>(out.common_ids.size()) >= required_corners) {
        // Verify points span both dimensions of the board (not all collinear on a single row or column)
        float min_x = out.object_points[0].x, max_x = out.object_points[0].x;
        float min_y = out.object_points[0].y, max_y = out.object_points[0].y;
        for (const auto &pt : out.object_points) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }
        bool has_2d_spread = (max_x - min_x >= 0.5f * m_config.square_length_mm) &&
                             (max_y - min_y >= 0.5f * m_config.square_length_mm);

        if (has_2d_spread) {
            out.is_stereo_valid = true;

            // Motion & Stationarity Estimation
            if (m_has_prev_corners && !m_prev_left_corners.empty()) {
                float total_displacement = 0.0f;
                int matched_prev_count = 0;

                for (size_t i = 0; i < out.common_ids.size(); ++i) {
                    int id = out.common_ids[i];
                    auto pit = m_prev_left_corners.find(id);
                    if (pit != m_prev_left_corners.end()) {
                        cv::Point2f diff = out.left_common_corners[i] - pit->second;
                        total_displacement += std::sqrt(diff.x * diff.x + diff.y * diff.y);
                        matched_prev_count++;
                    }
                }

                if (matched_prev_count >= required_corners) {
                    float raw_speed = total_displacement / static_cast<float>(matched_prev_count);
                    if (m_smoothed_speed <= 0.001f) {
                        m_smoothed_speed = raw_speed;
                    } else {
                        m_smoothed_speed = 0.5f * raw_speed + 0.5f * m_smoothed_speed;
                    }
                    out.motion_speed = m_smoothed_speed;
                    out.is_stationary = (out.motion_speed <= m_config.motion_threshold_px);
                }
            }

            // Update previous corners cache for next frame
            m_prev_left_corners.clear();
            for (size_t i = 0; i < out.common_ids.size(); ++i) {
                m_prev_left_corners[out.common_ids[i]] = out.left_common_corners[i];
            }
            m_has_prev_corners = true;
        } else {
            reset_motion_tracking();
        }
    } else {
        reset_motion_tracking();
    }

    return out.is_stereo_valid;
}

void CharucoDetector::draw_mono_detection(cv::Mat &image,
                                         const MonoDetection &detection,
                                         bool draw_axes,
                                         const cv::Mat &camera_matrix,
                                         const cv::Mat &dist_coeffs) {
    if (image.empty()) return;

    // Draw detected markers
    if (!detection.marker_ids.empty()) {
        cv::aruco::drawDetectedMarkers(image, detection.marker_corners, detection.marker_ids, cv::Scalar(100, 100, 255));
    }

    // Draw ChArUco corners
    if (!detection.charuco_corners.empty()) {
        cv::aruco::drawDetectedCornersCharuco(image, detection.charuco_corners, detection.charuco_ids, cv::Scalar(0, 255, 0));
    }

    // Draw coordinate axes if pose is valid
    if (draw_axes && detection.pose_valid && !camera_matrix.empty() && !dist_coeffs.empty()) {
        cv::drawFrameAxes(image, camera_matrix, dist_coeffs, detection.rvec, detection.tvec,
                          m_config.square_length_mm * 2.0f);
    }
}

void CharucoDetector::draw_stereo_detection(cv::Mat &left_img,
                                           cv::Mat &right_img,
                                           const StereoDetection &detection) {
    // Draw mono features
    draw_mono_detection(left_img, detection.left);
    draw_mono_detection(right_img, detection.right);

    // Overlay common corner highlights
    for (const auto &pt : detection.left_common_corners) {
        cv::circle(left_img, pt, 5, cv::Scalar(255, 200, 0), 1, cv::LINE_AA);
    }
    for (const auto &pt : detection.right_common_corners) {
        cv::circle(right_img, pt, 5, cv::Scalar(255, 200, 0), 1, cv::LINE_AA);
    }

    // Draw status badge on top-left of Left image
    std::string status_text;
    cv::Scalar status_color;

    if (!detection.is_stereo_valid) {
        status_text = "No Stereo Board (Need >= " + std::to_string(m_config.min_corners) + " corners)";
        status_color = cv::Scalar(0, 0, 255); // Red
    } else if (detection.is_stationary) {
        status_text = "Board Ready: Still (" + std::to_string(detection.common_ids.size()) + " corners)";
        status_color = cv::Scalar(0, 255, 0); // Green
    } else {
        status_text = "Board Moving (" + std::to_string(detection.common_ids.size()) + " corners, speed: " +
                      std::to_string(static_cast<int>(detection.motion_speed * 10.0f) / 10.0f) + "px)";
        status_color = cv::Scalar(0, 165, 255); // Orange
    }

    cv::putText(left_img, status_text, cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(left_img, status_text, cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2, cv::LINE_AA);
}

} // namespace dvrk_data::calibration
