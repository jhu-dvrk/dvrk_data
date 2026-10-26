// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/stereo_calibrator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <sstream>

namespace dvrk_data::calibration {

int StereoCalibrator::add_frame(const cv::Mat &left_img,
                               const cv::Mat &right_img,
                               const StereoDetection &detection) {
    if (!detection.is_stereo_valid || detection.common_ids.size() < 6 || detection.object_points.size() < 6) {
        return -1;
    }

    if (left_img.empty() || right_img.empty() || left_img.size() != right_img.size()) {
        return -1;
    }

    // Reject frames whose resolution does not match previously collected frames
    if (!m_frames.empty() && left_img.size() != m_frames[0].left_image.size()) {
        std::cerr << "[StereoCalibrator] Rejecting frame with mismatched resolution: "
                  << left_img.cols << "x" << left_img.rows << " vs expected "
                  << m_frames[0].left_image.cols << "x" << m_frames[0].left_image.rows << std::endl;
        return -1;
    }

    CalibrationFrame frame;
    frame.id = m_next_frame_id++;
    frame.left_image = left_img.clone();
    frame.right_image = right_img.clone();
    frame.charuco_ids = detection.common_ids;
    frame.left_corners = detection.left_common_corners;
    frame.right_corners = detection.right_common_corners;
    frame.object_points = detection.object_points;
    frame.rvec = detection.left.rvec;
    frame.pose_valid = detection.left.pose_valid;

    m_frames.push_back(frame);
    return frame.id;
}

bool StereoCalibrator::remove_frame(int frame_id) {
    auto it = std::find_if(m_frames.begin(), m_frames.end(),
                           [frame_id](const CalibrationFrame &f) { return f.id == frame_id; });
    if (it != m_frames.end()) {
        m_frames.erase(it);
        return true;
    }
    return false;
}

void StereoCalibrator::clear_frames() {
    m_frames.clear();
}

bool StereoCalibrator::calibrate(const CalibratorOptions &opts,
                                StereoCalibrationResult &result,
                                std::string &error_msg,
                                ProgressCallback progress_cb) {
    result = StereoCalibrationResult();
    result.model = opts.model;

    if (m_frames.size() < 5) {
        error_msg = "Need at least 5 calibration frames (collected: " +
                    std::to_string(m_frames.size()) + ").";
        return false;
    }

    if (progress_cb) {
        progress_cb(0.05, "Preparing calibration correspondence data (" +
                    std::to_string(m_frames.size()) + " frames)...");
    }

    cv::Size img_size = m_frames[0].left_image.size();
    result.image_size = img_size;

    std::vector<std::vector<cv::Point3f>> all_object_points;
    std::vector<std::vector<cv::Point2f>> all_left_points;
    std::vector<std::vector<cv::Point2f>> all_right_points;

    for (const auto &f : m_frames) {
        // Reject incompatible frame dimensions
        if (f.left_image.size() != img_size || f.right_image.size() != img_size) {
            error_msg = "Mismatched frame resolutions in calibration set (" +
                        std::to_string(f.left_image.cols) + "x" + std::to_string(f.left_image.rows) +
                        " vs expected " + std::to_string(img_size.width) + "x" +
                        std::to_string(img_size.height) + "). Please clear frames and re-collect.";
            return false;
        }

        if (f.object_points.size() < 6) {
            continue; // DLT pose estimation requires at least 6 points
        }
        // Verify points span both dimensions of the board (not all collinear on a single row or column)
        float min_x = f.object_points[0].x, max_x = f.object_points[0].x;
        float min_y = f.object_points[0].y, max_y = f.object_points[0].y;
        for (const auto &pt : f.object_points) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }
        if (std::abs(max_x - min_x) < 1e-3f || std::abs(max_y - min_y) < 1e-3f) {
            continue; // Skip collinear points
        }

        all_object_points.push_back(f.object_points);
        all_left_points.push_back(f.left_corners);
        all_right_points.push_back(f.right_corners);
    }

    if (all_object_points.size() < 5) {
        error_msg = "Insufficient valid frames for calibration (need at least 5 frames with >= 6 non-collinear corners; valid: " +
                    std::to_string(all_object_points.size()) + ").";
        return false;
    }

    auto make_default_K = [](const cv::Size &sz) {
        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        double f = std::max(sz.width, sz.height);
        K.at<double>(0, 0) = f;
        K.at<double>(1, 1) = f;
        K.at<double>(0, 2) = sz.width / 2.0;
        K.at<double>(1, 2) = sz.height / 2.0;
        return K;
    };

    if (progress_cb) {
        progress_cb(0.15, "Initializing camera intrinsic parameters...");
    }

    cv::Mat K1, K2;
    try {
        K1 = cv::initCameraMatrix2D(all_object_points, all_left_points, img_size);
        if (K1.empty() || K1.at<double>(0, 0) < 10.0 || !std::isfinite(K1.at<double>(0, 0))) {
            K1 = make_default_K(img_size);
        }
    } catch (...) {
        K1 = make_default_K(img_size);
    }

    try {
        K2 = cv::initCameraMatrix2D(all_object_points, all_right_points, img_size);
        if (K2.empty() || K2.at<double>(0, 0) < 10.0 || !std::isfinite(K2.at<double>(0, 0))) {
            K2 = make_default_K(img_size);
        }
    } catch (...) {
        K2 = make_default_K(img_size);
    }

    // Monocular calibration flags
    int mono_flags = cv::CALIB_USE_INTRINSIC_GUESS;
    if (opts.model == DistortionModel::RATIONAL_POLYNOMIAL) {
        mono_flags |= cv::CALIB_RATIONAL_MODEL;
    }
    if (opts.fix_aspect_ratio) {
        mono_flags |= cv::CALIB_FIX_ASPECT_RATIO;
    }
    if (opts.zero_tangent_dist) {
        mono_flags |= cv::CALIB_ZERO_TANGENT_DIST;
    }

    cv::Mat D1, D2;
    std::vector<cv::Mat> rvecs1, tvecs1, rvecs2, tvecs2;

    try {
        if (progress_cb) {
            progress_cb(0.25, "Calibrating Left camera intrinsics and distortion...");
        }
        cv::calibrateCamera(all_object_points, all_left_points, img_size, K1, D1,
                            rvecs1, tvecs1, mono_flags,
                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 1e-4));

        if (progress_cb) {
            progress_cb(0.50, "Calibrating Right camera intrinsics and distortion...");
        }
        cv::calibrateCamera(all_object_points, all_right_points, img_size, K2, D2,
                            rvecs2, tvecs2, mono_flags,
                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 1e-4));
    } catch (const cv::Exception &e) {
        error_msg = std::string("Monocular calibration failed: ") + e.what();
        return false;
    }

    // Joint Stereo Calibration
    int stereo_flags = 0;
    if (opts.fix_intrinsics) {
        stereo_flags |= cv::CALIB_FIX_INTRINSIC;
    } else {
        stereo_flags |= cv::CALIB_USE_INTRINSIC_GUESS;
    }
    if (opts.model == DistortionModel::RATIONAL_POLYNOMIAL) {
        stereo_flags |= cv::CALIB_RATIONAL_MODEL;
    }
    if (opts.zero_tangent_dist) {
        stereo_flags |= cv::CALIB_ZERO_TANGENT_DIST;
    }

    cv::Mat R, T, E, F;
    try {
        if (progress_cb) {
            progress_cb(0.70, "Optimizing stereo baseline and extrinsics (R, T)...");
        }
        result.rms_error = cv::stereoCalibrate(
            all_object_points, all_left_points, all_right_points,
            K1, D1, K2, D2, img_size, R, T, E, F,
            stereo_flags,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 1e-5)
        );
    } catch (const cv::Exception &e) {
        error_msg = std::string("Stereo calibration failed: ") + e.what();
        return false;
    }

    if (progress_cb) {
        progress_cb(0.85, "Computing epipolar rectification maps...");
    }

    result.K1 = K1;
    result.D1 = D1;
    result.K2 = K2;
    result.D2 = D2;
    result.R = R;
    result.T = T;
    result.E = E;
    result.F = F;
    result.baseline_mm = cv::norm(T);

    // Stereo Rectification
    cv::stereoRectify(
        K1, D1, K2, D2, img_size, R, T,
        result.R1, result.R2, result.P1, result.P2, result.Q,
        cv::CALIB_ZERO_DISPARITY, opts.alpha_rectification, img_size,
        &result.roi1, &result.roi2
    );

    // Precalculate Undistort & Rectify lookup maps
    cv::initUndistortRectifyMap(K1, D1, result.R1, result.P1, img_size,
                                CV_32FC1, result.map1_l, result.map2_l);
    cv::initUndistortRectifyMap(K2, D2, result.R2, result.P2, img_size,
                                CV_32FC1, result.map1_r, result.map2_r);

    // Calculate vertical epipolar error on all collected points
    double total_epipolar_err = 0.0;
    double max_epipolar_err = 0.0;
    int point_count = 0;

    result.per_frame_rms.clear();
    result.per_frame_error_by_id.clear();

    for (size_t fi = 0; fi < all_object_points.size(); ++fi) {
        std::vector<cv::Point2f> rect_pts1, rect_pts2;
        cv::undistortPoints(all_left_points[fi], rect_pts1, K1, D1, result.R1, result.P1);
        cv::undistortPoints(all_right_points[fi], rect_pts2, K2, D2, result.R2, result.P2);

        double frame_err_sum = 0.0;
        for (size_t pi = 0; pi < rect_pts1.size(); ++pi) {
            double v_err = std::abs(rect_pts1[pi].y - rect_pts2[pi].y);
            total_epipolar_err += v_err;
            frame_err_sum += v_err * v_err;
            if (v_err > max_epipolar_err) {
                max_epipolar_err = v_err;
            }
            point_count++;
        }

        double frame_rms = rect_pts1.empty() ? 0.0 : std::sqrt(frame_err_sum / rect_pts1.size());
        result.per_frame_rms.push_back(frame_rms);
        if (fi < m_frames.size()) {
            m_frames[fi].reprojection_error = frame_rms;
            result.per_frame_error_by_id[m_frames[fi].id] = frame_rms;
        }
    }

    result.avg_epipolar_error_px = (point_count > 0) ? (total_epipolar_err / point_count) : 0.0;
    result.max_epipolar_error_px = max_epipolar_err;
    result.success = true;

    if (progress_cb) {
        progress_cb(1.0, "Calibration completed successfully!");
    }

    return true;
}

void StereoCalibrator::rectify_stereo(const cv::Mat &in_left,
                                     const cv::Mat &in_right,
                                     cv::Mat &out_left,
                                     cv::Mat &out_right,
                                     const StereoCalibrationResult &calib) {
    if (!calib.success || in_left.empty() || in_right.empty()) {
        out_left = in_left;
        out_right = in_right;
        return;
    }

    if (in_left.size() != in_right.size() ||
        (calib.image_size.width > 0 && in_left.size() != calib.image_size) ||
        (!calib.map1_l.empty() && in_left.size() != calib.map1_l.size()) ||
        (!calib.map1_r.empty() && in_right.size() != calib.map1_r.size())) {
        out_left = in_left;
        out_right = in_right;
        return;
    }

    cv::remap(in_left, out_left, calib.map1_l, calib.map2_l, cv::INTER_LINEAR);
    cv::remap(in_right, out_right, calib.map1_r, calib.map2_r, cv::INTER_LINEAR);
}

cv::Mat StereoCalibrator::make_anaglyph(const cv::Mat &rect_left, const cv::Mat &rect_right) {
    if (rect_left.empty() || rect_right.empty() || rect_left.size() != rect_right.size()) {
        return rect_left;
    }

    cv::Mat l_bgr = rect_left;
    cv::Mat r_bgr = rect_right;

    cv::Mat anaglyph(rect_left.rows, rect_left.cols, CV_8UC3);

    // Left eye -> Red channel
    // Right eye -> Blue and Green channels
    int total_pixels = rect_left.rows * rect_left.cols;
    const uint8_t *l_ptr = l_bgr.data;
    const uint8_t *r_ptr = r_bgr.data;
    uint8_t *out_ptr = anaglyph.data;

    for (int i = 0; i < total_pixels; ++i) {
        out_ptr[0] = r_ptr[0]; // B
        out_ptr[1] = r_ptr[1]; // G
        out_ptr[2] = l_ptr[2]; // R
        l_ptr += 3;
        r_ptr += 3;
        out_ptr += 3;
    }

    return anaglyph;
}

cv::Mat StereoCalibrator::make_side_by_side_epipolar(const cv::Mat &rect_left,
                                                    const cv::Mat &rect_right,
                                                    int line_spacing,
                                                    int active_line_y) {
    if (rect_left.empty() || rect_right.empty()) return rect_left;

    int w = rect_left.cols;
    int h = rect_left.rows;

    cv::Mat sbs(h, w * 2, CV_8UC3);
    rect_left.copyTo(sbs(cv::Rect(0, 0, w, h)));
    rect_right.copyTo(sbs(cv::Rect(w, 0, w, h)));

    // Draw horizontal epipolar lines across both images
    if (line_spacing > 0) {
        for (int y = line_spacing; y < h; y += line_spacing) {
            cv::line(sbs, cv::Point(0, y), cv::Point(w * 2, y),
                     cv::Scalar(0, 220, 0), 1, cv::LINE_AA);
        }
    }

    // Active line highlight
    if (active_line_y >= 0 && active_line_y < h) {
        cv::line(sbs, cv::Point(0, active_line_y), cv::Point(w * 2, active_line_y),
                 cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    // Dividing line between left and right views
    cv::line(sbs, cv::Point(w, 0), cv::Point(w, h), cv::Scalar(255, 255, 255), 2);

    return sbs;
}

} // namespace dvrk_data::calibration
