// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/stereo_calibrator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>
#include <limits>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <set>
#include <stdexcept>

namespace dvrk_data::calibration {

namespace {
ProjectionDiagnostics project_sample(const CalibrationFrame &frame, const cv::Mat &K,
                                     const cv::Mat &D, const cv::Vec3d &rvec,
                                     const cv::Vec3d &tvec,
                                     const std::vector<cv::Point2f> &detected) {
    ProjectionDiagnostics result;
    result.rvec = rvec;
    result.tvec = tvec;
    try {
        cv::Matx33d rotation;
        cv::Rodrigues(rvec, rotation);
        std::vector<cv::Point2f> projected;
        cv::projectPoints(frame.object_points, rvec, tvec, K, D, projected);
        double sum = 0;
        for (size_t i = 0; i < projected.size(); ++i) {
            const auto &p = frame.object_points[i];
            const auto camera = rotation * cv::Vec3d(p.x, p.y, p.z) + tvec;
            if (!std::isfinite(camera[2]) || camera[2] <= 1e-6 ||
                !std::isfinite(projected[i].x) || !std::isfinite(projected[i].y) ||
                std::abs(projected[i].x) > 1e7 || std::abs(projected[i].y) > 1e7)
                throw std::runtime_error("Board projects behind camera or outside numeric range");
            const auto delta = projected[i] - detected[i];
            sum += delta.dot(delta);
            result.corners.push_back({frame.charuco_ids[i], detected[i], projected[i],
                                      delta, cv::norm(delta)});
        }
        result.rms_px = std::sqrt(sum / projected.size());
        result.valid = true;
    } catch (const std::exception &e) {
        result.corners.clear();
        result.reason = e.what();
    }
    return result;
}
}

SampleDiagnostics StereoCalibrator::diagnose_sample(const CalibrationFrame &frame,
                                                     const StereoCalibrationResult &calib, bool robust_pose) {
    SampleDiagnostics result;
    auto fail = [&](const std::string &reason) {
        result.left_fit.reason = result.right_fit.reason = reason;
        result.left_to_right.reason = result.right_to_left.reason = reason;
        result.epipolar_reason = reason;
        return result;
    };
    if (!calib.success || calib.K1.empty() || calib.K2.empty() || calib.R.empty() || calib.T.empty())
        return fail("Run stereo calibration before validating samples.");
    if (frame.left_image.empty() || frame.right_image.empty() ||
        frame.left_image.size() != calib.image_size || frame.right_image.size() != calib.image_size)
        return fail("Sample resolution does not match the calibration.");
    const auto count = frame.object_points.size();
    if (count < 6 || frame.left_corners.size() != count || frame.right_corners.size() != count ||
        frame.charuco_ids.size() != count)
        return fail("Sample needs at least six matching board corners and IDs.");
    std::set<int> ids;
    for (size_t i = 0; i < count; ++i) {
        const auto &p = frame.object_points[i];
        const auto &l = frame.left_corners[i]; const auto &r = frame.right_corners[i];
        if (frame.charuco_ids[i] < 0 || !ids.insert(frame.charuco_ids[i]).second ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(l.x) || !std::isfinite(l.y) || !std::isfinite(r.x) || !std::isfinite(r.y))
            return fail("Invalid or duplicate corner correspondences.");
    }
    bool spans_plane = false;
    for (size_t i = 1; i < count && !spans_plane; ++i)
        for (size_t j = i + 1; j < count; ++j)
            if (cv::norm((frame.object_points[i] - frame.object_points[0]).cross(
                          frame.object_points[j] - frame.object_points[0])) > 1e-6) {
                spans_plane = true; break;
            }
    if (!spans_plane) return fail("Detected board corners do not span a plane.");
    auto fit = [&](const cv::Mat &K, const cv::Mat &D, const std::vector<cv::Point2f> &points) {
        ProjectionDiagnostics fit_result;
        try {
            cv::Vec3d rvec, tvec;
            if (robust_pose) {
                cv::Mat inliers;
                if (!cv::solvePnPRansac(frame.object_points,points,K,D,rvec,tvec,false,100,3.0,0.99,inliers) || inliers.total()<6)
                    throw std::runtime_error("Insufficient robust pose inliers.");
                std::vector<cv::Point3f> objects; std::vector<cv::Point2f> pixels;
                for (int i=0;i<inliers.rows;++i) {
                    const int k=inliers.at<int>(i); objects.push_back(frame.object_points[k]); pixels.push_back(points[k]);
                }
                cv::solvePnP(objects,pixels,K,D,rvec,tvec,true);
            } else if (!cv::solvePnP(frame.object_points, points, K, D, rvec, tvec))
                throw std::runtime_error("Could not estimate board pose.");
            fit_result = project_sample(frame, K, D, rvec, tvec, points);
        } catch (const std::exception &e) { fit_result.reason = e.what(); }
        return fit_result;
    };
    result.left_fit = fit(calib.K1, calib.D1, frame.left_corners);
    result.right_fit = fit(calib.K2, calib.D2, frame.right_corners);
    auto transfer = [&](const ProjectionDiagnostics &source, bool from_left) {
        ProjectionDiagnostics target;
        if (!source.valid) { target.reason = "Source pose unavailable: " + source.reason; return target; }
        try {
            cv::Mat source_rotation;
            cv::Rodrigues(source.rvec, source_rotation);
            cv::Mat R, T;
            calib.R.convertTo(R, CV_64F); calib.T.reshape(1, 3).convertTo(T, CV_64F);
            cv::Mat rotation, translation, rvec;
            if (from_left) {
                rotation = R * source_rotation;
                translation = R * cv::Mat(source.tvec) + T;
            } else {
                rotation = R.t() * source_rotation;
                translation = R.t() * (cv::Mat(source.tvec) - T);
            }
            cv::Rodrigues(rotation, rvec);
            cv::Vec3d rv(rvec), tv(translation);
            target = project_sample(frame, from_left ? calib.K2 : calib.K1,
                                     from_left ? calib.D2 : calib.D1, rv, tv,
                                     from_left ? frame.right_corners : frame.left_corners);
        } catch (const std::exception &e) { target.reason = e.what(); }
        return target;
    };
    result.left_to_right = transfer(result.left_fit, true);
    result.right_to_left = transfer(result.right_fit, false);
    try {
        if (calib.R1.empty() || calib.R2.empty() || calib.P1.empty() || calib.P2.empty())
            throw std::runtime_error("Rectification unavailable.");
        std::vector<cv::Point2f> left, right;
        cv::undistortPoints(frame.left_corners, left, calib.K1, calib.D1, calib.R1, calib.P1);
        cv::undistortPoints(frame.right_corners, right, calib.K2, calib.D2, calib.R2, calib.P2);
        double sum = 0;
        for (size_t i = 0; i < count; ++i) {
            const double delta = left[i].y - right[i].y;
            if (!std::isfinite(delta)) throw std::runtime_error("Non-finite rectified corner.");
            sum += delta * delta;
            result.epipolar_by_corner_id[frame.charuco_ids[i]] = std::abs(delta);
        }
        result.epipolar_rms_px = std::sqrt(sum / count);
    } catch (const std::exception &e) { result.epipolar_reason = e.what(); }
    return result;
}

bool StereoCalibrator::render_grid(const CalibrationFrame &frame, const CharucoConfig &board,
                                   const StereoCalibrationResult &calib,
                                   const SampleDiagnostics &diagnostics, ProjectionMode mode,
                                   double color_max_px, int selected_corner_id,
                                   GridReprojection &output, std::string &error_msg) {
    output = GridReprojection();
    error_msg.clear();
    if (board.squares_x < 2 || board.squares_y < 2 || !std::isfinite(board.square_length_mm) ||
        board.square_length_mm <= 0 || !std::isfinite(color_max_px) || color_max_px <= 0) {
        error_msg = "Invalid board dimensions or residual scale."; return false;
    }
    const auto &left = mode == ProjectionMode::RIGHT_TO_LEFT ? diagnostics.right_to_left : diagnostics.left_fit;
    const auto &right = mode == ProjectionMode::LEFT_TO_RIGHT ? diagnostics.left_to_right : diagnostics.right_fit;
    if (!left.valid || !right.valid) {
        error_msg = !left.valid ? "Left: " + left.reason : "Right: " + right.reason;
        return false;
    }
    try {
        auto render = [&](const cv::Mat &image, const cv::Mat &K, const cv::Mat &D,
                          const ProjectionDiagnostics &fit, cv::Mat &overlay) {
            if (image.channels() == 1) cv::cvtColor(image, overlay, cv::COLOR_GRAY2BGR);
            else overlay = image.clone();
            cv::Matx33d rotation; cv::Rodrigues(fit.rvec, rotation);
            auto in_front = [&](const cv::Point3f &p) {
                return (rotation * cv::Vec3d(p.x, p.y, p.z) + fit.tvec)[2] > 1e-6;
            };
            const float width = board.squares_x * board.square_length_mm;
            const float height = board.squares_y * board.square_length_mm;
            for (int axis = 0; axis < 2; ++axis) {
                const int n = axis ? board.squares_y : board.squares_x;
                for (int i = 0; i <= n; ++i) {
                    std::vector<cv::Point3f> line;
                    for (int j = 0; j <= 64; ++j)
                        line.emplace_back(axis ? width*j/64 : i*board.square_length_mm,
                                          axis ? i*board.square_length_mm : height*j/64, 0);
                    std::vector<cv::Point2f> projected;
                    cv::projectPoints(line, fit.rvec, fit.tvec, K, D, projected);
                    for (size_t j = 1; j < projected.size(); ++j) {
                        const auto a = projected[j-1], b = projected[j];
                        if (!in_front(line[j-1]) || !in_front(line[j]) ||
                            !std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) || !std::isfinite(b.y) ||
                            std::max({std::abs(a.x), std::abs(a.y), std::abs(b.x), std::abs(b.y)}) > 1e7) continue;
                        cv::Point pa = a, pb = b;
                        if (cv::clipLine(overlay.size(), pa, pb)) cv::line(overlay, pa, pb, cv::Scalar(75,110,75), 1, cv::LINE_AA);
                    }
                }
            }
            for (const auto &corner : fit.corners) {
                const double f = std::clamp(corner.magnitude_px / color_max_px, 0.0, 1.0);
                const cv::Scalar color(0, 255*std::min(1.0,2*(1-f)), 255*std::min(1.0,2*f));
                cv::Point a = corner.detected, b = corner.projected;
                if (cv::clipLine(overlay.size(), a, b)) cv::arrowedLine(overlay, a, b, color, 1, cv::LINE_AA, 0, 0.3);
                if (!cv::Rect(0,0,overlay.cols,overlay.rows).contains(cv::Point(corner.detected))) continue;
                cv::circle(overlay, corner.detected, 3, color, cv::FILLED, cv::LINE_AA);
                if (corner.id == selected_corner_id) {
                    cv::circle(overlay, corner.detected, 7, cv::Scalar(255,255,255), 2, cv::LINE_AA);
                    cv::putText(overlay, "#" + std::to_string(corner.id), corner.detected + cv::Point2f(8,-8),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,255), 1, cv::LINE_AA);
                }
            }
        };
        render(frame.left_image, calib.K1, calib.D1, left, output.left_overlay);
        render(frame.right_image, calib.K2, calib.D2, right, output.right_overlay);
        output.left_rms_px = left.rms_px; output.right_rms_px = right.rms_px;
        return true;
    } catch (const std::exception &e) {
        output = GridReprojection(); error_msg = e.what(); return false;
    }
}

bool StereoCalibrator::reproject_grid(const CalibrationFrame &frame, const CharucoConfig &board,
                                      const StereoCalibrationResult &calib, GridReprojection &output,
                                      std::string &error_msg) {
    return render_grid(frame, board, calib, diagnose_sample(frame, calib),
                       ProjectionMode::LEFT_TO_RIGHT, 3.0, -1, output, error_msg);
}

int StereoCalibrator::add_frame(const cv::Mat &left_img,
                               const cv::Mat &right_img,
                               const StereoDetection &detection) {
    return add_frame(left_img, right_img, detection, 0);
}

int StereoCalibrator::add_frame(const cv::Mat &left_img,
                               const cv::Mat &right_img,
                               const StereoDetection &detection, int frame_id) {
    if (frame_id < 0 || frame_id == std::numeric_limits<int>::max() ||
        m_next_frame_id == std::numeric_limits<int>::max() ||
        (frame_id && std::any_of(m_frames.begin(), m_frames.end(),
            [frame_id](const auto &frame) { return frame.id == frame_id; }))) return -1;
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
    frame.id = frame_id ? frame_id : m_next_frame_id;
    m_next_frame_id = std::max(m_next_frame_id, frame.id + 1);
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
    std::vector<size_t> valid_frame_indices;

    for (auto &f : m_frames) {
        f.epipolar_rms_px = std::numeric_limits<double>::quiet_NaN();
    }

    for (size_t i = 0; i < m_frames.size(); ++i) {
        const auto &f = m_frames[i];
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
        valid_frame_indices.push_back(i);
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

    result.per_frame_epipolar_rms_px.clear();
    result.epipolar_rms_by_id.clear();

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
        result.per_frame_epipolar_rms_px.push_back(frame_rms);
        if (fi < valid_frame_indices.size()) {
            size_t frame_idx = valid_frame_indices[fi];
            m_frames[frame_idx].epipolar_rms_px = frame_rms;
            result.epipolar_rms_by_id[m_frames[frame_idx].id] = frame_rms;
        }
    }

    result.avg_epipolar_error_px = (point_count > 0) ? (total_epipolar_err / point_count) : 0.0;
    result.max_epipolar_error_px = max_epipolar_err;
    const auto completed_at = std::time(nullptr);
    std::tm utc{};
    if (gmtime_r(&completed_at, &utc)) {
        std::ostringstream date;
        date << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
        result.calibration_date = date.str();
    }
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
                                                    int line_spacing) {
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

    // Dividing line between left and right views
    cv::line(sbs, cv::Point(w, 0), cv::Point(w, h), cv::Scalar(255, 255, 255), 2);

    return sbs;
}

} // namespace dvrk_data::calibration
