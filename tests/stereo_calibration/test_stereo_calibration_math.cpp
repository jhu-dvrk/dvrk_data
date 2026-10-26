// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/charuco_detector.hpp"
#include "dvrk_data/calibration/stereo_calibrator.hpp"
#include "dvrk_data/calibration/calibration_io.hpp"
#include "dvrk_data/calibration/board_config.hpp"

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <cassert>
#include <filesystem>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    std::cout << "[Test] Running automated stereo calibration math verification..." << std::endl;

    dvrk_data::calibration::CharucoConfig charuco_cfg;
    charuco_cfg.squares_x = 8;
    charuco_cfg.squares_y = 6;
    charuco_cfg.square_length_mm = 5.0f;
    charuco_cfg.marker_length_mm = 3.5f;

    dvrk_data::calibration::CharucoDetector detector(charuco_cfg);
    dvrk_data::calibration::StereoCalibrator calibrator;

    // Ground truth camera parameters
    cv::Size img_size(640, 480);
    cv::Mat K_gt = (cv::Mat_<double>(3, 3) << 500.0, 0.0, 320.0,
                                               0.0, 500.0, 240.0,
                                               0.0, 0.0, 1.0);
    cv::Mat D_gt = (cv::Mat_<double>(1, 5) << 0.05, -0.02, 0.001, -0.001, 0.0);

    // True stereo baseline: 6.0 mm along X
    cv::Mat R_gt = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat T_gt = (cv::Mat_<double>(3, 1) << -6.0, 0.0, 0.0);

    auto board = detector.get_board();
    const auto &object_pts = board->chessboardCorners;

    // Generate 8 diverse synthetic board poses
    std::vector<cv::Vec3d> test_rvecs = {
        cv::Vec3d(0.0, 0.0, 0.0),
        cv::Vec3d(0.2, 0.0, 0.1),
        cv::Vec3d(-0.2, 0.0, -0.1),
        cv::Vec3d(0.0, 0.25, 0.0),
        cv::Vec3d(0.0, -0.25, 0.0),
        cv::Vec3d(0.15, 0.15, 0.05),
        cv::Vec3d(-0.15, -0.15, -0.05),
        cv::Vec3d(0.1, -0.2, 0.1)
    };

    std::vector<cv::Vec3d> test_tvecs = {
        cv::Vec3d(-20.0, -15.0, 120.0),
        cv::Vec3d(15.0, -10.0, 110.0),
        cv::Vec3d(-10.0, 20.0, 130.0),
        cv::Vec3d(10.0, 15.0, 115.0),
        cv::Vec3d(0.0, 0.0, 100.0),
        cv::Vec3d(-15.0, -5.0, 125.0),
        cv::Vec3d(20.0, 10.0, 105.0),
        cv::Vec3d(-5.0, 15.0, 118.0)
    };

    for (size_t i = 0; i < test_rvecs.size(); ++i) {
        std::vector<cv::Point2f> left_pts, right_pts;
        cv::projectPoints(object_pts, test_rvecs[i], test_tvecs[i], K_gt, D_gt, left_pts);

        // Right eye pose
        cv::Mat R_mat;
        cv::Rodrigues(test_rvecs[i], R_mat);
        cv::Mat R_right = R_gt * R_mat;
        cv::Vec3d rvec_right;
        cv::Rodrigues(R_right, rvec_right);
        cv::Vec3d tvec_right = cv::Vec3d(
            test_tvecs[i][0] + T_gt.at<double>(0),
            test_tvecs[i][1] + T_gt.at<double>(1),
            test_tvecs[i][2] + T_gt.at<double>(2)
        );

        cv::projectPoints(object_pts, rvec_right, tvec_right, K_gt, D_gt, right_pts);

        dvrk_data::calibration::StereoDetection det;
        det.is_stereo_valid = true;
        det.object_points = object_pts;
        det.left_common_corners = left_pts;
        det.right_common_corners = right_pts;
        for (size_t c = 0; c < object_pts.size(); ++c) {
            det.common_ids.push_back(static_cast<int>(c));
        }

        cv::Mat dummy_l(img_size, CV_8UC3, cv::Scalar(0));
        cv::Mat dummy_r(img_size, CV_8UC3, cv::Scalar(0));
        calibrator.add_frame(dummy_l, dummy_r, det);
    }

    assert(calibrator.frame_count() == test_rvecs.size());

    dvrk_data::calibration::CalibratorOptions opts;
    opts.model = dvrk_data::calibration::DistortionModel::PLUMB_BOB;
    dvrk_data::calibration::StereoCalibrationResult result;
    std::string err;

    bool ok = calibrator.calibrate(opts, result, err);
    if (!ok) {
        std::cerr << "Calibration failed: " << err << std::endl;
        return 1;
    }

    std::cout << "Calibration computed successfully!\n"
              << "  RMS Error: " << result.rms_error << " px\n"
              << "  Computed Baseline: " << result.baseline_mm << " mm (Ground truth: 6.00 mm)\n"
              << "  Avg Vertical Epipolar Error: " << result.avg_epipolar_error_px << " px\n";

    // Verify baseline accuracy
    double baseline_diff = std::abs(result.baseline_mm - 6.0);
    if (baseline_diff >= 0.25) {
        std::cerr << "Baseline error too large: " << baseline_diff << std::endl;
        return 1;
    }
    if (result.avg_epipolar_error_px >= 0.5) {
        std::cerr << "Epipolar error too large: " << result.avg_epipolar_error_px << std::endl;
        return 1;
    }

    // Test saving and loading calibration file
    char temp_pattern[] = "/tmp/dvrk-data-calib-XXXXXX";
    const char *created = mkdtemp(temp_pattern);
    if (!created) { std::cerr << "Cannot create test directory" << std::endl; return 1; }
    std::string tmp_dir = created;
    bool save_ok = dvrk_data::calibration::CalibrationIO::save_stereo_calibration_file(
        tmp_dir + "/stereo_calibration.yaml", result);
    if (!save_ok) {
        std::cerr << "Failed to save stereo_calibration.yaml" << std::endl;
        return 1;
    }

    bool save_ros_ok = dvrk_data::calibration::CalibrationIO::save_ros_camera_info_pair(
        tmp_dir, result);
    if (!save_ros_ok) {
        std::cerr << "Failed to save ROS CameraInfo pair" << std::endl;
        return 1;
    }

    // Verify ROS CameraInfo P2 translation column is in meters
    std::ifstream ifs_right(tmp_dir + "/right.yaml");
    if (!ifs_right.is_open()) {
        std::cerr << "Failed to read exported right.yaml" << std::endl;
        return 1;
    }
    std::string line;
    bool in_proj = false;
    bool verified_meters = false;
    while (std::getline(ifs_right, line)) {
        if (line.find("projection_matrix:") != std::string::npos) {
            in_proj = true;
        }
        if (in_proj && line.find("data: [") != std::string::npos) {
            auto start = line.find("[");
            auto end = line.find("]");
            std::string content = line.substr(start + 1, end - start - 1);
            std::stringstream ss(content);
            std::string token;
            int idx = 0;
            double fx = 0.0, tx_m = 0.0;
            while (std::getline(ss, token, ',')) {
                double val = std::stod(token);
                if (idx == 0) fx = val;
                if (idx == 3) tx_m = val;
                idx++;
            }
            double baseline_m = std::abs(tx_m) / fx;
            std::cout << "[Test] ROS right.yaml Tx: " << tx_m << " (baseline: "
                      << baseline_m * 1000.0 << " mm, " << baseline_m << " meters)" << std::endl;
            if (std::abs(baseline_m - (result.baseline_mm / 1000.0)) > 1e-4) {
                std::cerr << "ROS P2 baseline translation is not in meters!" << std::endl;
                return 1;
            }
            verified_meters = true;
            break;
        }
    }
    if (!verified_meters) {
        std::cerr << "Could not verify ROS projection matrix data in right.yaml" << std::endl;
        return 1;
    }

    // Verify resolution mismatch rejection in StereoCalibrator
    cv::Mat mismatched_left = cv::Mat::zeros(240, 320, CV_8UC3);
    cv::Mat mismatched_right = cv::Mat::zeros(240, 320, CV_8UC3);
    dvrk_data::calibration::StereoDetection dummy_det;
    dummy_det.is_stereo_valid = true;
    dummy_det.common_ids = {0, 1, 2, 3, 4, 5};
    dummy_det.left_common_corners = std::vector<cv::Point2f>(6, cv::Point2f(10, 10));
    dummy_det.right_common_corners = std::vector<cv::Point2f>(6, cv::Point2f(10, 10));
    dummy_det.object_points = std::vector<cv::Point3f>(6, cv::Point3f(0, 0, 0));
    int reject_res = calibrator.add_frame(mismatched_left, mismatched_right, dummy_det);
    if (reject_res != -1) {
        std::cerr << "StereoCalibrator failed to reject mismatched resolution frame!" << std::endl;
        return 1;
    }

    dvrk_data::calibration::StereoCalibrationResult loaded_result;
    bool load_ok = dvrk_data::calibration::CalibrationIO::load_stereo_calibration_file(
        tmp_dir + "/stereo_calibration.yaml", loaded_result);
    if (!load_ok) {
        std::cerr << "Failed to load stereo_calibration.yaml" << std::endl;
        return 1;
    }

    if (std::abs(loaded_result.baseline_mm - result.baseline_mm) >= 1e-4) {
        std::cerr << "Loaded baseline does not match saved baseline" << std::endl;
        return 1;
    }

    if (result.per_frame_error_by_id.empty() || result.per_frame_error_by_id.size() != result.per_frame_rms.size()) {
        std::cerr << "result.per_frame_error_by_id is missing or has wrong size!" << std::endl;
        return 1;
    }

    // Test that rectify_stereo rejects mismatched image dimensions
    cv::Mat small_left(120, 160, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat small_right(120, 160, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat rect_out_l, rect_out_r;
    dvrk_data::calibration::StereoCalibrator::rectify_stereo(small_left, small_right, rect_out_l, rect_out_r, loaded_result);
    if (rect_out_l.size() != small_left.size() || rect_out_r.size() != small_right.size()) {
        std::cerr << "rectify_stereo failed to preserve/reject incompatible input resolution!" << std::endl;
        return 1;
    }

    // Test that incomplete calibration YAML (e.g. missing R2 and P2) fails to load
    std::string incomplete_yaml = tmp_dir + "/incomplete_calib.yaml";
    {
        cv::FileStorage fs_inc(incomplete_yaml, cv::FileStorage::WRITE);
        fs_inc << "image_width" << loaded_result.image_size.width;
        fs_inc << "image_height" << loaded_result.image_size.height;
        fs_inc << "K1" << loaded_result.K1;
        fs_inc << "D1" << loaded_result.D1;
        fs_inc << "K2" << loaded_result.K2;
        fs_inc << "D2" << loaded_result.D2;
        fs_inc << "R" << loaded_result.R;
        fs_inc << "T" << loaded_result.T;
        fs_inc << "R1" << loaded_result.R1;
        fs_inc << "P1" << loaded_result.P1;
        // Intentionally omit R2 and P2!
        fs_inc.release();
    }
    dvrk_data::calibration::StereoCalibrationResult inc_result;
    bool inc_load_ok = dvrk_data::calibration::CalibrationIO::load_stereo_calibration_file(incomplete_yaml, inc_result);
    if (inc_load_ok) {
        std::cerr << "CalibrationIO::load_stereo_calibration_file unexpectedly succeeded on incomplete file missing R2/P2!" << std::endl;
        return 1;
    }

    // Test board JSON serialization and deserialization
    std::string test_json_path = tmp_dir + "/test_board.json";
    std::string json_err;
    if (!dvrk_data::calibration::save_board_config(test_json_path, charuco_cfg, json_err)) {
        std::cerr << "Failed to save test_board.json: " << json_err << std::endl;
        return 1;
    }

    dvrk_data::calibration::CharucoConfig loaded_board_cfg;
    if (!dvrk_data::calibration::load_board_config(test_json_path, loaded_board_cfg, json_err)) {
        std::cerr << "Failed to load test_board.json: " << json_err << std::endl;
        return 1;
    }

    if (loaded_board_cfg.squares_x != charuco_cfg.squares_x ||
        loaded_board_cfg.squares_y != charuco_cfg.squares_y ||
        std::abs(loaded_board_cfg.square_length_mm - charuco_cfg.square_length_mm) > 1e-3f ||
        std::abs(loaded_board_cfg.marker_length_mm - charuco_cfg.marker_length_mm) > 1e-3f ||
        loaded_board_cfg.dictionary_id != charuco_cfg.dictionary_id) {
        std::cerr << "Board config values mismatch after JSON roundtrip" << std::endl;
        return 1;
    }

    std::filesystem::remove_all(tmp_dir);

    std::cout << "[Test] All calibration math, rectification, board JSON, and IO checks PASSED!" << std::endl;
    return 0;
}
