// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/charuco_detector.hpp"
#include "dvrk_data/calibration/stereo_calibrator.hpp"
#include "dvrk_data/calibration/calibration_io.hpp"
#include "dvrk_data/calibration/board_config.hpp"

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <regex>
#include <sstream>
#include <cassert>
#include <filesystem>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

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
    if (!std::regex_match(result.calibration_date, std::regex("[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z"))) {
        std::cerr << "Successful calibration must record a UTC date" << std::endl;
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

    // Alpha changes rectification field of view without changing the camera fit.
    auto uncropped_options = opts;
    uncropped_options.alpha_rectification = 1.0;
    dvrk_data::calibration::StereoCalibrationResult uncropped;
    if (!calibrator.calibrate(uncropped_options, uncropped, err) ||
        uncropped.P1.at<double>(0, 0) >= result.P1.at<double>(0, 0) ||
        cv::norm(uncropped.K1, result.K1) > 1e-6 ||
        cv::norm(uncropped.T, result.T) > 1e-6 ||
        uncropped.map1_l.size() != img_size ||
        cv::norm(uncropped.map1_l, result.map1_l, cv::NORM_INF) < 1.0) {
        std::cerr << "Rectification alpha did not retain more field of view: " << err << std::endl;
        return 1;
    }

    // Validate projection with known intrinsics, unequal eyes, distortion and
    // a nontrivial stereo rotation. Pose must be estimated, not taken from capture.
    using namespace dvrk_data::calibration;
    StereoCalibrationResult truth;
    truth.success = true;
    truth.image_size = img_size;
    truth.K1 = K_gt.clone();
    truth.K2 = K_gt.clone();
    truth.K2.at<double>(0, 0) = 515.0;
    cv::Rodrigues(cv::Vec3d(0.01, -0.025, 0.015), truth.R);
    truth.T = T_gt.clone();
    for (int parameters : {5, 8}) {
        truth.D1 = cv::Mat::zeros(1, parameters, CV_64F);
        truth.D2 = cv::Mat::zeros(1, parameters, CV_64F);
        D_gt.copyTo(truth.D1.colRange(0, 5));
        D_gt.copyTo(truth.D2.colRange(0, 5));
        truth.D2.at<double>(0) = 0.08;
        if (parameters == 8) {
            truth.D1.at<double>(5) = 0.012;
            truth.D2.at<double>(5) = -0.018;
        }
        CalibrationFrame sample = calibrator.get_frames()[5];
        sample.rvec = cv::Vec3d(2, 1, -1); // Intentionally wrong capture-time pose.
        sample.pose_valid = true;
        const auto pose = test_rvecs[5];
        const cv::Mat translation = (cv::Mat_<double>(3, 1) <<
            test_tvecs[5][0], test_tvecs[5][1], test_tvecs[5][2]);
        cv::Mat rotation, right_rotation, right_pose;
        cv::Rodrigues(pose, rotation);
        right_rotation = truth.R * rotation;
        cv::Rodrigues(right_rotation, right_pose);
        cv::Mat right_translation = truth.R * translation + truth.T;
        cv::projectPoints(sample.object_points, pose, translation, truth.K1, truth.D1, sample.left_corners);
        cv::projectPoints(sample.object_points, right_pose, right_translation, truth.K2, truth.D2, sample.right_corners);
        GridReprojection projection;
        if (!StereoCalibrator::reproject_grid(sample, charuco_cfg, truth, projection, err) ||
            projection.left_rms_px > 0.001 || projection.right_rms_px > 0.001 ||
            projection.left_overlay.size() != img_size ||
            projection.right_overlay.size() != img_size ||
            cv::norm(sample.left_image, projection.left_overlay) == 0) {
            std::cerr << "Known-geometry grid reprojection failed: " << err << std::endl;
            return 1;
        }
        truth.T.at<double>(1) += 1.0;
        if (!StereoCalibrator::reproject_grid(sample, charuco_cfg, truth, projection, err) ||
            projection.right_rms_px < 3.0 || projection.left_rms_px > 0.001) {
            std::cerr << "Grid validation did not expose incorrect stereo translation" << std::endl;
            return 1;
        }
        truth.T.at<double>(1) -= 1.0;
        sample.right_image = cv::Mat::zeros(120, 160, CV_8UC3);
        if (StereoCalibrator::reproject_grid(sample, charuco_cfg, truth, projection, err) || err.empty()) {
            std::cerr << "Grid validation accepted incompatible resolution" << std::endl;
            return 1;
        }
        sample.right_image = sample.left_image;
        sample.right_corners.pop_back();
        if (StereoCalibrator::reproject_grid(sample, charuco_cfg, truth, projection, err)) {
            std::cerr << "Grid validation accepted mismatched correspondences" << std::endl;
            return 1;
        }
        sample.right_corners = sample.left_corners;
        for (size_t i = 0; i < sample.object_points.size(); ++i)
            sample.object_points[i] = cv::Point3f(static_cast<float>(i), 0, 0);
        if (StereoCalibrator::reproject_grid(sample, charuco_cfg, truth, projection, err)) {
            std::cerr << "Grid validation accepted collinear points" << std::endl;
            return 1;
        }
    }
    GridReprojection uncalibrated_projection;
    if (StereoCalibrator::reproject_grid(calibrator.get_frames()[0], charuco_cfg,
                                         StereoCalibrationResult(), uncalibrated_projection, err)) {
        std::cerr << "Grid validation accepted missing calibration" << std::endl;
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

    // Raw archive preserves pixels, pairing IDs, and exact YAML bytes.
    auto archive_frames = calibrator.get_frames();
    archive_frames[0].left_image = cv::Mat(img_size, CV_8UC3, cv::Scalar(17, 39, 212));
    archive_frames[0].right_image = cv::Mat(img_size, CV_8UC3, cv::Scalar(78, 123, 4));
    std::string archive_dir, archive_error;
    if (!CalibrationIO::save_calibration_archive(tmp_dir, archive_frames, charuco_cfg,
                                                 archive_dir, archive_error)) {
        std::cerr << "Could not save raw archive: " << archive_error << std::endl;
        return 1;
    }
    auto read_file = [](const std::filesystem::path &path) {
        std::ifstream file(path, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    };
    for (const char *file : {"left.yaml", "right.yaml", "stereo_calibration.yaml"}) {
        const auto contents = read_file(std::filesystem::path(tmp_dir) / file);
        if (contents.find("calibration_date:") == std::string::npos ||
            contents.find(result.calibration_date) == std::string::npos) {
            std::cerr << "Calibration YAML is missing the original calibration date: " << file << std::endl;
            return 1;
        }
    }
    for (const char *file : {"left.yaml", "right.yaml", "stereo_calibration.yaml"}) {
        if (read_file(std::filesystem::path(tmp_dir) / file).empty() ||
            read_file(std::filesystem::path(tmp_dir) / file) !=
            read_file(std::filesystem::path(archive_dir) / file)) {
            std::cerr << "Archived YAML does not match exported YAML" << std::endl;
            return 1;
        }
    }
    for (const auto &frame : archive_frames) {
        std::ostringstream name;
        name << "sample_" << std::setw(6) << std::setfill('0') << frame.id << ".png";
        for (const auto &eye : {"left", "right"}) {
            auto image = cv::imread((std::filesystem::path(archive_dir) / eye / name.str()).string());
            const auto &original = std::string(eye) == "left" ? frame.left_image : frame.right_image;
            if (image.size() != original.size() || image.type() != original.type() ||
                cv::norm(image, original, cv::NORM_INF) != 0) {
                std::cerr << "Raw image pixels changed during archive" << std::endl;
                return 1;
            }
        }
    }
    CharucoConfig archived_board;
    if (!load_board_config((std::filesystem::path(archive_dir) / "board.json").string(),
                            archived_board, archive_error) ||
        archived_board.squares_x != charuco_cfg.squares_x ||
        archived_board.square_length_mm != charuco_cfg.square_length_mm) {
        std::cerr << "Archived board configuration mismatch" << std::endl;
        return 1;
    }
    std::string second_archive;
    if (!CalibrationIO::save_calibration_archive(tmp_dir, archive_frames, charuco_cfg,
                                                 second_archive, archive_error) || second_archive == archive_dir) {
        std::cerr << "Repeated save overwrote an archive" << std::endl;
        return 1;
    }
    const auto bad_parent = std::filesystem::path(tmp_dir) / "missing_yaml";
    std::filesystem::create_directory(bad_parent);
    std::string failed_archive;
    if (CalibrationIO::save_calibration_archive(bad_parent.string(), archive_frames, charuco_cfg,
                                                failed_archive, archive_error) ||
        archive_error.empty() || !failed_archive.empty() || !std::filesystem::is_empty(bad_parent)) {
        std::cerr << "Failed archive left incomplete output or reported success" << std::endl;
        return 1;
    }
    if (CalibrationIO::save_calibration_archive(tmp_dir, {}, charuco_cfg, failed_archive, archive_error)) {
        std::cerr << "Empty calibration set was accepted for archive" << std::endl;
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
    if (loaded_result.calibration_date != result.calibration_date) {
        std::cerr << "Calibration date changed during save/load" << std::endl;
        return 1;
    }
    {
        auto undated = read_file(std::filesystem::path(tmp_dir) / "stereo_calibration.yaml");
        const auto start = undated.find("calibration_date:");
        undated.erase(start, undated.find('\n', start) - start + 1);
        const auto path = std::filesystem::path(tmp_dir) / "missing_date.yaml";
        std::ofstream(path) << undated;
        StereoCalibrationResult missing_date;
        if (CalibrationIO::load_stereo_calibration_file(path.string(), missing_date)) {
            std::cerr << "Calibration YAML without calibration_date must be rejected" << std::endl;
            return 1;
        }
        auto missing_date_result = result;
        missing_date_result.calibration_date.clear();
        if (CalibrationIO::save_ros_camera_info_pair(tmp_dir + "/missing_date", missing_date_result) ||
            CalibrationIO::save_stereo_calibration_file((std::filesystem::path(tmp_dir) / "missing_date_export.yaml").string(), missing_date_result)) {
            std::cerr << "Calibration without a date must not be exported" << std::endl;
            return 1;
        }
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
        fs_inc << "calibration_date" << loaded_result.calibration_date;
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
