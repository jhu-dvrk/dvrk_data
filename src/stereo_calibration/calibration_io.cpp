// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/calibration_io.hpp"

#include <opencv2/calib3d.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace dvrk_data::calibration {

namespace {

void write_mat_data(std::ostream &os, const cv::Mat &mat) {
    os << "[";
    for (int r = 0; r < mat.rows; ++r) {
        for (int c = 0; c < mat.cols; ++c) {
            if (r > 0 || c > 0) os << ", ";
            os << std::setprecision(8) << mat.at<double>(r, c);
        }
    }
    os << "]\n";
}

bool write_ros_yaml(const std::string &filepath,
                    const std::string &camera_name,
                    const cv::Size &size,
                    const cv::Mat &K,
                    const cv::Mat &D,
                    const cv::Mat &R,
                    const cv::Mat &P,
                    DistortionModel model) {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) return false;

    ofs << "# ROS 2 CameraInfo configuration\n";
    ofs << "image_width: " << size.width << "\n";
    ofs << "image_height: " << size.height << "\n";
    ofs << "camera_name: " << camera_name << "\n";

    ofs << "camera_matrix:\n";
    ofs << "  rows: 3\n  cols: 3\n  data: ";
    write_mat_data(ofs, K);

    ofs << "distortion_model: "
        << (model == DistortionModel::RATIONAL_POLYNOMIAL ? "rational_polynomial" : "plumb_bob")
        << "\n";
    ofs << "distortion_coefficients:\n";
    ofs << "  rows: 1\n  cols: " << (D.total()) << "\n  data: ";
    write_mat_data(ofs, D.reshape(1, 1));

    ofs << "rectification_matrix:\n";
    ofs << "  rows: 3\n  cols: 3\n  data: ";
    write_mat_data(ofs, R);

    ofs << "projection_matrix:\n";
    ofs << "  rows: 3\n  cols: 4\n  data: ";
    write_mat_data(ofs, P);

    return true;
}

} // anonymous namespace

bool CalibrationIO::save_ros_camera_info_pair(const std::string &output_dir,
                                              const StereoCalibrationResult &result,
                                              const std::string &left_name,
                                              const std::string &right_name) {
    if (!result.success) return false;

    std::filesystem::create_directories(output_dir);
    std::string left_path = (std::filesystem::path(output_dir) / (left_name + ".yaml")).string();
    std::string right_path = (std::filesystem::path(output_dir) / (right_name + ".yaml")).string();

    // Convert translation column from millimeters to meters for ROS CameraInfo standard
    // (Tx = -fx' * B with baseline B in meters)
    cv::Mat P1_ros = result.P1.clone();
    cv::Mat P2_ros = result.P2.clone();
    if (!P1_ros.empty() && P1_ros.cols == 4) {
        P1_ros.col(3) /= 1000.0;
    }
    if (!P2_ros.empty() && P2_ros.cols == 4) {
        P2_ros.col(3) /= 1000.0;
    }

    bool ok_l = write_ros_yaml(left_path, left_name, result.image_size,
                               result.K1, result.D1, result.R1, P1_ros, result.model);
    bool ok_r = write_ros_yaml(right_path, right_name, result.image_size,
                               result.K2, result.D2, result.R2, P2_ros, result.model);

    return ok_l && ok_r;
}

bool CalibrationIO::save_stereo_calibration_file(const std::string &filepath,
                                                const StereoCalibrationResult &result) {
    if (!result.success) return false;

    try {
        auto parent = std::filesystem::path(filepath).parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }
        cv::FileStorage fs(filepath, cv::FileStorage::WRITE);
        if (!fs.isOpened()) return false;

        fs << "type" << "dvrk_calibration:stereo@1.0.0";
        fs << "distortion_model" << (result.model == DistortionModel::RATIONAL_POLYNOMIAL ? "rational_polynomial" : "plumb_bob");
        fs << "image_width" << result.image_size.width;
        fs << "image_height" << result.image_size.height;
        fs << "rms_error" << result.rms_error;
        fs << "baseline_mm" << result.baseline_mm;
        fs << "avg_epipolar_error_px" << result.avg_epipolar_error_px;
        fs << "max_epipolar_error_px" << result.max_epipolar_error_px;

        // Matrices
        fs << "K1" << result.K1;
        fs << "D1" << result.D1;
        fs << "K2" << result.K2;
        fs << "D2" << result.D2;
        fs << "R" << result.R;
        fs << "T" << result.T;
        fs << "E" << result.E;
        fs << "F" << result.F;
        fs << "R1" << result.R1;
        fs << "R2" << result.R2;
        fs << "P1" << result.P1;
        fs << "P2" << result.P2;
        fs << "Q" << result.Q;

        fs.release();
        return true;
    } catch (const cv::Exception &e) {
        std::cerr << "Error writing calibration file: " << e.what() << std::endl;
        return false;
    }
}

bool CalibrationIO::load_stereo_calibration_file(const std::string &filepath,
                                                StereoCalibrationResult &result) {
    try {
        cv::FileStorage fs(filepath, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;

        int w = 0, h = 0;
        fs["image_width"] >> w;
        fs["image_height"] >> h;
        result.image_size = cv::Size(w, h);

        std::string model_str;
        fs["distortion_model"] >> model_str;
        result.model = (model_str == "rational_polynomial")
            ? DistortionModel::RATIONAL_POLYNOMIAL : DistortionModel::PLUMB_BOB;

        fs["rms_error"] >> result.rms_error;
        fs["baseline_mm"] >> result.baseline_mm;
        fs["avg_epipolar_error_px"] >> result.avg_epipolar_error_px;
        fs["max_epipolar_error_px"] >> result.max_epipolar_error_px;

        fs["K1"] >> result.K1;
        fs["D1"] >> result.D1;
        fs["K2"] >> result.K2;
        fs["D2"] >> result.D2;
        fs["R"] >> result.R;
        fs["T"] >> result.T;
        fs["E"] >> result.E;
        fs["F"] >> result.F;
        fs["R1"] >> result.R1;
        fs["R2"] >> result.R2;
        fs["P1"] >> result.P1;
        fs["P2"] >> result.P2;
        fs["Q"] >> result.Q;

        fs.release();

        // Validate required calibration and rectification matrices for both cameras
        bool valid_matrices =
            result.image_size.width > 0 && result.image_size.height > 0 &&
            !result.K1.empty() && result.K1.size() == cv::Size(3, 3) &&
            !result.K2.empty() && result.K2.size() == cv::Size(3, 3) &&
            !result.D1.empty() && !result.D2.empty() &&
            !result.R1.empty() && result.R1.size() == cv::Size(3, 3) &&
            !result.R2.empty() && result.R2.size() == cv::Size(3, 3) &&
            !result.P1.empty() && result.P1.rows == 3 && result.P1.cols == 4 &&
            !result.P2.empty() && result.P2.rows == 3 && result.P2.cols == 4 &&
            !result.R.empty() && result.R.size() == cv::Size(3, 3) &&
            !result.T.empty() &&
            ((result.T.rows == 3 && result.T.cols == 1) || (result.T.rows == 1 && result.T.cols == 3));

        if (valid_matrices) {
            cv::initUndistortRectifyMap(result.K1, result.D1, result.R1, result.P1,
                                        result.image_size, CV_32FC1, result.map1_l, result.map2_l);
            cv::initUndistortRectifyMap(result.K2, result.D2, result.R2, result.P2,
                                        result.image_size, CV_32FC1, result.map1_r, result.map2_r);
            result.success = true;
            return true;
        }

        std::cerr << "Error loading calibration file: Missing or invalid matrices for stereo rectification." << std::endl;
        return false;
    } catch (const cv::Exception &e) {
        std::cerr << "Error loading calibration file: " << e.what() << std::endl;
        return false;
    }
}

} // namespace dvrk_data::calibration
