// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#include <dvrk_data/calibration/rectification.hpp>
#include <dvrk_data/calibration/calibration_io.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace dvrk_data::calibration {
bool Rectification::load(const std::string &path, std::string &error) {
    m_calibration = StereoCalibrationResult();
    try {
        cv::FileStorage file(path, cv::FileStorage::READ);
        std::string type, model;
        if (!file.isOpened()) throw std::runtime_error("Cannot open calibration file: " + path);
        file["type"] >> type;
        file["distortion_model"] >> model;
        if (type != "dvrk_calibration:stereo@1.0.0")
            throw std::runtime_error("Expected a dvrk_calibration:stereo@1.0.0 calibration");
        if (model != "plumb_bob" && model != "rational_polynomial")
            throw std::runtime_error("Unsupported distortion model: " + model);
        StereoCalibrationResult result;
        if (!CalibrationIO::load_stereo_calibration_file(path, result))
            throw std::runtime_error("Invalid stereo calibration: " + path);
        const auto size = result.image_size;
        if (size.width <= 0 || size.height <= 0 || (size.width % 2) || (size.height % 2))
            throw std::runtime_error("Calibration requires positive even per-eye dimensions for I420 output");
        for (const auto &matrix : {result.K1, result.K2, result.D1, result.D2,
                                   result.R, result.T, result.R1, result.R2,
                                   result.P1, result.P2, result.map1_l, result.map2_l,
                                   result.map1_r, result.map2_r}) {
            if (matrix.empty() || !cv::checkRange(matrix))
                throw std::runtime_error("Calibration contains empty or non-finite matrices");
        }
        auto valid_distortion = [&](size_t count) {
            return model == "plumb_bob" ? count == 5 : (count == 8 || count == 14);
        };
        if (!valid_distortion(result.D1.total()) || !valid_distortion(result.D2.total()) || cv::norm(result.T) < 1e-9)
            throw std::runtime_error("Invalid distortion coefficients or zero stereo baseline");
        for (const auto &matrix : {result.K1, result.K2, result.P1, result.P2}) {
            cv::Mat camera;
            matrix.convertTo(camera, CV_64F);
            if (camera.at<double>(0, 0) <= 0 || camera.at<double>(1, 1) <= 0)
                throw std::runtime_error("Calibration focal lengths must be positive");
        }
        m_calibration = std::move(result);
        error.clear();
        return true;
    } catch (const std::exception &e) {
        error = e.what();
        return false;
    }
}
void Rectification::remap(const cv::Mat &input, cv::Mat &output, bool right_eye) const {
    if (!m_calibration.success) throw std::runtime_error("Rectification has no loaded calibration");
    if (input.size() != image_size() || input.type() != CV_8UC3)
        throw std::runtime_error("Rectification input must be BGR at the calibrated per-eye dimensions");
    cv::remap(input, output,
              right_eye ? m_calibration.map1_r : m_calibration.map1_l,
              right_eye ? m_calibration.map2_r : m_calibration.map2_l,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}
} // namespace dvrk_data::calibration
