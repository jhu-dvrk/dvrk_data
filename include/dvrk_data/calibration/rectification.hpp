// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#ifndef DVRK_DATA_CALIBRATION_RECTIFICATION_HPP
#define DVRK_DATA_CALIBRATION_RECTIFICATION_HPP
#include <dvrk_data/calibration/stereo_calibrator.hpp>
#include <string>

namespace dvrk_data::calibration {
// Strict processing API: incompatible inputs are errors, never raw passthrough.
// Maps are prepared once and shared read-only by subsequent frames.
class Rectification {
public:
    bool load(const std::string &path, std::string &error);
    cv::Size image_size() const { return m_calibration.image_size; }
    void remap(const cv::Mat &input, cv::Mat &output, bool right_eye) const;
private:
    StereoCalibrationResult m_calibration;
};
} // namespace dvrk_data::calibration
#endif
