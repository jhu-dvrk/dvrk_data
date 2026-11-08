// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#ifndef DVRK_DATA_CALIBRATION_LENS_DISTORTION_HPP
#define DVRK_DATA_CALIBRATION_LENS_DISTORTION_HPP

#include <array>
#include <string>
#include <opencv2/core.hpp>

namespace dvrk_data::calibration {

struct StereoCalibrationResult;

struct LensDistortionView {
    cv::Mat image;
    std::array<double, 2> max_displacement_px{};
    std::array<int, 2> invalid_pixels{};
    double color_max_px = 0.0;
};

/// Ideal pixels -> distorted sensor pixels, keeping each camera's K unchanged.
/// Draw both eyes with one displacement scale; stereo R/T and rectification are unused.
bool render_lens_distortion(const StereoCalibrationResult &calibration,
                            LensDistortionView &output, std::string &error);

} // namespace dvrk_data::calibration

#endif
