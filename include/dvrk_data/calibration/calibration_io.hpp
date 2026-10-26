// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#ifndef DVRK_DATA_CALIBRATION_CALIBRATION_IO_HPP
#define DVRK_DATA_CALIBRATION_CALIBRATION_IO_HPP

#include "dvrk_data/calibration/stereo_calibrator.hpp"
#include <string>

namespace dvrk_data::calibration {

class CalibrationIO {
public:
    /// Save ROS2 standard CameraInfo YAML files (left.yaml and right.yaml)
    static bool save_ros_camera_info_pair(const std::string &output_dir,
                                          const StereoCalibrationResult &result,
                                          const std::string &left_name = "left",
                                          const std::string &right_name = "right");

    /// Save full stereo calibration to dVRK JSON / YAML format
    static bool save_stereo_calibration_file(const std::string &filepath,
                                            const StereoCalibrationResult &result);

    /// Load full stereo calibration from file (YAML or JSON)
    static bool load_stereo_calibration_file(const std::string &filepath,
                                            StereoCalibrationResult &result);
};

} // namespace dvrk_data::calibration

#endif // DVRK_DATA_CALIBRATION_CALIBRATION_IO_HPP
