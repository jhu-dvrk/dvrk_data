// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#ifndef DVRK_DATA_CALIBRATION_BOARD_CONFIG_HPP
#define DVRK_DATA_CALIBRATION_BOARD_CONFIG_HPP

#include "dvrk_data/calibration/charuco_detector.hpp"
#include <string>

namespace dvrk_data::calibration {

/// Convert predefined dictionary integer ID to canonical string name
std::string dictionary_to_string(int dict_id);

/// Convert canonical string name (e.g. "DICT_4X4_50") to dictionary integer ID
int string_to_dictionary(const std::string &dict_name);

/// Load ChArUco board settings from a JSON file
bool load_board_config(const std::string &filepath,
                       CharucoConfig &config,
                       std::string &error_msg);

/// Save ChArUco board settings to a JSON file
bool save_board_config(const std::string &filepath,
                       const CharucoConfig &config,
                       std::string &error_msg);

} // namespace dvrk_data::calibration

#endif // DVRK_DATA_CALIBRATION_BOARD_CONFIG_HPP
