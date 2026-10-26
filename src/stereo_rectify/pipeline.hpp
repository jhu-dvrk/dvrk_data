// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#ifndef DVRK_DATA_RECTIFY_PIPELINE_HPP
#define DVRK_DATA_RECTIFY_PIPELINE_HPP
#include <dvrk_data/config.hpp>
#include <opencv2/core.hpp>
#include <string>

struct RectifyConfig {
    dvrk_data::AppConfig streams;
    std::string calibration_file;
};
RectifyConfig load_rectify_config(const std::string &path);
std::string build_rectify_pipeline(const RectifyConfig &config, cv::Size eye_size);
#endif
