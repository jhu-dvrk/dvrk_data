// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/board_config.hpp"

#include <json/json.h>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace dvrk_data::calibration {

std::string dictionary_to_string(int dict_id) {
    switch (dict_id) {
        case cv::aruco::DICT_4X4_50: return "DICT_4X4_50";
        case cv::aruco::DICT_4X4_100: return "DICT_4X4_100";
        case cv::aruco::DICT_4X4_250: return "DICT_4X4_250";
        case cv::aruco::DICT_5X5_50: return "DICT_5X5_50";
        case cv::aruco::DICT_5X5_100: return "DICT_5X5_100";
        case cv::aruco::DICT_5X5_250: return "DICT_5X5_250";
        case cv::aruco::DICT_6X6_50: return "DICT_6X6_50";
        case cv::aruco::DICT_6X6_100: return "DICT_6X6_100";
        case cv::aruco::DICT_6X6_250: return "DICT_6X6_250";
        default: return "DICT_4X4_50";
    }
}

int string_to_dictionary(const std::string &dict_name) {
    if (dict_name == "DICT_4X4_50") return cv::aruco::DICT_4X4_50;
    if (dict_name == "DICT_4X4_100") return cv::aruco::DICT_4X4_100;
    if (dict_name == "DICT_4X4_250") return cv::aruco::DICT_4X4_250;
    if (dict_name == "DICT_5X5_50") return cv::aruco::DICT_5X5_50;
    if (dict_name == "DICT_5X5_100") return cv::aruco::DICT_5X5_100;
    if (dict_name == "DICT_5X5_250") return cv::aruco::DICT_5X5_250;
    if (dict_name == "DICT_6X6_50") return cv::aruco::DICT_6X6_50;
    if (dict_name == "DICT_6X6_100") return cv::aruco::DICT_6X6_100;
    if (dict_name == "DICT_6X6_250") return cv::aruco::DICT_6X6_250;
    return cv::aruco::DICT_4X4_50;
}

bool load_board_config(const std::string &filepath,
                       CharucoConfig &config,
                       std::string &error_msg) {
    if (!std::filesystem::exists(filepath)) {
        error_msg = "Board configuration file does not exist: " + filepath;
        return false;
    }

    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        error_msg = "Unable to open board configuration file: " + filepath;
        return false;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errs;
    if (!Json::parseFromStream(builder, ifs, &root, &errs)) {
        error_msg = "Failed to parse JSON: " + errs;
        return false;
    }

    if (root.isMember("squares_x")) {
        config.squares_x = root["squares_x"].asInt();
    }
    if (root.isMember("squares_y")) {
        config.squares_y = root["squares_y"].asInt();
    }
    if (root.isMember("square_length_mm")) {
        config.square_length_mm = root["square_length_mm"].asFloat();
    }
    if (root.isMember("marker_length_mm")) {
        config.marker_length_mm = root["marker_length_mm"].asFloat();
    }
    if (root.isMember("dictionary")) {
        config.dictionary_id = string_to_dictionary(root["dictionary"].asString());
    }
    if (root.isMember("min_corners")) {
        config.min_corners = std::max(6, root["min_corners"].asInt());
    } else {
        config.min_corners = std::max(6, config.min_corners);
    }

    if (config.squares_x <= 2 || config.squares_y <= 2) {
        error_msg = "Grid must have at least 3 squares along each axis.";
        return false;
    }
    if (config.marker_length_mm >= config.square_length_mm) {
        error_msg = "Marker length must be strictly smaller than square length.";
        return false;
    }

    return true;
}

bool save_board_config(const std::string &filepath,
                       const CharucoConfig &config,
                       std::string &error_msg) {
    auto parent = std::filesystem::path(filepath).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }

    Json::Value root;
    root["type"] = "dvrk_calibration:charuco_board@1.0.0";
    root["squares_x"] = config.squares_x;
    root["squares_y"] = config.squares_y;
    root["square_length_mm"] = config.square_length_mm;
    root["marker_length_mm"] = config.marker_length_mm;
    root["dictionary"] = dictionary_to_string(config.dictionary_id);

    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        error_msg = "Failed to open file for writing: " + filepath;
        return false;
    }

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &ofs);

    return true;
}

} // namespace dvrk_data::calibration
