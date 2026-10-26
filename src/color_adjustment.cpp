// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#include <dvrk_data/color_adjustment.hpp>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace dvrk_data {
std::string load_stereo_color_adjustment(AppConfig &config,
    const std::string &processor_config, const std::string &explicit_file) {
    auto nonneutral = [](const ColorAdjustment &color) {
        return color.brightness != 0 || color.contrast != 1 || color.saturation != 1 || color.hue != 0;
    };
    if (nonneutral(config.left_color) || nonneutral(config.right_color))
        std::cerr << "Inline camera color settings are ignored; move them to stereo_color_adjustment.json.\n";
    config.left_color = ColorAdjustment();
    config.right_color = ColorAdjustment();
    const auto path = explicit_file.empty()
        ? std::filesystem::absolute(processor_config).parent_path() / "stereo_color_adjustment.json"
        : std::filesystem::absolute(explicit_file);
    if (explicit_file.empty() && !std::filesystem::exists(path)) return "";
    Json::Value root;
    if (!Config::load_from_file(path.string(), root) ||
        !Config::check_type(root, "dvrk_data:stereo_color_adjustment@1.0.0", path.string()))
        throw std::runtime_error("Cannot load stereo color adjustment: " + path.string());
    auto parse = [](const Json::Value &eye) {
        if (!eye.isObject()) throw std::runtime_error("Color adjustment requires left and right objects");
        auto parameter = [&](const char *key, double fallback, double low, double high) {
            if (eye.isMember(key) && !eye[key].isNumeric())
                throw std::runtime_error(std::string("Color parameter must be numeric: ") + key);
            double value = eye.get(key, fallback).asDouble();
            if (!std::isfinite(value) || value < low || value > high)
                throw std::runtime_error(std::string("Color parameter outside videobalance range: ") + key);
            return value;
        };
        ColorAdjustment value;
        value.brightness = parameter("brightness", 0, -1, 1);
        value.contrast = parameter("contrast", 1, 0, 2);
        value.saturation = parameter("saturation", 1, 0, 2);
        value.hue = parameter("hue", 0, -1, 1);
        return value;
    };
    config.left_color = parse(root["left"]);
    config.right_color = parse(root["right"]);
    return path.lexically_normal().string();
}
}
