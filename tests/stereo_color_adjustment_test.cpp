// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#include <dvrk_data/color_adjustment.hpp>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

void check(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
int main() {
    char pattern[] = "/tmp/dvrk-color-XXXXXX";
    const char *created = mkdtemp(pattern);
    if (!created) return 1;
    std::filesystem::path directory(created);
    try {
        const auto processor = directory / "stereo_alignment.json";
        const auto color = directory / "stereo_color_adjustment.json";
        dvrk_data::AppConfig cfg;
        cfg.left_color.brightness = 0.25;
        check(dvrk_data::load_stereo_color_adjustment(cfg, processor.string()).empty(), "Unexpected discovered file");
        check(cfg.left_color.brightness == 0 && cfg.right_color.contrast == 1, "No file must mean neutral correction");
        Json::Value root;
        root["type"] = "dvrk_data:stereo_color_adjustment@1.0.0";
        root["left"]["brightness"] = 0.1;
        root["right"]["contrast"] = 0.9;
        auto save = [&](const std::filesystem::path &path) { std::ofstream out(path); out << root; };
        save(color);
        check(dvrk_data::load_stereo_color_adjustment(cfg, processor.string()) == color.string(), "Discovery must use processor config directory");
        check(cfg.left_color.brightness == 0.1 && cfg.right_color.contrast == 0.9, "Color parameters were not loaded");
        const auto alternate = directory / "alternate.json";
        root["left"]["brightness"] = -0.1; save(alternate);
        dvrk_data::load_stereo_color_adjustment(cfg, processor.string(), alternate.string());
        check(cfg.left_color.brightness == -0.1, "Explicit color file must override discovery");
        auto rejected = [&](const std::filesystem::path &path) {
            bool failed = false;
            try { dvrk_data::load_stereo_color_adjustment(cfg, processor.string(), path.string()); }
            catch (const std::exception &) { failed = true; }
            check(failed, "Invalid/explicitly missing color file was accepted");
        };
        rejected(directory / "missing.json");
        root["right"]["contrast"] = 9; save(alternate); rejected(alternate);
        root["right"]["contrast"] = "bad"; save(alternate); rejected(alternate);
        root["type"] = "wrong"; save(alternate); rejected(alternate);
        std::filesystem::remove_all(directory);
        std::cout << "Shared stereo color discovery, neutral defaults and validation PASSED\n";
        return 0;
    } catch (const std::exception &error) {
        std::filesystem::remove_all(directory);
        std::cerr << error.what() << '\n'; return 1;
    }
}
