#pragma once
#include <dvrk_data/calibration/calibration_io.hpp>
#include <dvrk_data/calibration/board_config.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

struct CalibrationArchiveFixture {
    std::filesystem::path root;
    std::string archive;
    dvrk_data::calibration::CharucoConfig board;
    cv::Mat image;
    CalibrationArchiveFixture() {
        char pattern[] = "/tmp/dvrk-archive-test-XXXXXX";
        const auto *created = mkdtemp(pattern);
        if (!created) throw std::runtime_error("Cannot create calibration test directory");
        root = created;
        for (const char *name : {"left.yaml", "right.yaml", "stereo_calibration.yaml"})
            std::ofstream(root / name) << "unused by raw image reload\n";
        dvrk_data::calibration::CharucoDetector detector(board);
        cv::Mat gray;
        detector.get_board()->draw({640, 480}, gray, 24);
        cv::cvtColor(gray, image, cv::COLOR_GRAY2BGR);
        std::vector<dvrk_data::calibration::CalibrationFrame> frames(2);
        frames[0].id = 2; frames[1].id = 10;
        for (auto &frame : frames) frame.left_image = frame.right_image = image;
        std::string error;
        if (!dvrk_data::calibration::CalibrationIO::save_calibration_archive(
                root.string(), frames, board, archive, error)) throw std::runtime_error(error);
    }
    ~CalibrationArchiveFixture() {
        std::error_code error;
        std::filesystem::remove_all(root, error);
    }
};
