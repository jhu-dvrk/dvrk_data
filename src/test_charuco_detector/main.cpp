// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/charuco_detector.hpp"
#include "dvrk_data/calibration/gst_stereo_receiver.hpp"
#include <dvrk_data/dvrk_gst_socket.hpp>

#include <iostream>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

void print_usage(const char *prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --synthetic                     Run detection on generated synthetic stereo pair (offline test)\n"
              << "  -i, --input <socket/pipe>      Stereo side-by-side input (default: @dvrk:stereo_source:stereo)\n"
              << "  -x, --squares-x <count>         Squares along X (default: 8)\n"
              << "  -y, --squares-y <count>         Squares along Y (default: 6)\n"
              << "  -s, --square-len <mm>           Square length in mm (default: 5.0)\n"
              << "  -m, --marker-len <mm>           Marker length in mm (default: 3.5)\n"
              << "  --save                          Save annotated output images\n"
              << "  -h, --help                      Show this help message\n";
}

int main(int argc, char *argv[]) {
    dvrk_data::calibration::CharucoConfig charuco_cfg;
    dvrk_data::calibration::ReceiverConfig receiver_cfg;
    bool run_synthetic = false;
    bool save_output = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--synthetic") {
            run_synthetic = true;
        } else if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
            receiver_cfg.stereo_input = argv[++i];
        } else if ((arg == "-x" || arg == "--squares-x") && i + 1 < argc) {
            charuco_cfg.squares_x = std::stoi(argv[++i]);
        } else if ((arg == "-y" || arg == "--squares-y") && i + 1 < argc) {
            charuco_cfg.squares_y = std::stoi(argv[++i]);
        } else if ((arg == "-s" || arg == "--square-len") && i + 1 < argc) {
            charuco_cfg.square_length_mm = std::stof(argv[++i]);
        } else if ((arg == "-m" || arg == "--marker-len") && i + 1 < argc) {
            charuco_cfg.marker_length_mm = std::stof(argv[++i]);
        } else if (arg == "--save") {
            save_output = true;
        }
    }

    std::cout << "Initializing CharucoDetector with grid "
              << charuco_cfg.squares_x << "x" << charuco_cfg.squares_y
              << " (sq: " << charuco_cfg.square_length_mm << "mm, marker: "
              << charuco_cfg.marker_length_mm << "mm)..." << std::endl;

    dvrk_data::calibration::CharucoDetector detector(charuco_cfg);

    cv::Mat left_img, right_img;

    if (run_synthetic) {
        std::cout << "Creating synthetic stereo frame pair..." << std::endl;
        cv::Mat board_img;
        auto board = detector.get_board();
        board->draw(cv::Size(640, 480), board_img, 20, 1);

        cv::Mat board_bgr;
        cv::cvtColor(board_img, board_bgr, cv::COLOR_GRAY2BGR);

        // Place on canvas (1280x720) with small disparity shift to simulate stereo endoscope
        left_img = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(180, 180, 180));
        right_img = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(180, 180, 180));

        cv::Rect left_roi(300, 120, 640, 480);
        cv::Rect right_roi(280, 120, 640, 480); // 20px horizontal disparity

        board_bgr.copyTo(left_img(left_roi));
        board_bgr.copyTo(right_img(right_roi));
    } else {
        std::cout << "Connecting to stream (" << receiver_cfg.stereo_input << ")..." << std::endl;
        dvrk_data::calibration::GstStereoReceiver receiver;
        if (!receiver.start(receiver_cfg)) {
            std::cerr << "Failed to connect to stream. Falling back to synthetic test." << std::endl;
            return 1;
        }

        uint64_t ts = 0;
        std::cout << "Waiting for frame pair..." << std::endl;
        if (!receiver.get_latest_frame_pair(left_img, right_img, ts, 2000)) {
            std::cerr << "Timeout waiting for stream frame pair." << std::endl;
            receiver.stop();
            return 1;
        }
        receiver.stop();
    }

    std::cout << "Running detection on frame pair (" << left_img.cols << "x" << left_img.rows << " per eye)..." << std::endl;

    dvrk_data::calibration::StereoDetection stereo_det;
    // Run two consecutive detections to test stationarity logic
    detector.detect_stereo(left_img, right_img, stereo_det);
    detector.detect_stereo(left_img, right_img, stereo_det);

    std::cout << "\n--- Detection Results ---" << std::endl;
    std::cout << "Left Markers:         " << stereo_det.left.marker_ids.size() << std::endl;
    std::cout << "Left ChArUco Corners:  " << stereo_det.left.charuco_corners.size() << std::endl;
    std::cout << "Right Markers:        " << stereo_det.right.marker_ids.size() << std::endl;
    std::cout << "Right ChArUco Corners: " << stereo_det.right.charuco_corners.size() << std::endl;
    std::cout << "Common Stereo Corners: " << stereo_det.common_ids.size() << std::endl;
    std::cout << "Stereo Valid:         " << (stereo_det.is_stereo_valid ? "YES" : "NO") << std::endl;
    std::cout << "Motion Speed:         " << stereo_det.motion_speed << " px" << std::endl;
    std::cout << "Stationary:           " << (stereo_det.is_stationary ? "YES" : "NO") << std::endl;

    if (stereo_det.is_stereo_valid) {
        std::cout << "First 5 common corner IDs: ";
        for (size_t i = 0; i < std::min<size_t>(5, stereo_det.common_ids.size()); ++i) {
            std::cout << stereo_det.common_ids[i] << " ";
        }
        std::cout << "\n";
    }

    if (save_output) {
        cv::Mat left_draw = left_img.clone();
        cv::Mat right_draw = right_img.clone();
        detector.draw_stereo_detection(left_draw, right_draw, stereo_det);
        cv::imwrite("detection_left.png", left_draw);
        cv::imwrite("detection_right.png", right_draw);
        std::cout << "\nSaved annotated images to detection_left.png and detection_right.png." << std::endl;
    }

    return stereo_det.is_stereo_valid ? 0 : 1;
}
