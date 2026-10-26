// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "calibration_window.hpp"
#include "dvrk_data/calibration/board_config.hpp"
#include <dvrk_data/dvrk_gst_socket.hpp>

#include <gtkmm.h>
#include <iostream>
#include <string>

void print_usage(const char *prog) {
    std::cout << "Usage: " << prog << " -c <board_config.json> [options]\n"
              << "Required:\n"
              << "  -c, --config <file.json>       Path to ChArUco board configuration JSON\n"
              << "Options:\n"
              << "  -i, --input <socket/pipe>     Stereo side-by-side input (default: @dvrk:stereo_source:stereo)\n"
              << "  -t, --testsrc                  Use videotestsrc instead of sockets (offline test mode)\n"
              << "  --list-sockets                 List active @dvrk abstract sockets and exit\n"
              << "  -h, --help                     Show this help message\n\n"
              << "Generate a board and matching JSON using:\n"
              << "  ros2 run dvrk_data generate_charuco_board.py -x 8 -y 6 -s 5.0 -m 3.5 -o my_board.pdf\n";
}

int main(int argc, char *argv[]) {
    std::string board_file;
    dvrk_data::calibration::ReceiverConfig receiver_cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--list-sockets") {
            dvrk_gst::print_sockets();
            return 0;
        } else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            board_file = argv[++i];
        } else if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
            receiver_cfg.stereo_input = argv[++i];
        } else if (arg == "-t" || arg == "--testsrc") {
            receiver_cfg.use_testsrc = true;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    if (board_file.empty()) {
        std::cerr << "Error: A board configuration JSON file is required.\n\n";
        print_usage(argv[0]);
        return 1;
    }

    dvrk_data::calibration::CharucoConfig charuco_cfg;
    std::string err_msg;
    if (!dvrk_data::calibration::load_board_config(board_file, charuco_cfg, err_msg)) {
        std::cerr << "Error loading board configuration: " << err_msg << std::endl;
        return 1;
    }

    std::cout << "Loaded board config from: " << board_file << "\n"
              << "  Grid: " << charuco_cfg.squares_x << " x " << charuco_cfg.squares_y << "\n"
              << "  Square size: " << charuco_cfg.square_length_mm << " mm\n"
              << "  Marker size: " << charuco_cfg.marker_length_mm << " mm\n"
              << "  Dictionary: " << dvrk_data::calibration::dictionary_to_string(charuco_cfg.dictionary_id) << "\n"
              << "Starting dVRK Calibrator GUI..." << std::endl;

    auto app = Gtk::Application::create("org.dvrk.stereo_calibration");
    dvrk_data::calibration::CalibrationWindow window(board_file, receiver_cfg, charuco_cfg);

    return app->run(window);
}
