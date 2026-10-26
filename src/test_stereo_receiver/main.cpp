// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/gst_stereo_receiver.hpp"
#include <dvrk_data/dvrk_gst_socket.hpp>
#include <dvrk_data/video_caps.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <opencv2/imgcodecs.hpp>

void print_usage(const char *prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  -i, --input <socket/pipeline>  Stereo side-by-side input (default: @dvrk:stereo_source:stereo)\n"
              << "  -t, --testsrc                   Use videotestsrc instead of sockets\n"
              << "  -n, --frames <count>            Number of frame pairs to receive (default: 30)\n"
              << "  --save                          Save first received frame pair to disk\n"
              << "  --list-sockets                  List active @dvrk abstract sockets and exit\n"
              << "  -h, --help                      Show this help message\n";
}

int main(int argc, char *argv[]) {
    dvrk_data::calibration::ReceiverConfig config;
    int target_frames = 30;
    bool save_first_pair = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--list-sockets") {
            dvrk_gst::print_sockets();
            return 0;
        } else if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
            config.stereo_input = argv[++i];
        } else if (arg == "-t" || arg == "--testsrc") {
            config.use_testsrc = true;
        } else if ((arg == "-n" || arg == "--frames") && i + 1 < argc) {
            target_frames = std::stoi(argv[++i]);
        } else if (arg == "--save") {
            save_first_pair = true;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    std::cout << "Starting GstStereoReceiver..." << std::endl;
    std::cout << "  Mode:   Stereo Side-by-Side (from dvrk_data stereo_source)" << std::endl;
    std::cout << "  Source: " << (config.use_testsrc ? "videotestsrc" : config.stereo_input) << std::endl;

    dvrk_data::calibration::GstStereoReceiver receiver;
    if (!receiver.start(config)) {
        std::cerr << "Failed to start GstStereoReceiver." << std::endl;
        return 1;
    }

    std::cout << "Waiting for stereo frame pairs (target: " << target_frames << " frames)..." << std::endl;

    cv::Mat left, right;
    uint64_t ts = 0;
    int received_count = 0;
    bool saved = false;

    auto start_time = std::chrono::steady_clock::now();

    while (received_count < target_frames) {
        if (receiver.get_latest_frame_pair(left, right, ts, 500)) {
            received_count++;
            if (!saved && save_first_pair) {
                cv::imwrite("test_left.png", left);
                cv::imwrite("test_right.png", right);
                std::cout << "Saved test_left.png and test_right.png ("
                          << left.cols << "x" << left.rows << " per eye)" << std::endl;
                saved = true;
            }

            if (received_count % 10 == 0 || received_count == 1) {
                auto stats = receiver.get_stats();
                std::cout << "Frame pair #" << received_count
                          << " | Single eye: " << left.cols << "x" << left.rows
                          << " | fps: " << stats.fps
                          << " | TS: " << ts << std::endl;
            }
        } else {
            std::cout << "Waiting for frames..." << std::endl;
        }
    }

    auto total_time_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    std::cout << "\nSuccessfully received " << received_count << " frame pairs in "
              << total_time_sec << " seconds (" << (received_count / total_time_sec) << " fps)." << std::endl;

    receiver.stop();
    return 0;
}
