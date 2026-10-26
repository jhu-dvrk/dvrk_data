// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#ifndef DVRK_DATA_CALIBRATION_GST_STEREO_RECEIVER_HPP
#define DVRK_DATA_CALIBRATION_GST_STEREO_RECEIVER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/core.hpp>

// Forward declarations for GStreamer
typedef struct _GstElement GstElement;
typedef struct _GstSample GstSample;
typedef struct _GstBus GstBus;

namespace dvrk_data::calibration {

struct ReceiverConfig {
    std::string stereo_input = "@dvrk:stereo_source:stereo";
    int width = 0;       // 0 for auto
    int height = 0;      // 0 for auto
    int framerate = 0;   // 0 for auto
    bool use_testsrc = false; // Fallback to videotestsrc if sockets are unavailable
};

struct StreamStats {
    bool is_connected = false;
    std::string status_message = "Disconnected";
    uint64_t frames_received = 0;
    uint64_t pairs_delivered = 0;
    double fps = 0.0;
    int single_eye_width = 0;
    int single_eye_height = 0;
};

class GstStereoReceiver {
public:
    using FramePairCallback = std::function<void(const cv::Mat &left, const cv::Mat &right, uint64_t timestamp_ns)>;

    GstStereoReceiver();
    ~GstStereoReceiver();

    // Disable copy
    GstStereoReceiver(const GstStereoReceiver &) = delete;
    GstStereoReceiver &operator=(const GstStereoReceiver &) = delete;

    /// Start streaming pipeline(s) with automatic background connection/reconnection.
    bool start(const ReceiverConfig &config = ReceiverConfig());

    /// Stop streaming and release pipeline resources.
    void stop();

    /// Returns true if receiver thread is running.
    bool is_running() const;

    /// Returns true if currently connected and receiving frames.
    bool is_connected() const;

    /// Pull latest synchronized frame pair.
    /// @param left Output left frame (BGR, CV_8UC3)
    /// @param right Output right frame (BGR, CV_8UC3)
    /// @param timestamp_ns Frame capture timestamp
    /// @param timeout_ms Max time to wait for a new pair in milliseconds
    /// @return true if a fresh pair was acquired within timeout
    bool get_latest_frame_pair(cv::Mat &left, cv::Mat &right, uint64_t &timestamp_ns, int timeout_ms = 100);

    /// Set an asynchronous callback invoked whenever a new frame pair arrives.
    void set_callback(FramePairCallback callback);

    /// Get streaming diagnostics and frame rates.
    StreamStats get_stats() const;

private:
    struct StreamChannel {
        std::string name;
        std::string pipeline_str;
        GstElement *pipeline = nullptr;
        GstElement *appsink = nullptr;
        GstBus *bus = nullptr;
        std::atomic<uint64_t> frame_count{0};
        cv::Mat latest_frame;
        uint64_t latest_pts = 0;
        bool has_frame = false;
        std::mutex frame_mutex;
    };

    bool create_channel(StreamChannel &channel, const std::string &input_source, int width, int height, int framerate, bool use_testsrc);
    void destroy_channel(StreamChannel &channel);
    bool check_channel_bus(StreamChannel &channel);
    void worker_loop_side_by_side();

    ReceiverConfig m_config;
    StreamChannel m_stereo_channel; // Side-by-side stream

    std::atomic<bool> m_running{false};
    std::atomic<bool> m_connected{false};
    std::thread m_worker_thread;

    mutable std::mutex m_pair_mutex;
    std::condition_variable m_pair_cv;
    cv::Mat m_latest_left;
    cv::Mat m_latest_right;
    uint64_t m_latest_timestamp_ns = 0;
    bool m_new_pair_ready = false;

    FramePairCallback m_callback;
    mutable std::mutex m_callback_mutex;

    // Stats
    mutable std::mutex m_stats_mutex;
    mutable std::chrono::steady_clock::time_point m_last_stats_time;
    mutable std::chrono::steady_clock::time_point m_last_frame_time;
    mutable StreamStats m_stats;
};

} // namespace dvrk_data::calibration

#endif // DVRK_DATA_CALIBRATION_GST_STEREO_RECEIVER_HPP
