// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/gst_stereo_receiver.hpp"
#include <dvrk_data/dvrk_gst_socket.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <iostream>
#include <sstream>

namespace dvrk_data::calibration {

namespace {

void ensure_gst_init() {
    static bool initialized = false;
    if (!initialized) {
        gst_init(nullptr, nullptr);
        initialized = true;
    }
}

bool extract_mat_from_sample(GstSample *sample, cv::Mat &out_mat, uint64_t &out_pts) {
    if (!sample) return false;

    GstCaps *caps = gst_sample_get_caps(sample);
    if (!caps) return false;

    GstVideoInfo vinfo;
    if (!gst_video_info_from_caps(&vinfo, caps)) {
        return false;
    }

    GstBuffer *buf = gst_sample_get_buffer(sample);
    if (!buf) return false;

    out_pts = GST_BUFFER_PTS_IS_VALID(buf) ? GST_BUFFER_PTS(buf) : 0;

    GstVideoFrame vframe;
    if (!gst_video_frame_map(&vframe, &vinfo, buf, GST_MAP_READ)) {
        return false;
    }

    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&vframe, 0);
    uint8_t *data = static_cast<uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&vframe, 0));

    // Wrap buffer in cv::Mat respecting row stride/padding and clone to own memory
    cv::Mat view(vinfo.height, vinfo.width, CV_8UC3, data, stride);
    out_mat = view.clone();

    gst_video_frame_unmap(&vframe);
    return true;
}

} // anonymous namespace

GstStereoReceiver::GstStereoReceiver() {
    ensure_gst_init();
    m_stereo_channel.name = "stereo";
    m_last_stats_time = std::chrono::steady_clock::now();
}

GstStereoReceiver::~GstStereoReceiver() {
    stop();
}

bool GstStereoReceiver::create_channel(StreamChannel &channel,
                                      const std::string &input_source,
                                      int width, int height, int framerate,
                                      bool use_testsrc) {
    std::ostringstream pipe_ss;

    if (use_testsrc || input_source.find("videotestsrc") != std::string::npos || input_source == "test") {
        pipe_ss << "videotestsrc is-live=true pattern=smpte";
        int test_w = (width > 0) ? width : (channel.name == "stereo" ? 1280 : 640);
        int test_h = (height > 0) ? height : 480;
        pipe_ss << " ! video/x-raw,width=" << test_w << ",height=" << test_h;
        if (framerate > 0) {
            pipe_ss << ",framerate=" << framerate << "/1";
        } else {
            pipe_ss << ",framerate=30/1";
        }
    } else if (dvrk_gst::is_socket_reference(input_source)) {
        pipe_ss << dvrk_gst::build_src(input_source, width, height, "I420");
    } else {
        // Plain pipeline string or device
        pipe_ss << input_source;
        if (width > 0 && height > 0) {
            pipe_ss << " ! video/x-raw,width=" << width << ",height=" << height;
        }
    }

    pipe_ss << " ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink max-buffers=2 drop=true sync=false";
    channel.pipeline_str = pipe_ss.str();

    GError *error = nullptr;
    channel.pipeline = gst_parse_launch(channel.pipeline_str.c_str(), &error);
    if (error != nullptr || channel.pipeline == nullptr) {
        if (error) g_error_free(error);
        return false;
    }

    channel.appsink = gst_bin_get_by_name(GST_BIN(channel.pipeline), "sink");
    if (!channel.appsink) {
        destroy_channel(channel);
        return false;
    }

    channel.bus = gst_element_get_bus(channel.pipeline);

    GstStateChangeReturn ret = gst_element_set_state(channel.pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        destroy_channel(channel);
        return false;
    }

    return true;
}

void GstStereoReceiver::destroy_channel(StreamChannel &channel) {
    if (channel.pipeline) {
        gst_element_set_state(channel.pipeline, GST_STATE_NULL);
        if (channel.bus) {
            gst_bus_set_flushing(channel.bus, TRUE);
            gst_object_unref(channel.bus);
            channel.bus = nullptr;
        }
        if (channel.appsink) {
            gst_object_unref(channel.appsink);
            channel.appsink = nullptr;
        }
        gst_object_unref(channel.pipeline);
        channel.pipeline = nullptr;
    }
    channel.has_frame = false;
}

bool GstStereoReceiver::check_channel_bus(StreamChannel &channel) {
    if (!channel.bus) return false;

    GstMessage *msg = gst_bus_pop_filtered(
        channel.bus,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    if (msg != nullptr) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *err = nullptr;
            gchar *dbg = nullptr;
            gst_message_parse_error(msg, &err, &dbg);
            if (err) g_error_free(err);
            if (dbg) g_free(dbg);
        }
        gst_message_unref(msg);
        return false; // Error or EOS occurred
    }

    return true;
}

bool GstStereoReceiver::start(const ReceiverConfig &config) {
    stop();

    m_config = config;

    // Normalize socket paths if provided without leading '@'
    if (m_config.stereo_input.rfind("dvrk:", 0) == 0) {
        m_config.stereo_input = "@" + m_config.stereo_input;
    }

    m_running = true;
    m_connected = false;
    {
        std::lock_guard<std::mutex> lock(m_stats_mutex);
        m_stats.is_connected = false;
        m_stats.status_message = "Connecting...";
    }
    m_last_stats_time = std::chrono::steady_clock::now();

    m_worker_thread = std::thread(&GstStereoReceiver::worker_loop_side_by_side, this);
    return true;
}

void GstStereoReceiver::stop() {
    m_running = false;
    m_connected = false;
    m_pair_cv.notify_all();

    if (m_worker_thread.joinable()) {
        m_worker_thread.join();
    }

    destroy_channel(m_stereo_channel);
    {
        std::lock_guard<std::mutex> lock(m_stats_mutex);
        m_stats.is_connected = false;
        m_stats.status_message = "Stopped";
    }
}

bool GstStereoReceiver::is_running() const {
    return m_running;
}

bool GstStereoReceiver::is_connected() const {
    return get_stats().is_connected;
}

void GstStereoReceiver::worker_loop_side_by_side() {
    constexpr GstClockTime timeout_ns = 100 * GST_MSECOND;
    int consecutive_timeouts = 0;

    while (m_running) {
        // 1. If pipeline is not active, attempt connection
        if (m_stereo_channel.pipeline == nullptr) {
            m_connected = false;
            {
                std::lock_guard<std::mutex> lock(m_stats_mutex);
                m_stats.is_connected = false;
            }
            consecutive_timeouts = 0;

            if (!m_config.use_testsrc && dvrk_gst::is_socket_reference(m_config.stereo_input)) {
                auto sockets = dvrk_gst::list_sockets();
                bool exists = false;
                for (const auto &s : sockets) {
                    if (s == m_config.stereo_input) { exists = true; break; }
                }

                if (!exists) {
                    {
                        std::lock_guard<std::mutex> lock(m_stats_mutex);
                        m_stats.status_message = "Waiting for " + m_config.stereo_input + "...";
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                    continue;
                }
            }

            {
                std::lock_guard<std::mutex> lock(m_stats_mutex);
                m_stats.status_message = "Connecting to " + m_config.stereo_input + "...";
            }
            if (!create_channel(m_stereo_channel, m_config.stereo_input, m_config.width, m_config.height, m_config.framerate, m_config.use_testsrc)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
                continue;
            }
        }

        // 2. Check for pipeline bus errors / stream disconnection
        if (!check_channel_bus(m_stereo_channel)) {
            std::cerr << "[GstStereoReceiver] Connection lost or stream error on "
                      << m_stereo_channel.name << ". Reconnecting..." << std::endl;
            destroy_channel(m_stereo_channel);
            m_connected = false;
            {
                std::lock_guard<std::mutex> lock(m_stats_mutex);
                m_stats.is_connected = false;
                m_stats.status_message = "Connection lost. Retrying...";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
            continue;
        }

        // 3. Try pulling sample
        GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(m_stereo_channel.appsink), timeout_ns);
        if (!sample) {
            consecutive_timeouts++;
            bool is_eos = (m_stereo_channel.appsink && gst_app_sink_is_eos(GST_APP_SINK(m_stereo_channel.appsink)));
            bool bus_err = !check_channel_bus(m_stereo_channel);
            bool socket_lost = false;

            if (consecutive_timeouts >= 12 && !m_config.use_testsrc && dvrk_gst::is_socket_reference(m_config.stereo_input)) {
                auto sockets = dvrk_gst::list_sockets();
                bool exists = false;
                for (const auto &s : sockets) {
                    if (s == m_config.stereo_input) { exists = true; break; }
                }
                socket_lost = !exists;
            }

            if (is_eos || bus_err || socket_lost) {
                destroy_channel(m_stereo_channel);
                m_connected = false;
                {
                    std::lock_guard<std::mutex> lock(m_stats_mutex);
                    m_stats.is_connected = false;
                    m_stats.status_message = "Connection lost. Waiting for source...";
                }
                consecutive_timeouts = 0;
            }
            continue;
        }

        consecutive_timeouts = 0;

        cv::Mat full_mat;
        uint64_t pts = 0;
        bool ok = extract_mat_from_sample(sample, full_mat, pts);
        gst_sample_unref(sample);

        if (!ok || full_mat.empty() || full_mat.cols < 2) continue;

        m_connected = true;
        {
            std::lock_guard<std::mutex> lock(m_stats_mutex);
            m_stats.is_connected = true;
            m_stats.status_message = "Connected";
            m_last_frame_time = std::chrono::steady_clock::now();
        }
        m_stereo_channel.frame_count++;

        int half_w = full_mat.cols / 2;
        int h = full_mat.rows;

        // Split Left (0..half_w) and Right (half_w..full_w)
        cv::Mat left_view = full_mat(cv::Rect(0, 0, half_w, h)).clone();
        cv::Mat right_view = full_mat(cv::Rect(half_w, 0, half_w, h)).clone();

        uint64_t ts = (pts > 0) ? pts :
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

        {
            std::lock_guard<std::mutex> lock(m_pair_mutex);
            m_latest_left = left_view;
            m_latest_right = right_view;
            m_latest_timestamp_ns = ts;
            m_new_pair_ready = true;
        }
        {
            std::lock_guard<std::mutex> lock(m_stats_mutex);
            m_stats.pairs_delivered++;
            m_stats.single_eye_width = half_w;
            m_stats.single_eye_height = h;
        }
        m_pair_cv.notify_all();

        FramePairCallback cb;
        {
            std::lock_guard<std::mutex> lock(m_callback_mutex);
            cb = m_callback;
        }
        if (cb) {
            cb(left_view, right_view, ts);
        }
    }
}

bool GstStereoReceiver::get_latest_frame_pair(cv::Mat &left, cv::Mat &right, uint64_t &timestamp_ns, int timeout_ms) {
    std::unique_lock<std::mutex> lock(m_pair_mutex);
    if (!m_new_pair_ready) {
        if (!m_pair_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] {
            return m_new_pair_ready || !m_running;
        })) {
            return false;
        }
    }

    if (!m_new_pair_ready) {
        return false;
    }

    left = m_latest_left;
    right = m_latest_right;
    timestamp_ns = m_latest_timestamp_ns;
    m_new_pair_ready = false;
    return true;
}

void GstStereoReceiver::set_callback(FramePairCallback callback) {
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    m_callback = callback;
}

StreamStats GstStereoReceiver::get_stats() const {
    std::lock_guard<std::mutex> lock(m_stats_mutex);
    auto now = std::chrono::steady_clock::now();
    double elapsed_sec = std::chrono::duration<double>(now - m_last_stats_time).count();

    if (elapsed_sec >= 0.5) {
        uint64_t current_count = m_stereo_channel.frame_count.load();
        m_stats.fps = static_cast<double>(current_count - m_stats.frames_received) / elapsed_sec;
        m_stats.frames_received = current_count;
        m_last_stats_time = now;
    }

    if (m_stats.is_connected && m_last_frame_time.time_since_epoch().count() > 0) {
        double time_since_frame = std::chrono::duration<double>(now - m_last_frame_time).count();
        if (time_since_frame > 1.5) {
            m_stats.is_connected = false;
            m_stats.status_message = "Stream interrupted (no frames for >1.5s)";
            m_stats.fps = 0.0;
        }
    }

    return m_stats;
}

} // namespace dvrk_data::calibration
