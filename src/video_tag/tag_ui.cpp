#include "tag_ui.hpp"
#include <gst/video/videooverlay.h>
#include <gdk/gdkx.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <dvrk_data/collection_config.hpp>
#include <dvrk_data/tags.hpp>
#include <dvrk_data/video_timeline.hpp>

namespace {

bool json_timestamp(const Json::Value& value, long long& timestamp) {
    if (!value.isInt64() && !value.isUInt64()) {
        return false;
    }
    timestamp = value.asInt64();
    return timestamp >= 0;
}

bool explicit_annotation_timestamp_ns(const Json::Value& frame,
                                      long long& timestamp) {
    const Json::Value& realtime = frame["cpu"]["realtime"];
    if (!realtime.isObject()) {
        return false;
    }

    long long mono = 0;
    if (json_timestamp(realtime["mono_source_ns"], mono)) {
        timestamp = mono;
        return true;
    }

    long long left = 0;
    long long right = 0;
    const bool has_left = json_timestamp(realtime["left_source_ns"], left);
    const bool has_right = json_timestamp(realtime["right_source_ns"], right);
    if (has_left && has_right) {
        timestamp = left / 2 + right / 2 + (left % 2 + right % 2) / 2;
        return true;
    }
    if (has_left) {
        timestamp = left;
        return true;
    }
    if (has_right) {
        timestamp = right;
        return true;
    }

    return json_timestamp(realtime["recorder_reception_ns"], timestamp);
}

} // namespace

TagWindow::TagWindow(const std::string& video, const std::string&config, const std::string& tags_file, bool load_session_tags)
    : m_main_hbox(Gtk::ORIENTATION_HORIZONTAL, 10),
      m_left_vbox(Gtk::ORIENTATION_VERTICAL, 5),
      m_right_vbox(Gtk::ORIENTATION_VERTICAL, 5),
      m_video_container(Gtk::ORIENTATION_VERTICAL),
      m_timeline_hbox(Gtk::ORIENTATION_HORIZONTAL, 5),
      m_timeline_slider(Gtk::ORIENTATION_HORIZONTAL),
      m_frame_hbox(Gtk::ORIENTATION_HORIZONTAL, 5),
      m_frame_slider(Gtk::ORIENTATION_HORIZONTAL),
      m_controls_hbox(Gtk::ORIENTATION_HORIZONTAL, 5),
      m_save_quit_hbox(Gtk::ORIENTATION_HORIZONTAL, 10)
{
    m_data.video_path = video;
    m_data.config_path = config;
    m_load_session_tags = load_session_tags;

    set_title("C++ Video Tag - " + video);
    set_default_size(1300, 800);
    set_border_width(10); // Add space around main window

    // Layout setup
    add(m_main_hbox);
    m_main_hbox.pack_start(m_left_vbox, Gtk::PACK_EXPAND_WIDGET);
    m_main_hbox.pack_start(m_right_vbox, Gtk::PACK_SHRINK);

    // Left Side: Video
    m_current_stages_label.set_margin_bottom(5);
    m_current_stages_label.set_line_wrap(true);
    m_current_stages_label.set_justify(Gtk::JUSTIFY_CENTER);
    m_left_vbox.pack_start(m_current_stages_label, Gtk::PACK_SHRINK);

    m_video_container.set_size_request(800, 450);
    m_video_container.pack_start(m_video_view, Gtk::PACK_EXPAND_WIDGET);
    m_video_dispatch_connection = m_video_dispatcher.connect(
        sigc::mem_fun(*this, &TagWindow::display_video_sample));
    m_left_vbox.pack_start(m_video_container, Gtk::PACK_EXPAND_WIDGET);

    m_info_label.set_text("Loading...");
    m_left_vbox.pack_start(m_info_label, Gtk::PACK_SHRINK);

    m_stats_label.set_text("Ready");
    m_stats_label.set_alignment(0.0, 0.5);
    m_left_vbox.pack_start(m_stats_label, Gtk::PACK_SHRINK);

    // Frame Slider
    m_frame_hbox.pack_start(*Gtk::make_managed<Gtk::Label>("Frame:"), Gtk::PACK_SHRINK);
    m_frame_slider.set_range(0, 100);
    m_frame_slider.set_draw_value(false);
    m_frame_slider.set_tooltip_text("Seek by frame number");
    m_frame_slider.signal_value_changed().connect(sigc::mem_fun(*this, &TagWindow::on_frame_slider_moved));
    m_frame_hbox.pack_start(m_frame_slider, Gtk::PACK_EXPAND_WIDGET);
    m_frame_label.set_text("0 / 0 (00:00.000)");
    m_frame_label.set_width_chars(25);
    m_frame_hbox.pack_start(m_frame_label, Gtk::PACK_SHRINK);
    m_left_vbox.pack_start(m_frame_hbox, Gtk::PACK_SHRINK);

    // Controls
    m_begin_btn.set_label("Begin");
    m_begin_btn.set_tooltip_text("Jump to beginning (a)");
    m_begin_btn.signal_clicked().connect(sigc::mem_fun(*this, &TagWindow::on_begin));
    m_controls_hbox.pack_start(m_begin_btn, Gtk::PACK_SHRINK);

    m_prev_btn.set_label("Previous");
    m_prev_btn.set_tooltip_text("Previous frame (S); hold to accelerate");
    m_prev_btn.signal_pressed().connect(sigc::mem_fun(*this, &TagWindow::on_prev_btn_pressed));
    m_prev_btn.signal_released().connect(sigc::mem_fun(*this, &TagWindow::on_prev_btn_released));
    m_controls_hbox.pack_start(m_prev_btn, Gtk::PACK_SHRINK);

    m_play_btn.set_label("Play");
    m_play_btn.set_tooltip_text("Play/Pause (D or Space)");
    m_play_btn.set_size_request(80, -1);
    m_play_btn.signal_clicked().connect(sigc::mem_fun(*this, &TagWindow::on_play_pause));
    m_controls_hbox.pack_start(m_play_btn, Gtk::PACK_SHRINK);

    m_next_btn.set_label("Next");
    m_next_btn.set_tooltip_text("Next frame (F); hold to accelerate");
    m_next_btn.signal_pressed().connect(sigc::mem_fun(*this, &TagWindow::on_next_btn_pressed));
    m_next_btn.signal_released().connect(sigc::mem_fun(*this, &TagWindow::on_next_btn_released));
    m_controls_hbox.pack_start(m_next_btn, Gtk::PACK_SHRINK);

    m_speed_combo.append("0.1", "0.1x");
    m_speed_combo.append("0.25", "0.25x");
    m_speed_combo.append("0.5", "0.5x");
    m_speed_combo.append("0.75", "0.75x");
    m_speed_combo.append("1.0", "1.0x");
    m_speed_combo.append("1.25", "1.25x");
    m_speed_combo.append("1.5", "1.5x");
    m_speed_combo.append("2.0", "2.0x");
    m_speed_combo.append("3.0", "3.0x");
    m_speed_combo.append("4.0", "4.0x");
    m_speed_combo.append("5.0", "5.0x");
    m_speed_combo.set_active_id("1.0");
    m_speed_combo.set_tooltip_text("Playback speed and maximum held-step speed");
    m_speed_combo.signal_changed().connect(sigc::mem_fun(*this, &TagWindow::on_speed_changed));
    m_controls_hbox.pack_start(m_speed_combo, Gtk::PACK_SHRINK);

    m_left_vbox.pack_start(m_controls_hbox, Gtk::PACK_SHRINK);

    // Right Side: Tags
    m_right_vbox.set_size_request(350, -1);
    Gtk::Label* tags_title = Gtk::make_managed<Gtk::Label>();
    tags_title->set_markup("<b>Tags</b>");
    m_right_vbox.pack_start(*tags_title, Gtk::PACK_SHRINK);

    m_tags_scroll.set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_AUTOMATIC);
    m_tags_grid.set_column_spacing(5);
    m_tags_grid.set_row_spacing(2);
    m_tags_scroll.add(m_tags_grid);
    m_right_vbox.pack_start(m_tags_scroll, Gtk::PACK_EXPAND_WIDGET);

    m_save_btn.set_label("Save Tags");
    m_save_btn.set_tooltip_text("Save current tags to file");
    m_save_btn.signal_clicked().connect(sigc::mem_fun(*this, &TagWindow::on_save));
    m_save_quit_hbox.pack_start(m_save_btn, Gtk::PACK_EXPAND_WIDGET);

    m_quit_btn.set_label("Quit");
    m_quit_btn.set_tooltip_text("Quit (q)");
    m_quit_btn.signal_clicked().connect(sigc::mem_fun(*this, &TagWindow::close));
    m_save_quit_hbox.pack_start(m_quit_btn, Gtk::PACK_SHRINK);

    m_right_vbox.pack_start(m_save_quit_hbox, Gtk::PACK_SHRINK);

    show_all_children();

    // Configuration and Pipeline
    if (!config.empty()) load_config(config);
    if (!video.empty()) {
        setup_pipeline();
        load_tags(tags_file);
        update_tag_navigation_ui();
    }

    m_timer_conn = Glib::signal_timeout().connect(sigc::mem_fun(*this, &TagWindow::on_ui_update_timer), 100);
    signal_key_press_event().connect(sigc::mem_fun(*this, &TagWindow::on_key_press), false);
    signal_key_release_event().connect(sigc::mem_fun(*this, &TagWindow::on_key_release), false);
    signal_focus_out_event().connect([this](GdkEventFocus*) {
        clear_step_inputs();
        return false;
    }, false);
}

TagWindow::~TagWindow() {
    m_step_timer_conn.disconnect();
    m_timer_conn.disconnect();
    m_video_dispatch_connection.disconnect();
    if (m_data.pipeline) {
        gst_element_set_state(m_data.pipeline, GST_STATE_NULL);
        if (m_data.video_sink) gst_object_unref(m_data.video_sink);
        gst_object_unref(m_data.pipeline);
    }
    if (m_pending_video_sample) gst_sample_unref(m_pending_video_sample);
}

void TagWindow::add_tag_row(const std::string& tag_name) {
    if (m_tag_buttons.count(tag_name)) return;

    auto* btn = Gtk::make_managed<Gtk::ToggleButton>(tag_name);
    btn->set_hexpand(true);
    btn->set_margin_top(1);
    btn->set_margin_bottom(1);
    m_tags_grid.attach(*btn, 0, m_grid_row_count, 1, 1);
    m_tag_buttons[tag_name] = btn;
    btn->signal_clicked().connect([this, tag_name](){ on_tag_toggle(tag_name); });

    auto* count_lbl = Gtk::make_managed<Gtk::Label>();
    count_lbl->set_markup("<b>0</b>");
    count_lbl->set_width_chars(3);
    count_lbl->set_margin_top(1);
    count_lbl->set_margin_bottom(1);
    m_tags_grid.attach(*count_lbl, 1, m_grid_row_count, 1, 1);
    m_tag_count_labels[tag_name] = count_lbl;

    auto* combo = Gtk::make_managed<Gtk::ComboBoxText>();
    combo->set_no_show_all(true);
    combo->hide();
    combo->set_margin_top(1);
    combo->set_margin_bottom(1);
    m_tags_grid.attach(*combo, 2, m_grid_row_count, 1, 1);
    m_tag_combos[tag_name] = combo;
    combo->signal_changed().connect([this, tag_name](){ on_tag_jump(tag_name); });

    m_grid_row_count++;
}

void TagWindow::load_config(const std::string& path) {
    if (path.empty()) return;

    Json::Value root;
    if (!dc::Config::load_from_file(path, root)) {
        std::cerr << "Failed to load config: " << path << std::endl;
        return;
    }

    // dc::Config::load_from_file already checks for dvrk_data:record@1.0.0
    dc::AppConfig cfg = dc::Config::parse_app_config(root);

    for (const auto& s : cfg.stages) {
        m_data.stages.push_back(s);
        add_tag_row(s + "_start");
        add_tag_row(s + "_end");
    }

    for (const auto& t : cfg.tags) {
        m_data.tags.push_back(t);
        add_tag_row(t);
    }
    m_tags_grid.show_all();
}

void TagWindow::load_sidecar_json() {
    std::string base = m_data.video_path;
    size_t dot = base.find_last_of(".");
    if (dot != std::string::npos) base = base.substr(0, dot);
    std::string json_file = base + ".json";

    Json::Value root;
    if (!dc::Config::load_from_file(json_file, root)) {
        std::cerr << "No sidecar JSON found at: " << json_file << std::endl;
        return;
    }

    if (!dc::Config::check_type(root, "dvrk_data:video_sidecar@1.0.0", json_file)) {
        return;
    }

    if (root.isMember("frames") && root["frames"].isArray()) {
        m_data.frame_cpu_timestamps.clear();
        m_data.frame_gst_timestamps.clear();

        for (const auto& frame : root["frames"]) {
            const Json::Value& pts = frame["gst"]["pts_ns"];
            long long cpu_ts = 0;
            if (!explicit_annotation_timestamp_ns(frame, cpu_ts)) {
                std::cerr << "Missing explicit annotation timestamp in " << json_file << std::endl;
                m_data.frame_cpu_timestamps.clear();
                m_data.frame_gst_timestamps.clear();
                return;
            }
            if (!pts.isInt64() && !pts.isUInt64()) {
                std::cerr << "Missing GStreamer PTS in " << json_file << std::endl;
                m_data.frame_cpu_timestamps.clear();
                m_data.frame_gst_timestamps.clear();
                return;
            }
            m_data.frame_cpu_timestamps.push_back(cpu_ts);
            const long long t = pts.asInt64();

            // Recorder timestamps use the live pipeline clock. MP4 playback
            // starts at zero; normalize the complete array after loading it.
            m_data.frame_gst_timestamps.push_back(t);
        }
        dc::normalize_video_pts(m_data.frame_gst_timestamps);
    }

    if (root.isMember("start_cpu_realtime_ns")) {
        m_data.session_start_cpu_ns = root["start_cpu_realtime_ns"].asInt64();
    } else if (!m_data.frame_cpu_timestamps.empty()) {
        m_data.session_start_cpu_ns = m_data.frame_cpu_timestamps[0];
    }

    if (!m_data.frame_cpu_timestamps.empty()) {
         m_data.session_duration_ns = m_data.frame_cpu_timestamps.back() - m_data.session_start_cpu_ns;
    }

    // Auto-load session tags if requested or if flag is present in JSON
    if (m_load_session_tags) {
        this->load_session_tags();
    }

    // Auto-load config labels if not provided
    if (m_data.config_path.empty() && root.isMember("config_files") && root["config_files"].isArray()) {
        for (const auto& cfg : root["config_files"]) {
            load_config(cfg.asString());
        }
    }

    m_info_label.set_text("Loaded sidecar JSON: " + json_file);
}

void TagWindow::setup_pipeline() {
    load_sidecar_json();

    // Use an explicit software decode chain to avoid NVMM/NVV4L2 negotiation issues during playback.
    // Deliver native-resolution frames to the zoomable view on the GTK thread.
    const char* pixel_format = G_BYTE_ORDER == G_LITTLE_ENDIAN ? "BGRx" : "xRGB";
    std::string pipe_str =
        "filesrc location=\"" + m_data.video_path +
        "\" ! qtdemux name=demux "
        "demux.video_0 ! queue ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=" +
        std::string(pixel_format) + " ! appsink name=vsink sync=true max-buffers=1 drop=true";
    GError* err = nullptr;
    m_data.pipeline = gst_parse_launch(pipe_str.c_str(), &err);
    if (!m_data.pipeline) {
        std::cerr << "Failed to create pipeline: " << (err ? err->message : "unknown") << std::endl;
        if (err) {
            std::cerr << "GStreamer error: " << err->message << std::endl;
            g_error_free(err);
        }
        return;
    }

    m_data.video_sink = gst_bin_get_by_name(GST_BIN(m_data.pipeline), "vsink");

    GstAppSinkCallbacks callbacks{};
    callbacks.new_preroll = &TagWindow::on_video_preroll;
    callbacks.new_sample = &TagWindow::on_video_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(m_data.video_sink), &callbacks, this, nullptr);

    gst_element_set_state(m_data.pipeline, GST_STATE_PAUSED);

    // Get duration and FPS
    GstState state = GST_STATE_PAUSED;
    gst_element_get_state(m_data.pipeline, &state, nullptr, GST_SECOND);

    gint64 duration = 0;
    if (gst_element_query_duration(m_data.pipeline, GST_FORMAT_TIME, &duration)) {
        m_data.duration_ns = duration;
        
        m_data.session_duration_ns = duration;
    }

    // Attempt to get FPS from sink caps
    GstPad* pad = gst_element_get_static_pad(m_data.video_sink, "sink");
    GstCaps* caps = gst_pad_get_current_caps(pad);
    if (caps) {
        GstStructure* s = gst_caps_get_structure(caps, 0);
        gint num, den;
        if (gst_structure_get_fraction(s, "framerate", &num, &den) && den != 0) {
            m_data.fps = (double)num / (double)den;
        }

        gint width, height;
        if (gst_structure_get_int(s, "width", &width) && gst_structure_get_int(s, "height", &height)) {
             std::stringstream ss;
             ss << "Res: " << width << "x" << height << " | FPS: " << std::fixed << std::setprecision(2) << m_data.fps;
             m_stats_label.set_text(ss.str());
        }
        gst_caps_unref(caps);
    }
    gst_object_unref(pad);

    if (!m_data.frame_gst_timestamps.empty()) {
        m_data.total_frames = m_data.frame_gst_timestamps.size();

        // Compute absolute indices based on FPS for reference
        // This helps us know "This is frame #120 in the raw video file"
        m_data.frame_abs_indices.clear();
        for (long long ts : m_data.frame_gst_timestamps) {
            long long idx = (long long)((double)ts / 1e9 * m_data.fps + 0.5); // Round
            m_data.frame_abs_indices.push_back(idx);
        }

        m_data.current_frame = 0;
        gst_element_set_state(m_data.pipeline, GST_STATE_PAUSED);
        do_seek(m_data.frame_gst_timestamps[0]);
    } else {
        m_data.total_frames = (long long)((double)m_data.duration_ns / 1e9 * m_data.fps);
    }
    m_frame_slider.set_range(0, m_data.total_frames > 0 ? m_data.total_frames - 1 : 0);
}

GstFlowReturn TagWindow::on_video_preroll(GstAppSink* sink, gpointer user_data) {
    return static_cast<TagWindow*>(user_data)->receive_video_sample(sink, true);
}

GstFlowReturn TagWindow::on_video_sample(GstAppSink* sink, gpointer user_data) {
    return static_cast<TagWindow*>(user_data)->receive_video_sample(sink, false);
}

GstFlowReturn TagWindow::receive_video_sample(GstAppSink* sink, bool preroll) {
    GstSample* sample = preroll ? gst_app_sink_pull_preroll(sink) : gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_FLUSHING;
    bool notify = false;
    {
        std::lock_guard<std::mutex> lock(m_video_mutex);
        if (m_pending_video_sample) gst_sample_unref(m_pending_video_sample);
        m_pending_video_sample = sample;
        notify = !m_video_notification_pending;
        m_video_notification_pending = true;
    }
    if (notify) m_video_dispatcher.emit();
    return GST_FLOW_OK;
}

void TagWindow::display_video_sample() {
    GstSample* sample = nullptr;
    {
        std::lock_guard<std::mutex> lock(m_video_mutex);
        sample = m_pending_video_sample;
        m_pending_video_sample = nullptr;
        m_video_notification_pending = false;
    }
    if (sample) {
        m_video_view.set_sample(sample);
        gst_sample_unref(sample);
    }
}

void TagWindow::do_seek(gint64 ns) {
    if (!m_data.pipeline) return;
    double speed = std::stod(m_speed_combo.get_active_id());
    gst_element_seek(m_data.pipeline, speed, GST_FORMAT_TIME,
                     (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_ACCURATE),
                     GST_SEEK_TYPE_SET, ns,
                     GST_SEEK_TYPE_NONE, -1);
}

void TagWindow::seek_to_frame(long long frame_idx) {
    clear_step_inputs();
    m_step_position_valid = false;
    if (frame_idx < 0) frame_idx = 0;
    if (frame_idx >= m_data.total_frames) frame_idx = m_data.total_frames - 1;
    
    // Update state first
    m_data.current_frame = frame_idx;
    
    do_seek(frame_time_ns(frame_idx));
}

long long TagWindow::frame_time_ns(long long frame_idx) const {
    if (frame_idx >= 0 && frame_idx < static_cast<long long>(m_data.frame_gst_timestamps.size())) {
        return m_data.frame_gst_timestamps[frame_idx];
    }
    return static_cast<long long>(static_cast<double>(std::max(0LL, frame_idx)) / m_data.fps * 1e9);
}

void TagWindow::on_play_pause() {
    clear_step_inputs();
    m_step_position_valid = false;
    GstState current, pending;
    gst_element_get_state(m_data.pipeline, &current, &pending, 0);
    if (current == GST_STATE_PLAYING) {
        gst_element_set_state(m_data.pipeline, GST_STATE_PAUSED);
        m_play_btn.set_label("Play");
    } else {
        // If we are at the beginning or before the first frame, skip to first frame
        if (!m_data.frame_gst_timestamps.empty()) {
            gint64 pos = 0;
            if (gst_element_query_position(m_data.pipeline, GST_FORMAT_TIME, &pos)) {
                if (pos < m_data.frame_gst_timestamps[0]) {
                    seek_to_frame(0);
                }
            }
        }
        gst_element_set_state(m_data.pipeline, GST_STATE_PLAYING);
        m_play_btn.set_label("Pause");
        // Ensure speed is maintained
        on_speed_changed();
    }
}

long long TagWindow::ns_to_nearest_frame(long long ns) {
    if (m_data.frame_gst_timestamps.empty()) {
        return (long long)((double)ns / 1e9 * m_data.fps);
    }
    
    // If we're before the first frame, return frame 0
    if (ns <= m_data.frame_gst_timestamps.front()) return 0;
    // If we're after the last frame, return the last frame index
    if (ns >= m_data.frame_gst_timestamps.back()) return m_data.frame_gst_timestamps.size() - 1;

    auto it = std::lower_bound(m_data.frame_gst_timestamps.begin(), m_data.frame_gst_timestamps.end(), ns);
    if (it == m_data.frame_gst_timestamps.end()) return m_data.frame_gst_timestamps.size() - 1;
    if (it == m_data.frame_gst_timestamps.begin()) return 0;

    long long dist1 = std::abs((long long)*it - ns);
    long long dist2 = std::abs((long long)*(it - 1) - ns);
    if (dist1 < dist2) return std::distance(m_data.frame_gst_timestamps.begin(), it);
    else return std::distance(m_data.frame_gst_timestamps.begin(), it - 1);
}

void TagWindow::on_begin() {
    if (m_data.pipeline) gst_element_set_state(m_data.pipeline, GST_STATE_PAUSED);
    seek_to_frame(0);
}

void TagWindow::on_frame_slider_moved() {
    if (m_internal_update) return;
    if (m_data.pipeline) gst_element_set_state(m_data.pipeline, GST_STATE_PAUSED);
    long long frame = (long long)m_frame_slider.get_value();
    seek_to_frame(frame);
}

void TagWindow::on_speed_changed() {
    // The held-step timer reads the current maximum on every tick.
    if (m_step_direction != 0) return;
    gint64 pos = 0;
    if (gst_element_query_position(m_data.pipeline, GST_FORMAT_TIME, &pos)) {
        do_seek(pos);
    }
}

void TagWindow::on_tag_toggle(const std::string& tag_name) {
    if (m_internal_update || m_data.total_frames == 0) return;

    auto& tags_at_frame = m_data.frame_tags[m_data.current_frame];
    auto it = std::find(tags_at_frame.begin(), tags_at_frame.end(), tag_name);

    if (m_tag_buttons[tag_name]->get_active()) {
        if (it == tags_at_frame.end()) {
            tags_at_frame.push_back(tag_name);
            m_data.unsaved_changes = true;
        }
    } else {
        if (it != tags_at_frame.end()) {
            tags_at_frame.erase(it);
            if (tags_at_frame.empty()) m_data.frame_tags.erase(m_data.current_frame);
            m_data.unsaved_changes = true;
        }
    }
    update_tag_navigation_ui();
}

void TagWindow::on_tag_jump(const std::string& tag_name) {
    if (m_internal_update) return;
    auto* combo = m_tag_combos[tag_name];
    std::string id = combo->get_active_id();
    if (id.empty()) return;

    long long frame = std::stoll(id);
    if (m_data.pipeline) gst_element_set_state(m_data.pipeline, GST_STATE_PAUSED);
    seek_to_frame(frame);
}

void TagWindow::update_tag_navigation_ui() {
    m_internal_update = true;

    // Calculate counts and frames for each tag
    std::map<std::string, std::vector<long long>> tag_occurences;
    for (auto const& [frame, tags] : m_data.frame_tags) {
        for (auto const& t : tags) {
            tag_occurences[t].push_back(frame);
        }
    }

    // Sort frame numbers for each tag to ensure dropdown order
    for (auto& [name, frames] : tag_occurences) {
        std::sort(frames.begin(), frames.end());
    }

    // Update labels and combo boxes
    for (auto const& [name, btn] : m_tag_buttons) {
        int count = tag_occurences.count(name) ? tag_occurences[name].size() : 0;
        m_tag_count_labels[name]->set_markup("<b>" + std::to_string(count) + "</b>");

        auto* combo = m_tag_combos[name];
        if (count > 0) {
            combo->show();
            combo->remove_all();
            int occurrence = 0;
            for (long long f : tag_occurences[name]) {
                long long rel_ns = 0;
                if (f >= 0 && f < (long long)m_data.frame_cpu_timestamps.size()) {
                    rel_ns = m_data.frame_cpu_timestamps[f] - m_data.session_start_cpu_ns;
                } else {
                    rel_ns = (long long)((double)f / m_data.fps * 1e9);
                }
                combo->append(std::to_string(f), std::to_string(++occurrence) + ": " +
                              format_time_simple((double)rel_ns / 1e9));
            }
        } else {
            combo->hide();
        }
    }
    m_internal_update = false;
}

std::string TagWindow::format_time_simple(double seconds) {
    int m = (int)seconds / 60;
    double s = seconds - (m * 60);
    std::stringstream ss;
    ss << m << ":" << std::fixed << std::setprecision(1) << s << "s";
    return ss.str();
}

bool TagWindow::on_ui_update_timer() {
    if (!m_data.pipeline) return true;

    gint64 pos = 0;
    if (gst_element_query_position(m_data.pipeline, GST_FORMAT_TIME, &pos)) {
        m_internal_update = true;

        // Skip to the first frame if the current position is before it
        if (!m_data.frame_gst_timestamps.empty() && pos < m_data.frame_gst_timestamps[0] - 10000000) {
            seek_to_frame(0);
            return true;
        }

        // Update Play/Pause button label based on actual state
        GstState current, pending;
        gst_element_get_state(m_data.pipeline, &current, &pending, 0);
        if (current == GST_STATE_PLAYING) {
            m_play_btn.set_label("Pause");

            // If playing, check if we need to skip to the next frame
            if (!m_data.frame_gst_timestamps.empty()) {
                gint64 current_frame_ts = m_data.frame_gst_timestamps[m_data.current_frame];
                
                // If GStreamer pos is significantly past the current recorded frame's expected timestamp,
                // we are moving towards the next frame.
                // If the next frame is contiguous, we do nothing.
                // If there is a large gap, we must SEEK.
                
                // Look ahead: Where should the next frame be?
                long long next_idx = m_data.current_frame + 1;
                
                if (next_idx < m_data.total_frames) {
                    long long next_frame_ts = m_data.frame_gst_timestamps[next_idx];
                    
                    // Are we approaching or past the point where we should switch to next_idx?
                    // Or are we adrift in a gap?
                    
                    // If pos > current + 10ms, we are "done" with current frame.
                    if (pos > current_frame_ts + 10000000) {
                        
                        // Check if next frame is contiguous in ABSOLUTE index time
                        bool is_contiguous = false;
                        if (!m_data.frame_abs_indices.empty()) {
                            long long cur_abs = m_data.frame_abs_indices[m_data.current_frame];
                            long long next_abs = m_data.frame_abs_indices[next_idx];
                            if (next_abs - cur_abs <= 1) is_contiguous = true;
                        } else {
                            // Fallback heuristic: timestamp difference is ~1 frame duration
                            double frame_dur = 1e9 / m_data.fps;
                            if (std::abs(next_frame_ts - current_frame_ts - frame_dur) < (frame_dur * 0.1)) {
                                is_contiguous = true;
                            }
                        }
                        
                        if (is_contiguous) {
                            // Smooth playback: Just update the index
                            // Wait, if it's strictly contiguous, the timestamp should just be +1/FPS
                            // But if we are past next_frame_ts itself, we should update index.
                            if (pos >= next_frame_ts - 5000000) { // Within 5ms of next frame or past it
                                m_data.current_frame = next_idx;
                            }
                        } else {
                            // GAP DETECTED: Jump immediately to next_frame_ts
                            // Only jump if we are NOT already close to it (to avoid loops)
                            if (pos < next_frame_ts - 10000000) { // If we are BEFORE the target (in the gap)
                                seek_to_frame(next_idx);
                                return true;
                            } else {
                                // We are already past the gap target? Just update index
                                m_data.current_frame = next_idx;
                            }
                        }
                    }
                } else {
                     // End of recorded frames - pause if we run over
                     if (pos > current_frame_ts + 50000000) {
                        gst_element_set_state(m_data.pipeline, GST_STATE_PAUSED);
                        m_play_btn.set_label("Play");
                        seek_to_frame(m_data.total_frames - 1);
                        return true;
                     }
                }
            }
        } else {
            m_play_btn.set_label("Play");
        }

        long long frame = ns_to_nearest_frame(pos);
        long long virtual_ms = frame_time_ns(frame) / 1000000;
        long long total_ms = frame_time_ns(m_data.total_frames - 1) / 1000000;
        
        auto format_virtual_time = [](long long ms) {
            int mins = ms / 60000;
            double secs = (ms % 60000) / 1000.0;
            std::stringstream ss;
            ss << mins << ":" << std::fixed << std::setfill('0') << std::setw(6) << std::setprecision(3) << secs;
            return ss.str();
        };
        
        std::stringstream ss_frame;
        ss_frame << frame << " / " << m_data.total_frames - 1 << " (" 
                 << format_virtual_time(virtual_ms) << " / " << format_virtual_time(total_ms) << ")";

        if (frame != m_data.current_frame || true) { // Always refresh display labels to ensure consistency
            m_data.current_frame = frame;
            m_frame_slider.set_value(frame);
            m_frame_label.set_text(ss_frame.str());

            // Update current stages label
            std::map<std::string, int> starts;
            std::map<std::string, int> ends;
            for (auto const& [f, tags] : m_data.frame_tags) {
                if (f > frame) break;
                for (const auto& t : tags) {
                    if (t.size() > 6 && t.compare(t.size() - 6, 6, "_start") == 0) {
                        starts[t.substr(0, t.size() - 6)]++;
                    } else if (t.size() > 4 && t.compare(t.size() - 4, 4, "_end") == 0) {
                        ends[t.substr(0, t.size() - 4)]++;
                    }
                }
            }

            std::string active_stages_str = "";
            for (auto const& [name, count] : starts) {
                int end_count = ends[name];
                if (count > end_count) {
                    for (int i = end_count + 1; i <= count; ++i) {
                        if (!active_stages_str.empty()) active_stages_str += ", ";
                        active_stages_str += name + " (" + std::to_string(i) + ")";
                    }
                }
            }

            if (active_stages_str.empty()) {
                m_current_stages_label.set_markup("<i>No active stages</i>");
            } else {
                m_current_stages_label.set_markup("<b>Current Stages: " + active_stages_str + "</b>");
            }

            // Update tag buttons state for this frame
            for (auto const& [name, btn] : m_tag_buttons) {
                auto const& tags = m_data.frame_tags[frame];
                bool active = std::find(tags.begin(), tags.end(), name) != tags.end();
                btn->set_active(active);
            }
        }
        m_internal_update = false;
    }
    return true;
}

bool TagWindow::on_key_press(GdkEventKey* event) {
    bool* pressed = nullptr;
    switch (gdk_keyval_to_lower(event->keyval)) {
        case GDK_KEY_s: pressed = &m_key_s_pressed; break;
        case GDK_KEY_d:
            if (!m_key_d_pressed) {
                on_play_pause();
                m_key_d_pressed = true;
            }
            return true;
        case GDK_KEY_f: pressed = &m_key_f_pressed; break;
        case GDK_KEY_space:
            if (!m_key_space_pressed) {
                on_play_pause();
                m_key_space_pressed = true;
            }
            return true;
        case GDK_KEY_a: on_begin(); return true;
        case GDK_KEY_q: close(); return true;
        default: return false;
    }
    // Ignore operating-system repeats; stepping is driven by monotonic time.
    if (!*pressed) {
        *pressed = true;
        update_step_hold();
    }
    return true;
}

bool TagWindow::on_key_release(GdkEventKey* event) {
    switch (gdk_keyval_to_lower(event->keyval)) {
        case GDK_KEY_s: m_key_s_pressed = false; break;
        case GDK_KEY_d: m_key_d_pressed = false; return true;
        case GDK_KEY_f: m_key_f_pressed = false; break;
        case GDK_KEY_space: m_key_space_pressed = false; return true;
        default: return false;
    }
    update_step_hold();
    return true;
}

void TagWindow::update_step_hold() {
    const bool backward = m_key_s_pressed || m_btn_prev_pressed;
    const bool forward = m_key_f_pressed || m_btn_next_pressed;
    const int direction = backward == forward ? 0 : (forward ? 1 : -1);
    if (direction == m_step_direction) return;
    m_step_timer_conn.disconnect();
    m_step_direction = direction;
    if (direction == 0 || !m_data.pipeline || m_data.total_frames == 0) return;

    GstState current, pending;
    gst_element_get_state(m_data.pipeline, &current, &pending, 0);
    if (current == GST_STATE_PLAYING) {
        m_step_position_valid = false;
        gint64 position = 0;
        if (gst_element_query_position(m_data.pipeline, GST_FORMAT_TIME, &position)) {
            m_data.current_frame = ns_to_nearest_frame(position);
        }
    }
    gst_element_set_state(m_data.pipeline, GST_STATE_PAUSED);
    m_play_btn.set_label("Play");
    if (!m_step_position_valid) {
        m_step_video_frame = std::llround(frame_time_ns(m_data.current_frame) * m_data.fps / 1e9);
        m_step_position_valid = true;
    }
    step_video_frames(direction); // A tap always moves one encoded frame.
    m_held_stepper.reset(g_get_monotonic_time() / 1e6);
    m_step_timer_conn = Glib::signal_timeout().connect(
        sigc::mem_fun(*this, &TagWindow::on_step_timer), 40);
}

void TagWindow::step_video_frames(long long delta) {
    const long long last = std::max(0LL, std::llround(m_data.duration_ns * m_data.fps / 1e9) - 1);
    const long long target = std::clamp(m_step_video_frame + delta, 0LL, last);
    if (target == m_step_video_frame) return;
    m_step_video_frame = target;
    const auto timestamp = std::llround(target * 1e9 / m_data.fps);
    m_data.current_frame = ns_to_nearest_frame(timestamp);
    do_seek(timestamp);
}

bool TagWindow::on_step_timer() {
    if (m_step_direction == 0) return false;
    const double max_speed = std::stod(m_speed_combo.get_active_id());
    const long long frames = m_held_stepper.advance(
        g_get_monotonic_time() / 1e6, m_data.fps, max_speed);
    // Batch fast steps into one seek instead of decoding every skipped frame.
    if (frames > 0) step_video_frames(m_step_direction * frames);
    return true;
}

void TagWindow::clear_step_inputs() {
    m_key_s_pressed = m_key_d_pressed = m_key_f_pressed = false;
    m_key_space_pressed = false;
    m_btn_prev_pressed = m_btn_next_pressed = false;
    update_step_hold();
}

void TagWindow::on_prev_btn_pressed() {
    m_btn_prev_pressed = true;
    update_step_hold();
}

void TagWindow::on_prev_btn_released() {
    m_btn_prev_pressed = false;
    update_step_hold();
}

void TagWindow::on_next_btn_pressed() {
    m_btn_next_pressed = true;
    update_step_hold();
}

void TagWindow::on_next_btn_released() {
    m_btn_next_pressed = false;
    update_step_hold();
}

std::string TagWindow::format_time(long long ns) {
    long long total_sec = ns / 1e9;
    int h = total_sec / 3600;
    int m = (total_sec % 3600) / 60;
    int s = total_sec % 60;
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(2) << h << ":"
       << std::setw(2) << m << ":" << std::setw(2) << s;
    return ss.str();
}

void TagWindow::on_save() {
    save_tags();
    update_tag_navigation_ui();
}

void TagWindow::save_tags() {
    std::string base = m_data.video_path;
    size_t dot = base.find_last_of(".");
    if (dot != std::string::npos) base = base.substr(0, dot);
    std::string tags_file = base + "_tags.json";

    Json::Value root;
    Json::Value stages_array(Json::arrayValue);
    Json::Value tags_obj(Json::objectValue);

    struct Stage {
        std::string name;
        long long start = -1;
        long long end = -1;
    };
    // Support multiple stages with same name using a list
    std::vector<Stage> stage_list;
    // Map to track open stage starts: name -> stack of start frames
    std::map<std::string, std::vector<long long>> open_starts;
    std::vector<std::string> error_messages;

    for (auto const& [frame, tags] : m_data.frame_tags) {
        for (auto const& t : tags) {
            if (t.find("_start") != std::string::npos) {
                std::string name = t.substr(0, t.length() - 6);
                if (!open_starts[name].empty()) {
                     std::string error_msg = "Error: Stage '" + name + "' overlaps with itself! (Start at " + 
                                            format_time(frame_time_ns(open_starts[name].back())) +
                                            " and " + format_time(frame_time_ns(frame)) + ")";
                     error_messages.push_back(error_msg); 
                }
                open_starts[name].push_back(frame);
            } else if (t.find("_end") != std::string::npos) {
                std::string name = t.substr(0, t.length() - 4);
                if (!open_starts[name].empty()) {
                    long long start = open_starts[name].back();
                    open_starts[name].pop_back();
                    stage_list.push_back({name, start, frame});
                } else {
                    // End without start - treat as disconnected end
                    Stage s; 
                    s.name = name; 
                    s.end = frame;
                    stage_list.push_back(s);
                }
            } else {
                tags_obj[t].append((Json::Int64)frame);
            }
        }
    }

    // Add remaining open starts
    for (auto& [name, starts] : open_starts) {
        for (long long s : starts) {
            Stage stage;
            stage.name = name;
            stage.start = s;
            stage_list.push_back(stage);
        }
    }

    std::string current_time_str = dc::get_current_timestamp_iso8601();
    root["type"] = "dvrk_data:video_tags@1.0.0";
    root["created_at"] = current_time_str;

    for (auto const& stage : stage_list) {
        std::string name = stage.name;
        if (stage.start != -1 && stage.end != -1) {
            Json::Value stage_entry;
            stage_entry["name"] = name;
            // Ensure chronological order for start/end
            long long s_frame = std::min(stage.start, stage.end);
            long long e_frame = std::max(stage.start, stage.end);

            if (stage.start > stage.end) {
                std::string error_msg = "Warning: Stage '" + name + "' has start > end! (" +
                                       format_time_simple(frame_time_ns(stage.start) / 1e9) + " > " +
                                       format_time_simple(frame_time_ns(stage.end) / 1e9) + ")";
                std::cerr << error_msg << std::endl;
                m_info_label.set_text(error_msg);
            }

            Json::Value start_obj;
            // frame_absolute: 0-based index in this specific video file
            // frame_relative: Same for now, unless we have session offset logic
            start_obj["frame_absolute"] = (Json::Int64)((s_frame >= 0 && s_frame < (long long)m_data.frame_abs_indices.size()) ? m_data.frame_abs_indices[s_frame] : s_frame);
            start_obj["frame_relative"] = (Json::Int64)s_frame;
            if (s_frame >= 0 && s_frame < (long long)m_data.frame_cpu_timestamps.size()) {
                start_obj["cpu_ts"] = (Json::Int64)m_data.frame_cpu_timestamps[s_frame];
            }
            start_obj["generated_at"] = current_time_str;
            
            Json::Value end_obj;
            end_obj["frame_absolute"] = (Json::Int64)((e_frame >= 0 && e_frame < (long long)m_data.frame_abs_indices.size()) ? m_data.frame_abs_indices[e_frame] : e_frame);
            end_obj["frame_relative"] = (Json::Int64)e_frame;
            if (e_frame >= 0 && e_frame < (long long)m_data.frame_cpu_timestamps.size()) {
                end_obj["cpu_ts"] = (Json::Int64)m_data.frame_cpu_timestamps[e_frame];
            }
            end_obj["generated_at"] = current_time_str;

            stage_entry["start"] = start_obj;
            stage_entry["end"] = end_obj;
            stages_array.append(stage_entry);
        } else {
            std::string status = (stage.start == -1) ? "missing start" : "missing end";
            std::string error_msg = "Error: Stage '" + name + "' is " + status + ". Not saved.";
            std::cerr << error_msg << std::endl;
            m_info_label.set_text(error_msg);
            error_messages.push_back(error_msg);
        }
    }

    if (!error_messages.empty()) {
        std::string full_msg = "";
        for (const auto& msg : error_messages) {
            full_msg += msg + "\n";
        }
        Gtk::MessageDialog dialog(*this, "Stage Error", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK, true);
        dialog.set_secondary_text(full_msg);
        dialog.run();
        return;
    }

    root["stages"] = stages_array;
    root["tags"] = tags_obj;
    root["generated_at"] = current_time_str;
    root["session_tags_loaded"] = m_data.session_tags_loaded;
    
    std::string vname = m_data.video_path;
    size_t last_slash = vname.find_last_of("/\\");
    if (last_slash != std::string::npos) vname = vname.substr(last_slash + 1);
    root["video_file"] = vname;

    std::ofstream ofs(tags_file);
    Json::StreamWriterBuilder builder;
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &ofs);

    m_data.unsaved_changes = false;
    m_info_label.set_text("Saved tags to " + tags_file);
    std::cout << "Saved tags to: " << tags_file << std::endl;
}

void TagWindow::load_tags(const std::string& explicit_path) {
    std::string tags_file = explicit_path;
    if (tags_file.empty()) {
        std::string base = m_data.video_path;
        size_t dot = base.find_last_of(".");
        if (dot != std::string::npos) base = base.substr(0, dot);
        tags_file = base + "_tags.json";
    }

    std::ifstream ifs(tags_file);
    if (!ifs.is_open()) {
        std::cout << "No existing tag file found at: " << tags_file << std::endl;
        return;
    }

    Json::Value root;
    ifs >> root;

    if (root.isMember("session_tags_loaded") && root["session_tags_loaded"].asBool()) {
        m_data.session_tags_loaded = true;
    }

    auto find_frame = [&](long long ts) -> long long {
        if (m_data.frame_cpu_timestamps.empty()) return ts;

        auto it = std::lower_bound(m_data.frame_cpu_timestamps.begin(), m_data.frame_cpu_timestamps.end(), ts);
        if (it == m_data.frame_cpu_timestamps.end()) return m_data.frame_cpu_timestamps.size() - 1;
        if (it == m_data.frame_cpu_timestamps.begin()) return 0;

        long long dist1 = std::abs((long long)*it - ts);
        long long dist2 = std::abs((long long)*(it - 1) - ts);
        if (dist1 < dist2) return std::distance(m_data.frame_cpu_timestamps.begin(), it);
        else return std::distance(m_data.frame_cpu_timestamps.begin(), it - 1);
    };

    std::vector<std::string> missing_tags;
    std::vector<std::string> stages_to_add;
    std::vector<std::string> tags_to_add;

    if (root.isMember("stages")) {
        for (auto& s : root["stages"]) {
            std::string name = s["name"].asString();
            if (m_tag_buttons.count(name + "_start") == 0 || m_tag_buttons.count(name + "_end") == 0) {
                missing_tags.push_back(name + " (stage)");
                stages_to_add.push_back(name);
            }
        }
    }
    if (root.isMember("tags")) {
        for (auto const& name : root["tags"].getMemberNames()) {
            if (m_tag_buttons.count(name) == 0) {
                missing_tags.push_back(name);
                tags_to_add.push_back(name);
            }
        }
    }

    MissingTagsResult res = MissingTagsResult::IGNORE;
    if (!missing_tags.empty()) {
        res = show_missing_tags_dialog(missing_tags, tags_file);
        if (res == MissingTagsResult::ACCEPT) {
            for (const auto& s : stages_to_add) {
                add_tag_row(s + "_start");
                add_tag_row(s + "_end");
            }
            for (const auto& t : tags_to_add) {
                add_tag_row(t);
            }
            m_tags_grid.show_all();
        }
    }

    if (root.isMember("stages")) {
        for (auto& s : root["stages"]) {
            std::string name = s["name"].asString();
            if (m_tag_buttons.count(name + "_start") == 0 || m_tag_buttons.count(name + "_end") == 0) continue;

            auto get_frame = [&](const Json::Value& v) -> long long {
                if (v.isObject()) {
                    // Priority 1: Use timestamp map if available and cpu_ts is present
                    if (!m_data.frame_cpu_timestamps.empty() && v.isMember("cpu_ts")) {
                        return find_frame(v["cpu_ts"].asInt64());
                    }
                    // Priority 2: Use absolute frame index
                    if (v.isMember("frame_absolute")) {
                        return v["frame_absolute"].asInt64();
                    }
                }
                return -1;
            };

            long long start_f = get_frame(s["start"]);
            long long end_f = get_frame(s["end"]);

            if (start_f >= 0 && start_f < m_data.total_frames)
                m_data.frame_tags[start_f].push_back(name + "_start");
            if (end_f >= 0 && end_f < m_data.total_frames)
                m_data.frame_tags[end_f].push_back(name + "_end");
        }
    }

    if (root.isMember("tags")) {
        for (auto const& name : root["tags"].getMemberNames()) {
            if (m_tag_buttons.count(name) == 0) continue;
            for (auto& f : root["tags"][name]) {
                m_data.frame_tags[find_frame(f.asInt64())].push_back(name);
            }
        }
    }

    if (!missing_tags.empty()) {
        std::string inf = (res == MissingTagsResult::ACCEPT) ? " (new tags added)" : " (some tags ignored)";
        m_info_label.set_markup("Loaded tags from <b>" + tags_file + "</b>" + inf);
    } else {
        m_info_label.set_markup("Loaded tags from <b>" + tags_file + "</b>");
    }
}

TagWindow::MissingTagsResult TagWindow::show_missing_tags_dialog(const std::vector<std::string>& missing_tags, const std::string& source) {
    if (missing_tags.empty()) return MissingTagsResult::IGNORE;

    std::string message = "Tags found in " + source + " but not in config:\n";
    for (size_t i = 0; i < missing_tags.size(); ++i) {
        message += " • " + missing_tags[i] + "\n";
        if (i > 15) {
            message += " ... and " + std::to_string(missing_tags.size() - i - 1) + " more";
            break;
        }
    }

    Gtk::MessageDialog dialog(*this, "Missing Tags", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_NONE, true);
    dialog.set_secondary_text(message);
    dialog.add_button("Accept", (int)MissingTagsResult::ACCEPT);
    dialog.add_button("Ignore", (int)MissingTagsResult::IGNORE);
    dialog.add_button("Quit", (int)MissingTagsResult::QUIT);

    int result = dialog.run();
    if (result == (int)MissingTagsResult::QUIT) {
        std::exit(0);
    }
    if (m_data.session_tags_loaded) {
        Gtk::MessageDialog dialog(*this, "Session Tags Already Loaded", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK, true);
        dialog.set_secondary_text("Session tags have already been imported into this video's tag file. Using -T again is not allowed to prevent duplicates.");
        dialog.run();
        std::exit(1);
    }

    return (MissingTagsResult)result;
}

void TagWindow::load_session_tags() {
    if (m_data.session_tags_loaded) {
        Gtk::MessageDialog dialog(*this, "Session Tags Already Loaded", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK, true);
        dialog.set_secondary_text("Session tags have already been imported into this video's tag file. Using -T again is not allowed to prevent duplicates.");
        dialog.run();
        std::exit(1);
    }

    std::string base = m_data.video_path;
    size_t last_slash = base.find_last_of("/\\");
    if (last_slash == std::string::npos) return;
    std::string session_dir = base.substr(0, last_slash);

    // Look for timestamped session tags first
    std::string tags_json = session_dir + "/tags.json";
    
    // Find any file matching *_tags.json in the session directory
    Glib::Dir dir(session_dir);
    for (const auto& entry : dir) {
        if (entry.size() > 10 && entry.compare(entry.size() - 10, 10, "_tags.json") == 0) {
            tags_json = session_dir + "/" + entry;
            break;
        }
    }

    std::cout << "Attempting to load session tags from: " << tags_json << std::endl;

    Json::Value root;
    if (!dc::Config::load_from_file(tags_json, root)) {
        std::cout << "No session tags found at: " << tags_json << std::endl;
        return;
    }

    if (!dc::Config::check_type(root, "dvrk_data:session_tags@1.0.0", tags_json)) {
        return;
    }

    std::cout << "Parsing unified tags.json..." << std::endl;

    auto find_frame = [&](long long ts) -> long long {
        if (m_data.frame_cpu_timestamps.empty()) return -1;
        auto it = std::lower_bound(m_data.frame_cpu_timestamps.begin(), m_data.frame_cpu_timestamps.end(), ts);

        long long idx = -1;
        if (it == m_data.frame_cpu_timestamps.end()) {
            idx = m_data.frame_cpu_timestamps.size() - 1;
        } else if (it == m_data.frame_cpu_timestamps.begin()) {
            idx = 0;
        } else {
            long long dist1 = std::abs((long long)*it - ts);
            long long dist2 = std::abs((long long)*(it - 1) - ts);
            if (dist1 < dist2) idx = std::distance(m_data.frame_cpu_timestamps.begin(), it);
            else idx = std::distance(m_data.frame_cpu_timestamps.begin(), it - 1);
        }

        if (idx != -1) {
            long long diff = std::abs((long long)m_data.frame_cpu_timestamps[idx] - ts);
            if (diff > 500000000LL) { // Increase to 500ms tolerance for UI latency
                return -1;
            }
        }
        return idx;
    };

    int count = 0;
    std::vector<std::string> missing_tags;
    std::vector<std::string> stages_to_add;
    std::vector<std::string> tags_to_add;

    if (root.isMember("stages")) {
        for (auto& s : root["stages"]) {
            std::string name = s["name"].asString();
            if (m_tag_buttons.count(name + "_start") == 0 || m_tag_buttons.count(name + "_end") == 0) {
                missing_tags.push_back(name + " (stage)");
                stages_to_add.push_back(name);
            }
        }
    }
    if (root.isMember("tags")) {
        for (auto const& name : root["tags"].getMemberNames()) {
            if (m_tag_buttons.count(name) == 0) {
                missing_tags.push_back(name);
                tags_to_add.push_back(name);
            }
        }
    }

    MissingTagsResult res = MissingTagsResult::IGNORE;
    if (!missing_tags.empty()) {
        res = show_missing_tags_dialog(missing_tags, "session file (" + tags_json + ")");
        if (res == MissingTagsResult::ACCEPT) {
            for (const auto& s : stages_to_add) {
                add_tag_row(s + "_start");
                add_tag_row(s + "_end");
            }
            for (const auto& t : tags_to_add) {
                add_tag_row(t);
            }
            m_tags_grid.show_all();
        }
    }

    if (root.isMember("stages")) {
        for (auto& s : root["stages"]) {
            std::string name = s["name"].asString();
            if (m_tag_buttons.count(name + "_start") == 0 || m_tag_buttons.count(name + "_end") == 0) continue;

            if (!s["start"].isObject() || !s["start"].isMember("cpu_ts") ||
                !s["end"].isObject() || !s["end"].isMember("cpu_ts")) {
                continue;
            }
            long long start_ts = s["start"]["cpu_ts"].asInt64();
            long long end_ts = s["end"]["cpu_ts"].asInt64();

            long long start_frame = find_frame(start_ts);
            long long end_frame = find_frame(end_ts);
            if (start_frame != -1) {
                m_data.frame_tags[start_frame].push_back(name + "_start");
                count++;
            }
            if (end_frame != -1) {
                m_data.frame_tags[end_frame].push_back(name + "_end");
                count++;
            }
        }
    }

    if (root.isMember("tags")) {
        for (auto const& name : root["tags"].getMemberNames()) {
            if (m_tag_buttons.count(name) == 0) continue;
            for (auto& t : root["tags"][name]) {
                if (!t.isObject() || !t.isMember("cpu_ts")) continue;
                long long tag_ts = t["cpu_ts"].asInt64();
                long long frame = find_frame(tag_ts);
                if (frame != -1) {
                    m_data.frame_tags[frame].push_back(name);
                    count++;
                }
            }
        }
    }

    m_data.session_tags_loaded = (count > 0);
    std::cout << "Successfully loaded " << count << " entries from session tags." << std::endl;
    std::string current_info = m_info_label.get_text();
    m_info_label.set_text(current_info + " | Session tags: " + std::to_string(count));
}
