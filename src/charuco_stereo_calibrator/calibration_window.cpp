// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "calibration_window.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace dvrk_data::calibration {

// ----------------------------------------------------------------------------
// VideoArea Implementation
// ----------------------------------------------------------------------------

VideoArea::VideoArea() {
    set_size_request(640, 360);
    add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK |
               Gdk::POINTER_MOTION_MASK | Gdk::SCROLL_MASK | Gdk::SMOOTH_SCROLL_MASK);
}

void VideoArea::set_image(const cv::Mat &bgr_mat) {
    if (bgr_mat.empty()) return;

    cv::Mat rgb;
    if (bgr_mat.channels() == 3) {
        cv::cvtColor(bgr_mat, rgb, cv::COLOR_BGR2RGB);
    } else if (bgr_mat.channels() == 1) {
        cv::cvtColor(bgr_mat, rgb, cv::COLOR_GRAY2RGB);
    } else {
        rgb = bgr_mat;
    }

    auto pixbuf = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, false, 8, rgb.cols, rgb.rows);
    int rowstride = pixbuf->get_rowstride();
    uint8_t *dest = pixbuf->get_pixels();

    for (int y = 0; y < rgb.rows; ++y) {
        std::memcpy(dest + y * rowstride, rgb.ptr(y), rgb.cols * 3);
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_img_width != rgb.cols || m_img_height != rgb.rows) reset_navigation();
        m_pixbuf = pixbuf;
        m_img_width = rgb.cols;
        m_img_height = rgb.rows;
    }

    queue_draw();
}

void VideoArea::set_active_epipolar_y(int y) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_active_y = y;
    queue_draw();
}

void VideoArea::reset_navigation() {
    m_zoom = 1.0;
    m_pan_x = m_pan_y = 0.0;
    m_dragging = false;
}

void VideoArea::set_navigation_enabled(bool enabled) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (enabled != m_navigation_enabled) reset_navigation();
    m_navigation_enabled = enabled;
    queue_draw();
}

void VideoArea::image_transform(double &scale, double &x, double &y) const {
    scale = std::min(static_cast<double>(get_allocated_width()) / m_img_width,
                     static_cast<double>(get_allocated_height()) / m_img_height) * m_zoom;
    x = (get_allocated_width() - m_img_width * scale) / 2.0 + m_pan_x;
    y = (get_allocated_height() - m_img_height * scale) / 2.0 + m_pan_y;
}

bool VideoArea::on_draw(const Cairo::RefPtr<Cairo::Context> &cr) {
    std::lock_guard<std::mutex> lock(m_mutex);
    int alloc_w = get_allocated_width();
    int alloc_h = get_allocated_height();

    // Background fill
    cr->set_source_rgb(0.06, 0.06, 0.06);
    cr->paint();

    if (!m_pixbuf || m_img_width <= 0 || m_img_height <= 0) {
        cr->set_source_rgb(0.5, 0.5, 0.5);
        cr->select_font_face("Sans", Cairo::FONT_SLANT_NORMAL, Cairo::FONT_WEIGHT_BOLD);
        cr->set_font_size(18.0);
        cr->move_to(alloc_w / 2.0 - 140, alloc_h / 2.0);
        cr->show_text("Waiting for GStreamer stereo stream...");
        return true;
    }

    double scale, dest_x, dest_y;
    image_transform(scale, dest_x, dest_y);
    const double dest_w = m_img_width * scale;
    // Draw through Cairo's transform; zoom never allocates an enlarged image.
    cr->save();
    cr->translate(dest_x, dest_y);
    cr->scale(scale, scale);
    Gdk::Cairo::set_source_pixbuf(cr, m_pixbuf, 0, 0);
    cr->rectangle(0, 0, m_img_width, m_img_height);
    cr->fill();
    cr->restore();

    // Draw active epipolar guideline if set
    if (!m_navigation_enabled && m_active_y >= 0 && m_active_y < m_img_height) {
        int screen_y = dest_y + static_cast<int>(m_active_y * scale);
        cr->set_source_rgb(1.0, 1.0, 0.0); // Bright Yellow
        cr->set_line_width(2.0);
        cr->move_to(dest_x, screen_y);
        cr->line_to(dest_x + dest_w, screen_y);
        cr->stroke();
    }

    return true;
}

bool VideoArea::on_button_press_event(GdkEventButton *event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_pixbuf || m_img_width <= 0 || m_img_height <= 0) return false;

    if (event->button != 1) return false;
    if (m_navigation_enabled) {
        if (event->type == GDK_2BUTTON_PRESS) {
            reset_navigation();
            queue_draw();
        } else {
            m_dragging = true;
            m_drag_x = event->x;
            m_drag_y = event->y;
        }
        return true;
    }
    double scale, dest_x, dest_y;
    image_transform(scale, dest_x, dest_y);
    const int clicked_img_y = static_cast<int>((event->y - dest_y) / scale);
    if (event->x >= dest_x && event->x < dest_x + m_img_width * scale &&
        event->y >= dest_y && clicked_img_y < m_img_height) {
        m_active_y = clicked_img_y;
        queue_draw();
    }
    return true;
}

bool VideoArea::on_button_release_event(GdkEventButton *event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_navigation_enabled || event->button != 1) return false;
    m_dragging = false;
    return true;
}

bool VideoArea::on_motion_notify_event(GdkEventMotion *event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_navigation_enabled || !m_dragging) return false;
    if (!(event->state & GDK_BUTTON1_MASK)) {
        m_dragging = false;
        return false;
    }
    m_pan_x += event->x - m_drag_x;
    m_pan_y += event->y - m_drag_y;
    m_drag_x = event->x;
    m_drag_y = event->y;
    queue_draw();
    return true;
}

bool VideoArea::on_scroll_event(GdkEventScroll *event) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_navigation_enabled || !m_pixbuf || m_img_width <= 0 || m_img_height <= 0)
        return false;
    double delta = 0;
    if (event->direction == GDK_SCROLL_UP) delta = -1;
    else if (event->direction == GDK_SCROLL_DOWN) delta = 1;
    else if (event->direction == GDK_SCROLL_SMOOTH) delta = event->delta_y;
    if (!std::isfinite(delta) || delta == 0) return true;
    double old_scale, old_x, old_y;
    image_transform(old_scale, old_x, old_y);
    if (old_scale <= 0) return true;
    m_zoom = std::clamp(m_zoom * std::pow(1.2, std::clamp(-delta, -20.0, 20.0)), 1.0, 32.0);
    if (m_zoom == 1.0) {
        reset_navigation();
    } else {
        double new_scale, new_x, new_y;
        image_transform(new_scale, new_x, new_y);
        // Keep the image point beneath the mouse fixed while zooming.
        m_pan_x += event->x - ((event->x - old_x) * new_scale / old_scale + new_x);
        m_pan_y += event->y - ((event->y - old_y) * new_scale / old_scale + new_y);
    }
    queue_draw();
    return true;
}

// ----------------------------------------------------------------------------
// CalibrationWindow Implementation
// ----------------------------------------------------------------------------

CalibrationWindow::CalibrationWindow(const std::string &board_config_file,
                                     const ReceiverConfig &receiver_cfg,
                                     const CharucoConfig &charuco_cfg)
    : m_board_config_file(board_config_file),
      m_receiver_config(receiver_cfg),
      m_charuco_config(charuco_cfg),
      m_main_box(Gtk::ORIENTATION_VERTICAL, 4),
      m_paned(Gtk::ORIENTATION_HORIZONTAL),
      m_video_box(Gtk::ORIENTATION_VERTICAL, 6),
      m_top_banner_box(Gtk::ORIENTATION_HORIZONTAL, 12),
      m_top_sep1(Gtk::ORIENTATION_VERTICAL),
      m_top_sep2(Gtk::ORIENTATION_VERTICAL),
      m_top_progress_box(Gtk::ORIENTATION_VERTICAL, 4),
      m_progress_spatial_box(Gtk::ORIENTATION_HORIZONTAL, 6),
      m_progress_tilt_box(Gtk::ORIENTATION_HORIZONTAL, 6),
      m_sidebar_box(Gtk::ORIENTATION_VERTICAL, 8),
      m_source_box(Gtk::ORIENTATION_VERTICAL, 4),
      m_board_box(Gtk::ORIENTATION_VERTICAL, 4),
      m_capture_box(Gtk::ORIENTATION_VERTICAL, 6),
      m_auto_capture_check("Auto-capture when stationary & novel"),
      m_capture_button("Capture Current Frame"),
      m_frame_buttons_box(Gtk::ORIENTATION_HORIZONTAL, 6),
      m_delete_frame_button("Delete Selected"),
      m_clear_frames_button("Clear All"),
      m_calib_box(Gtk::ORIENTATION_VERTICAL, 6),
      m_calibrate_button("Run Stereo Calibration"),
      m_filter_outliers_button("Filter Outliers & Recalibrate"),
      m_display_box(Gtk::ORIENTATION_VERTICAL, 6),
      m_save_button("Save Calibration...") {

    set_title("dVRK Stereo Endoscope Calibrator");
    set_default_size(1360, 800);
    set_border_width(6);

    // Initialize core components
    m_receiver = std::make_unique<GstStereoReceiver>();
    m_detector = std::make_unique<CharucoDetector>(m_charuco_config);
    m_coverage_tracker = std::make_unique<CoverageTracker>(6, 4);
    m_calibrator = std::make_unique<StereoCalibrator>();

    // Connect dispatchers
    m_dispatcher.connect(sigc::mem_fun(*this, &CalibrationWindow::on_frame_dispatched));
    m_calib_progress_dispatcher.connect(sigc::mem_fun(*this, &CalibrationWindow::on_calib_progress_dispatched));
    m_calib_finished_dispatcher.connect(sigc::mem_fun(*this, &CalibrationWindow::on_calib_finished_dispatched));

    // Layout assembly
    add(m_main_box);
    m_main_box.pack_start(m_paned, Gtk::PACK_EXPAND_WIDGET);

    // CSS styling for top banner and large progress bars
    try {
        auto css_provider = Gtk::CssProvider::create();
        css_provider->load_from_data(
            "progressbar trough { min-height: 28px; border-radius: 5px; background-color: #e9ecef; }\n"
            "progressbar progress { min-height: 28px; border-radius: 5px; background-color: #28a745; }\n"
            "progressbar text { font-size: 14px; font-weight: bold; color: #212529; }\n"
            ".top-banner-frame { background-color: #f8f9fa; border: 2px solid #dee2e6; border-radius: 8px; }\n"
        );
        Gtk::StyleContext::add_provider_for_screen(
            Gdk::Screen::get_default(), css_provider, GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
        m_top_banner_frame.get_style_context()->add_class("top-banner-frame");
    } catch (...) {}

    // --- Left Pane: Top Banner + Video Area ---
    m_top_banner_frame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    m_top_banner_frame.add(m_top_banner_box);
    m_top_banner_box.set_border_width(10);

    // 1. Large Status Guidance (Visible from across room)
    m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#6c757d'>INITIALIZING...</span>");
    m_big_status_label.set_alignment(0.0, 0.5);
    m_top_banner_box.pack_start(m_big_status_label, Gtk::PACK_EXPAND_WIDGET);

    m_top_banner_box.pack_start(m_top_sep1, Gtk::PACK_SHRINK);

    // 2. Large Sample Counter
    m_big_samples_label.set_markup("<span font_weight='bold' font_size='xx-large'>Samples: 0</span>");
    m_big_samples_label.set_alignment(0.5, 0.5);
    m_top_banner_box.pack_start(m_big_samples_label, Gtk::PACK_SHRINK);

    m_top_banner_box.pack_start(m_top_sep2, Gtk::PACK_SHRINK);

    // 3. Large Progress Bars (Spatial & Tilt)
    m_progress_spatial_title.set_markup("<b>Coverage:</b>");
    m_progress_spatial_box.pack_start(m_progress_spatial_title, Gtk::PACK_SHRINK);
    m_big_spatial_progress.set_size_request(200, 28);
    m_big_spatial_progress.set_show_text(true);
    m_progress_spatial_box.pack_start(m_big_spatial_progress, Gtk::PACK_EXPAND_WIDGET);
    m_top_progress_box.pack_start(m_progress_spatial_box, Gtk::PACK_SHRINK);

    m_progress_tilt_title.set_markup("<b>Tilt FOV:</b>");
    m_progress_tilt_box.pack_start(m_progress_tilt_title, Gtk::PACK_SHRINK);
    m_big_tilt_progress.set_size_request(200, 28);
    m_big_tilt_progress.set_show_text(true);
    m_progress_tilt_box.pack_start(m_big_tilt_progress, Gtk::PACK_EXPAND_WIDGET);
    m_top_progress_box.pack_start(m_progress_tilt_box, Gtk::PACK_SHRINK);

    m_top_banner_box.pack_start(m_top_progress_box, Gtk::PACK_SHRINK);

    m_video_box.pack_start(m_top_banner_frame, Gtk::PACK_SHRINK);
    m_video_box.pack_start(m_video_area, Gtk::PACK_EXPAND_WIDGET);
    m_paned.pack1(m_video_box, true, false);

    // --- Right Pane: Sidebar ---
    m_sidebar_scroll.set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_AUTOMATIC);
    m_sidebar_scroll.add(m_sidebar_box);
    m_sidebar_scroll.set_size_request(350, -1);
    m_paned.pack2(m_sidebar_scroll, false, false);

    // --- Sidebar Frame 1: Stream Source ---
    m_source_frame.set_label(" Video Stream Source ");
    m_source_frame.add(m_source_box);
    m_source_box.set_border_width(6);
    m_source_info_label.set_alignment(0.0, 0.5);
    m_fps_label.set_alignment(0.0, 0.5);
    m_source_box.pack_start(m_source_info_label, Gtk::PACK_SHRINK);
    m_source_box.pack_start(m_fps_label, Gtk::PACK_SHRINK);
    m_sidebar_box.pack_start(m_source_frame, Gtk::PACK_SHRINK);

    // --- Sidebar Frame 2: Target Board (Loaded from JSON) ---
    m_board_frame.set_label(" ChArUco Board Target ");
    m_board_frame.add(m_board_box);
    m_board_box.set_border_width(6);

    std::string filename_only = std::filesystem::path(m_board_config_file).filename().string();
    m_board_file_label.set_markup("Config: <b>" + filename_only + "</b>");
    m_board_file_label.set_alignment(0.0, 0.5);
    m_board_box.pack_start(m_board_file_label, Gtk::PACK_SHRINK);

    std::ostringstream board_info_ss;
    board_info_ss << "Grid: " << m_charuco_config.squares_x << " x " << m_charuco_config.squares_y << "\n"
                  << "Square: " << std::fixed << std::setprecision(1) << m_charuco_config.square_length_mm << " mm  |  "
                  << "Marker: " << m_charuco_config.marker_length_mm << " mm\n"
                  << "Dictionary: " << dictionary_to_string(m_charuco_config.dictionary_id);
    m_board_info_label.set_text(board_info_ss.str());
    m_board_info_label.set_alignment(0.0, 0.5);
    m_board_box.pack_start(m_board_info_label, Gtk::PACK_SHRINK);

    m_sidebar_box.pack_start(m_board_frame, Gtk::PACK_SHRINK);

    // --- Sidebar Frame 3: Capture Controls ---
    m_capture_frame.set_label(" Capture Controls ");
    m_capture_frame.add(m_capture_box);
    m_capture_box.set_border_width(6);

    m_stationarity_badge.set_markup("<b>Status: Initializing...</b>");
    m_capture_box.pack_start(m_stationarity_badge, Gtk::PACK_SHRINK);

    m_corners_count_label.set_text("Corners: 0");
    m_capture_box.pack_start(m_corners_count_label, Gtk::PACK_SHRINK);

    m_auto_capture_check.set_active(true);
    m_auto_capture_check.signal_toggled().connect([this] {
        m_auto_capture_enabled = m_auto_capture_check.get_active();
    });
    m_capture_box.pack_start(m_auto_capture_check, Gtk::PACK_SHRINK);

    m_capture_button.signal_clicked().connect(sigc::mem_fun(*this, &CalibrationWindow::on_capture_clicked));
    m_capture_box.pack_start(m_capture_button, Gtk::PACK_SHRINK);

    m_frames_count_label.set_text("Captured Frames: 0");
    m_capture_box.pack_start(m_frames_count_label, Gtk::PACK_SHRINK);

    m_frames_list_store = Gtk::ListStore::create(m_frame_cols);
    m_frames_tree_view.set_model(m_frames_list_store);
    m_frames_tree_view.append_column("ID", m_frame_cols.col_id);
    m_frames_tree_view.append_column("Corners", m_frame_cols.col_corners);
    m_frames_tree_view.append_column("Error (px)", m_frame_cols.col_error);
    m_frames_tree_view.get_selection()->signal_changed().connect([this] {
        if (m_calib_result.success && !m_is_calibrating.load() &&
            m_frames_tree_view.get_selection()->get_selected()) {
            if (m_display_mode == DisplayMode::GRID_VALIDATION) show_selected_grid();
            else m_display_mode_combo.set_active_id("validation");
        } else if (m_display_mode == DisplayMode::GRID_VALIDATION) {
            show_selected_grid();
        }
    });

    // A click on an already selected row should also reopen validation.
    m_frames_tree_view.signal_button_release_event().connect([this](GdkEventButton *event) {
        if (event->button != 1 || m_is_calibrating.load() || !m_calib_result.success)
            return false;
        Gtk::TreeModel::Path path;
        if (m_frames_tree_view.get_path_at_pos(static_cast<int>(event->x),
                                              static_cast<int>(event->y), path)) {
            if (m_display_mode == DisplayMode::GRID_VALIDATION) show_selected_grid();
            else m_display_mode_combo.set_active_id("validation");
        }
        return false;
    });

    m_frames_list_scroll.set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_AUTOMATIC);
    m_frames_list_scroll.set_size_request(-1, 100);
    m_frames_list_scroll.add(m_frames_tree_view);
    m_capture_box.pack_start(m_frames_list_scroll, Gtk::PACK_SHRINK);

    m_delete_frame_button.signal_clicked().connect(sigc::mem_fun(*this, &CalibrationWindow::on_delete_frame_clicked));
    m_clear_frames_button.signal_clicked().connect(sigc::mem_fun(*this, &CalibrationWindow::on_clear_frames_clicked));
    m_frame_buttons_box.pack_start(m_delete_frame_button, Gtk::PACK_EXPAND_WIDGET);
    m_frame_buttons_box.pack_start(m_clear_frames_button, Gtk::PACK_EXPAND_WIDGET);
    m_capture_box.pack_start(m_frame_buttons_box, Gtk::PACK_SHRINK);

    m_sidebar_box.pack_start(m_capture_frame, Gtk::PACK_SHRINK);

    // --- Sidebar Frame 4: Calibration Action & Quality ---
    m_calib_frame.set_label(" Stereo Calibration ");
    m_calib_frame.add(m_calib_box);
    m_calib_box.set_border_width(6);

    Gtk::Box *model_box = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 4));
    model_box->pack_start(*Gtk::manage(new Gtk::Label("Model:")), Gtk::PACK_SHRINK);
    m_dist_model_combo.append("plumb_bob", "Standard (5-param Plumb Bob)");
    m_dist_model_combo.append("rational", "Rational Polynomial (8-param)");
    m_dist_model_combo.set_active_id("plumb_bob");
    m_dist_model_combo.signal_changed().connect([this] {
        m_calib_options.model = (m_dist_model_combo.get_active_id() == "rational")
            ? DistortionModel::RATIONAL_POLYNOMIAL : DistortionModel::PLUMB_BOB;
    });
    model_box->pack_start(m_dist_model_combo, Gtk::PACK_EXPAND_WIDGET);
    m_calib_box.pack_start(*model_box, Gtk::PACK_SHRINK);

    m_calibrate_button.signal_clicked().connect(sigc::mem_fun(*this, &CalibrationWindow::on_calibrate_clicked));
    m_calib_box.pack_start(m_calibrate_button, Gtk::PACK_SHRINK);

    m_filter_outliers_button.set_sensitive(false);
    m_filter_outliers_button.signal_clicked().connect(sigc::mem_fun(*this, &CalibrationWindow::on_filter_outliers_clicked));
    m_calib_box.pack_start(m_filter_outliers_button, Gtk::PACK_SHRINK);

    m_calib_progress_bar.set_size_request(-1, 24);
    m_calib_progress_bar.set_show_text(true);
    m_calib_progress_bar.set_text("Not calibrated yet");
    m_calib_progress_bar.set_fraction(0.0);
    m_calib_box.pack_start(m_calib_progress_bar, Gtk::PACK_SHRINK);

    m_rms_label.set_markup("RMS Reprojection Error: <b>--</b>");
    m_baseline_label.set_markup("Baseline: <b>--</b>");
    m_epipolar_err_label.set_markup("Vertical Disparity Error: <b>--</b>");
    m_calib_box.pack_start(m_rms_label, Gtk::PACK_SHRINK);
    m_calib_box.pack_start(m_baseline_label, Gtk::PACK_SHRINK);
    m_calib_box.pack_start(m_epipolar_err_label, Gtk::PACK_SHRINK);

    m_sidebar_box.pack_start(m_calib_frame, Gtk::PACK_SHRINK);

    // --- Sidebar Frame 5: View Mode & Export ---
    m_display_frame.set_label(" View Mode & Export ");
    m_display_frame.add(m_display_box);
    m_display_box.set_border_width(6);

    Gtk::Box *view_box = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 4));
    view_box->pack_start(*Gtk::manage(new Gtk::Label("View:")), Gtk::PACK_SHRINK);
    m_display_mode_combo.append("tracking", "Live Tracking HUD");
    m_display_mode_combo.append("heatmap", "Coverage Heatmap Overlay");
    m_display_mode_combo.append("rectified", "Rectified Stereo (Epipolar)");
    m_display_mode_combo.append("anaglyph", "Red-Cyan 3D Anaglyph");
    m_display_mode_combo.append("validation", "Selected Sample: Grid Reprojection");
    m_display_mode_combo.set_active_id("tracking");
    m_display_mode_combo.signal_changed().connect(sigc::mem_fun(*this, &CalibrationWindow::on_display_mode_changed));
    view_box->pack_start(m_display_mode_combo, Gtk::PACK_EXPAND_WIDGET);
    m_display_box.pack_start(*view_box, Gtk::PACK_SHRINK);

    m_validation_label.set_text("After calibration, click a sample to validate its grid.");
    m_validation_label.set_line_wrap(true);
    m_validation_label.set_alignment(0.0, 0.5);
    m_display_box.pack_start(m_validation_label, Gtk::PACK_SHRINK);

    m_save_button.set_sensitive(false);
    m_save_button.signal_clicked().connect(sigc::mem_fun(*this, &CalibrationWindow::on_save_clicked));
    m_display_box.pack_start(m_save_button, Gtk::PACK_SHRINK);

    m_sidebar_box.pack_start(m_display_frame, Gtk::PACK_SHRINK);

    // Start video streaming (with non-blocking auto-reconnect)
    m_receiver->start(m_receiver_config);
    m_receiver->set_callback([this](const cv::Mat &l, const cv::Mat &r, uint64_t ts) {
        handle_new_frame(l, r, ts);
    });

    m_timer_connection = Glib::signal_timeout().connect(sigc::mem_fun(*this, &CalibrationWindow::on_timer_tick), 250);

    update_coverage_ui();
    show_all_children();
}

CalibrationWindow::~CalibrationWindow() {
    if (m_timer_connection.connected()) {
        m_timer_connection.disconnect();
    }
    if (m_calib_thread.joinable()) {
        m_calib_thread.join();
    }
    if (m_receiver) {
        m_receiver->stop();
    }
}

void CalibrationWindow::handle_new_frame(const cv::Mat &left, const cv::Mat &right, uint64_t ts) {
    // Stop video frame processing & marker detection while calibration is computing
    if (m_is_calibrating.load()) {
        return;
    }

    StereoDetection det;
    m_detector->detect_stereo(left, right, det);

    {
        std::lock_guard<std::mutex> lock(m_frame_mutex);
        m_current_left = left;
        m_current_right = right;
        m_current_detection = det;
        m_current_ts = ts;
        m_has_new_frame = true;
        m_last_frame_time = std::chrono::steady_clock::now();
        m_frame_is_stale = false;
    }

    m_dispatcher.emit();
}

void CalibrationWindow::on_frame_dispatched() {
    if (m_is_calibrating.load() || m_display_mode == DisplayMode::GRID_VALIDATION) {
        return;
    }

    cv::Mat left_copy, right_copy;
    StereoDetection det_copy;
    uint64_t ts = 0;

    {
        std::lock_guard<std::mutex> lock(m_frame_mutex);
        if (!m_has_new_frame) return;
        left_copy = m_current_left;
        right_copy = m_current_right;
        det_copy = m_current_detection;
        ts = m_current_ts;
        m_has_new_frame = false;
    }
    (void)ts;

    // Check connection status
    auto stats = m_receiver->get_stats();
    if (!stats.is_connected) {
        m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#888888'>WAITING FOR SOURCE...</span>");
        m_stationarity_badge.set_markup("<b>Status: " + Glib::Markup::escape_text(stats.status_message) + "</b>");
        return;
    }

    // Check for stream resolution mismatch against collected frames
    if (m_calibrator->frame_count() > 0) {
        cv::Size exp_sz = m_calibrator->expected_image_size();
        if (exp_sz.width > 0 && (left_copy.cols != exp_sz.width || left_copy.rows != exp_sz.height)) {
            m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#dc3545'>RESOLUTION CHANGED!</span>");
            m_stationarity_badge.set_markup("<span foreground='#dc3545'><b>Stream is " +
                std::to_string(left_copy.cols) + "x" + std::to_string(left_copy.rows) +
                ", expected " + std::to_string(exp_sz.width) + "x" + std::to_string(exp_sz.height) +
                ". Clear frames to reset.</b></span>");
            m_video_area.set_image(left_copy);
            return;
        }
    }

    // Decrement auto-capture cooldown
    if (m_auto_capture_cooldown > 0) {
        m_auto_capture_cooldown--;
    }

    // 1. Evaluate Detection & Guidance
    if (m_auto_capture_cooldown > 4) {
        m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#28a745'>✓ CAPTURED SAMPLE #" +
                                       std::to_string(m_calibrator->frame_count()) + "!</span>");
        m_stationarity_badge.set_markup("<span foreground='#28a745'><b>Captured! Move to next pose...</b></span>");
    } else if (!det_copy.is_stereo_valid) {
        if (!det_copy.common_ids.empty()) {
            m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#dc3545'>PARTIAL VIEW (" +
                                           std::to_string(det_copy.common_ids.size()) + "/6 CORNERS)</span>");
            m_stationarity_badge.set_markup("<span foreground='#dc3545'><b>Need >= 6 corners spanning 2D</b></span>");
        } else {
            m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#dc3545'>SEARCHING FOR BOARD...</span>");
            m_stationarity_badge.set_markup("<span foreground='#dc3545'><b>Target not in stereo view</b></span>");
        }
        m_still_frames_count = 0;
    } else if (!det_copy.is_stationary) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << det_copy.motion_speed;
        m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#fd7e14'>HOLD STILL (" + ss.str() + " px/f)</span>");
        m_stationarity_badge.set_markup("<span foreground='#fd7e14'><b>Moving (" + ss.str() + " px/f)</b></span>");
        m_still_frames_count = 0;
    } else {
        std::string novel_reason;
        bool is_novel = m_coverage_tracker->is_novel_coverage(
            det_copy.left_common_corners, det_copy.common_ids,
            det_copy.object_points, left_copy.size(), &novel_reason);

        if (is_novel) {
            m_still_frames_count++;
            m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#28a745'>STABLE — READY</span>");
            m_stationarity_badge.set_markup("<span foreground='#28a745'><b>Stationary (" + Glib::Markup::escape_text(novel_reason) + ")</b></span>");

            // Auto-capture when stationary for >= 2 frames and cooled down
            if (m_auto_capture_enabled && m_still_frames_count >= 2 && m_auto_capture_cooldown <= 0) {
                // Capture the same observation that passed the novelty check.
                if (capture_frame(left_copy, right_copy, det_copy)) {
                    m_auto_capture_cooldown = 10;
                }
            }
        } else {
            m_still_frames_count = 0;
            m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#17a2b8'>MOVE TARGET (Tilt or shift)</span>");
            m_stationarity_badge.set_markup("<span foreground='#17a2b8'><b>Duplicate pose (Change angle/distance)</b></span>");
        }
    }

    m_corners_count_label.set_text("Stereo Corners: " + std::to_string(det_copy.common_ids.size()));

    // 2. Render Selected Display Mode
    cv::Mat display_img;

    if (m_display_mode == DisplayMode::RECTIFIED_EPIPOLAR && m_calib_result.success) {
        cv::Mat rect_l, rect_r;
        StereoCalibrator::rectify_stereo(left_copy, right_copy, rect_l, rect_r, m_calib_result);
        display_img = StereoCalibrator::make_side_by_side_epipolar(rect_l, rect_r, 32);
    } else if (m_display_mode == DisplayMode::ANAGLYPH_3D && m_calib_result.success) {
        cv::Mat rect_l, rect_r;
        StereoCalibrator::rectify_stereo(left_copy, right_copy, rect_l, rect_r, m_calib_result);
        display_img = StereoCalibrator::make_anaglyph(rect_l, rect_r);
    } else if (m_display_mode == DisplayMode::COVERAGE_HEATMAP) {
        cv::Mat l_heat = left_copy.clone();
        m_coverage_tracker->draw_heatmap_overlay(l_heat, 0.45);
        m_detector->draw_stereo_detection(l_heat, right_copy, det_copy);
        cv::hconcat(l_heat, right_copy, display_img);
    } else { // LIVE_TRACKING
        cv::Mat l_annot = left_copy.clone();
        cv::Mat r_annot = right_copy.clone();
        m_detector->draw_stereo_detection(l_annot, r_annot, det_copy);
        cv::hconcat(l_annot, r_annot, display_img);
    }

    m_video_area.set_image(display_img);

    // 3. Update Status and Diagnostics
    update_stats_ui();
}

void CalibrationWindow::on_capture_clicked() {
    cv::Mat l, r;
    StereoDetection det;
    {
        std::lock_guard<std::mutex> lock(m_frame_mutex);
        if (m_frame_is_stale) return;
        l = m_current_left;
        r = m_current_right;
        det = m_current_detection;
    }

    capture_frame(l, r, det);
}

bool CalibrationWindow::capture_frame(const cv::Mat &l, const cv::Mat &r,
                                      const StereoDetection &det) {
    if (m_is_calibrating.load() || !det.is_stereo_valid) return false;
    std::string reason;
    if (!m_coverage_tracker->is_novel_coverage(det.left_common_corners, det.common_ids,
                                              det.object_points, l.size(), &reason)) {
        m_stationarity_badge.set_text(reason);
        return false;
    }

    int frame_id = m_calibrator->add_frame(l, r, det);
    if (frame_id > 0) {
        m_coverage_tracker->add_frame(det.left_common_corners, det.common_ids,
                                      det.object_points, l.size(),
                                      det.left.rvec, det.left.pose_valid);
        m_still_frames_count = 0;

        // Add to TreeView
        auto row = *(m_frames_list_store->append());
        row[m_frame_cols.col_id] = frame_id;
        row[m_frame_cols.col_corners] = static_cast<int>(det.common_ids.size());
        row[m_frame_cols.col_error] = "--";

        // Frame collection changed: disable outlier filtering until recalibrated
        m_filter_outliers_button.set_sensitive(false);

        update_coverage_ui();
        m_frames_count_label.set_text("Captured Frames: " + std::to_string(m_calibrator->frame_count()));
        m_big_samples_label.set_markup("<span font_weight='bold' font_size='xx-large'>Samples: " +
                                       std::to_string(m_calibrator->frame_count()) + "</span>");
        update_calib_status_ui();
    }
    return frame_id > 0;
}

void CalibrationWindow::on_delete_frame_clicked() {
    auto selected = m_frames_tree_view.get_selection()->get_selected();
    if (selected) {
        int id = (*selected)[m_frame_cols.col_id];
        m_calibrator->remove_frame(id);
        m_frames_list_store->erase(selected);
        m_frames_count_label.set_text("Captured Frames: " + std::to_string(m_calibrator->frame_count()));
        m_big_samples_label.set_markup("<span font_weight='bold' font_size='xx-large'>Samples: " +
                                       std::to_string(m_calibrator->frame_count()) + "</span>");

        // Rebuild coverage tracker from retained frames
        m_coverage_tracker->reset();
        for (const auto &f : m_calibrator->get_frames()) {
            m_coverage_tracker->add_frame(f.left_corners, f.charuco_ids, f.object_points,
                                        f.left_image.size(), f.rvec, f.pose_valid);
        }

        // Frame collection changed: disable outlier filtering until recalibrated
        m_filter_outliers_button.set_sensitive(false);

        update_coverage_ui();
        update_calib_status_ui();
    }
}

void CalibrationWindow::on_clear_frames_clicked() {
    m_still_frames_count = 0;
    m_auto_capture_cooldown = 0;
    m_calibrator->clear_frames();
    m_coverage_tracker->reset();
    m_frames_list_store->clear();
    m_frames_count_label.set_text("Captured Frames: 0");
    m_big_samples_label.set_markup("<span font_weight='bold' font_size='xx-large'>Samples: 0</span>");
    m_calib_result = StereoCalibrationResult();
    m_calibrated_frames.clear();
    m_pending_calibration_frames.clear();
    m_rms_label.set_markup("RMS Reprojection Error: <b>--</b>");
    m_baseline_label.set_markup("Baseline: <b>--</b>");
    m_epipolar_err_label.set_markup("Vertical Disparity Error: <b>--</b>");
    m_save_button.set_sensitive(false);
    m_filter_outliers_button.set_sensitive(false);
    if (m_display_mode_combo.get_active_id() == "rectified" ||
        m_display_mode_combo.get_active_id() == "anaglyph" ||
        m_display_mode_combo.get_active_id() == "validation") {
        m_display_mode_combo.set_active_id("tracking");
    }
    update_coverage_ui();
    update_calib_status_ui();
}

void CalibrationWindow::on_calibrate_clicked() {
    if (m_is_calibrating.load()) {
        return;
    }

    if (m_calibrator->frame_count() < 5) {
        Gtk::MessageDialog dialog(*this, "Insufficient Calibration Frames", false,
                                  Gtk::MESSAGE_WARNING, Gtk::BUTTONS_OK);
        dialog.set_secondary_text("Please collect at least 5 frames across diverse positions, depths, and angles before calibrating.");
        dialog.run();
        return;
    }

    if (m_calib_thread.joinable()) {
        m_calib_thread.join();
    }

    m_pending_calibration_frames = m_calibrator->get_frames();
    m_is_calibrating = true;
    m_display_mode_combo.set_sensitive(false);
    m_calibrate_button.set_sensitive(false);
    m_filter_outliers_button.set_sensitive(false);
    m_dist_model_combo.set_sensitive(false);
    m_capture_button.set_sensitive(false);
    m_delete_frame_button.set_sensitive(false);
    m_clear_frames_button.set_sensitive(false);
    m_save_button.set_sensitive(false);

    m_calib_progress_bar.set_fraction(0.05);
    m_calib_progress_bar.set_text("Starting optimization...");
    m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#007bff'>COMPUTING CALIBRATION...</span>");

    CalibratorOptions options_snapshot = m_calib_options;
    m_calib_thread = std::thread([this, options_snapshot]() {
        std::string err_msg;
        StereoCalibrationResult result;

        auto progress_cb = [this](double frac, const std::string &stage) {
            {
                std::lock_guard<std::mutex> lock(m_calib_state_mutex);
                m_calib_progress_frac = frac;
                m_calib_progress_stage = stage;
            }
            m_calib_progress_dispatcher.emit();
        };

        bool ok = m_calibrator->calibrate(options_snapshot, result, err_msg, progress_cb);

        {
            std::lock_guard<std::mutex> lock(m_calib_state_mutex);
            m_calib_done_ok = ok;
            m_calib_done_result = result;
            m_calib_done_error = err_msg;
        }

        m_calib_finished_dispatcher.emit();
    });
}

void CalibrationWindow::on_calib_progress_dispatched() {
    double frac = 0.0;
    std::string stage;
    {
        std::lock_guard<std::mutex> lock(m_calib_state_mutex);
        frac = m_calib_progress_frac;
        stage = m_calib_progress_stage;
    }

    m_calib_progress_bar.set_fraction(frac);
    std::ostringstream ss;
    ss << static_cast<int>(frac * 100) << "% - " << stage;
    m_calib_progress_bar.set_text(ss.str());
    m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#007bff'>" + Glib::Markup::escape_text(stage) + "...</span>");
}

void CalibrationWindow::on_calib_finished_dispatched() {
    if (m_calib_thread.joinable()) {
        m_calib_thread.join();
    }
    m_is_calibrating = false;
    m_display_mode_combo.set_sensitive(true);

    bool ok = false;
    StereoCalibrationResult result;
    std::string err_msg;
    {
        std::lock_guard<std::mutex> lock(m_calib_state_mutex);
        ok = m_calib_done_ok;
        result = m_calib_done_result;
        err_msg = m_calib_done_error;
    }

    if (!ok) m_pending_calibration_frames.clear();
    m_calibrate_button.set_sensitive(true);
    m_dist_model_combo.set_sensitive(true);
    m_capture_button.set_sensitive(true);
    m_delete_frame_button.set_sensitive(true);
    m_clear_frames_button.set_sensitive(true);
    m_filter_outliers_button.set_sensitive(ok && m_calibrator->frame_count() > 5);

    if (ok) {
        m_calib_result = result;
        m_calibrated_frames = std::move(m_pending_calibration_frames);
        m_calib_progress_bar.set_fraction(1.0);
        m_calib_progress_bar.set_text("Calibration Complete (100%)");
        m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#28a745'>CALIBRATION SUCCESSFUL!</span>");

        std::ostringstream ss_rms, ss_base, ss_epi;
        ss_rms << std::fixed << std::setprecision(3) << m_calib_result.rms_error << " px";
        ss_base << std::fixed << std::setprecision(2) << m_calib_result.baseline_mm << " mm";
        ss_epi << std::fixed << std::setprecision(3) << m_calib_result.avg_epipolar_error_px << " px";

        m_rms_label.set_markup("RMS Reprojection Error: <b><span foreground='#5cb85c'>" + ss_rms.str() + "</span></b>");
        m_baseline_label.set_markup("Baseline: <b>" + ss_base.str() + "</b>");
        m_epipolar_err_label.set_markup("Vertical Disparity Error: <b>" + ss_epi.str() + "</b>");

        // Update TreeView per-frame errors by unique frame ID
        auto children = m_frames_list_store->children();
        for (auto it = children.begin(); it != children.end(); ++it) {
            int id = (*it)[m_frame_cols.col_id];
            auto err_it = m_calib_result.per_frame_error_by_id.find(id);
            if (err_it != m_calib_result.per_frame_error_by_id.end()) {
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2) << err_it->second;
                (*it)[m_frame_cols.col_error] = ss.str();
            }
        }

        m_save_button.set_sensitive(true);
        m_display_mode_combo.set_active_id("rectified");
    } else {
        m_calib_progress_bar.set_fraction(0.0);
        m_calib_progress_bar.set_text("Calibration Failed");
        m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#dc3545'>CALIBRATION FAILED</span>");

        Gtk::MessageDialog dialog(*this, "Calibration Failed", false,
                                  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
        dialog.set_secondary_text(err_msg);
        dialog.run();
    }
}

void CalibrationWindow::on_filter_outliers_clicked() {
    if (m_is_calibrating.load()) return;
    if (!m_calib_result.success || m_calib_result.per_frame_rms.empty()) {
        return;
    }

    const auto &frames = m_calibrator->get_frames();
    if (frames.size() <= 5) {
        Gtk::MessageDialog dialog(*this, "Cannot Filter Outliers", false,
                                  Gtk::MESSAGE_INFO, Gtk::BUTTONS_OK);
        dialog.set_secondary_text("A minimum of 5 frames is required for calibration. Current collection has " +
                                  std::to_string(frames.size()) + " frames.");
        dialog.run();
        return;
    }

    // Collect errors of currently retained frames using unique frame ID
    std::vector<double> current_frame_errors;
    std::vector<std::pair<double, int>> frame_error_pairs; // (error, frame_id)
    for (const auto &frame : frames) {
        auto it = m_calib_result.per_frame_error_by_id.find(frame.id);
        if (it != m_calib_result.per_frame_error_by_id.end()) {
            current_frame_errors.push_back(it->second);
            frame_error_pairs.emplace_back(it->second, frame.id);
        }
    }

    if (current_frame_errors.size() <= 5) {
        Gtk::MessageDialog dialog(*this, "Cannot Filter Outliers", false,
                                  Gtk::MESSAGE_INFO, Gtk::BUTTONS_OK);
        dialog.set_secondary_text("Fewer than 5 calibrated frames available.");
        dialog.run();
        return;
    }

    // Compute median reprojection error across currently evaluated frames
    std::vector<double> sorted_errors = current_frame_errors;
    std::sort(sorted_errors.begin(), sorted_errors.end());
    size_t n = sorted_errors.size();
    double median_err = (n % 2 == 1)
        ? sorted_errors[n / 2]
        : 0.5 * (sorted_errors[n / 2 - 1] + sorted_errors[n / 2]);

    double threshold = 1.5 * median_err;

    // Identify candidate outliers exceeding 1.5x median error
    std::vector<std::pair<double, int>> candidate_outliers; // (error, frame_id)
    for (const auto &pair : frame_error_pairs) {
        if (pair.first > threshold) {
            candidate_outliers.push_back(pair);
        }
    }

    if (candidate_outliers.empty()) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3)
           << "Median error: " << median_err << " px\n"
           << "Cutoff threshold (1.5x): " << threshold << " px\n\n"
           << "All " << frames.size() << " frames are consistent with low reprojection error.";
        Gtk::MessageDialog dialog(*this, "No Outliers Detected", false,
                                  Gtk::MESSAGE_INFO, Gtk::BUTTONS_OK);
        dialog.set_secondary_text(ss.str());
        dialog.run();
        return;
    }

    // Safety: ensure dropping outliers never leaves fewer than 5 frames
    std::sort(candidate_outliers.rbegin(), candidate_outliers.rend()); // Highest error first
    size_t max_allowed_drops = frames.size() - 5;
    size_t num_to_drop = std::min(candidate_outliers.size(), max_allowed_drops);

    std::vector<int> dropped_ids;
    for (size_t i = 0; i < num_to_drop; ++i) {
        dropped_ids.push_back(candidate_outliers[i].second);
    }

    // Remove dropped frames from StereoCalibrator and TreeView
    for (int id : dropped_ids) {
        m_calibrator->remove_frame(id);

        auto children = m_frames_list_store->children();
        for (auto it = children.begin(); it != children.end(); ++it) {
            if ((*it)[m_frame_cols.col_id] == id) {
                m_frames_list_store->erase(it);
                break;
            }
        }
    }

    // Rebuild coverage tracker from retained frames
    m_coverage_tracker->reset();
    for (const auto &f : m_calibrator->get_frames()) {
        m_coverage_tracker->add_frame(f.left_corners, f.charuco_ids, f.object_points,
                                        f.left_image.size(), f.rvec, f.pose_valid);
    }

    m_frames_count_label.set_text("Captured Frames: " + std::to_string(m_calibrator->frame_count()));
    m_big_samples_label.set_markup("<span font_weight='bold' font_size='xx-large'>Samples: " +
                                   std::to_string(m_calibrator->frame_count()) + "</span>");
    update_coverage_ui();

    // Automatically trigger recalibration with the clean dataset
    on_calibrate_clicked();
}

void CalibrationWindow::on_save_clicked() {
    if (!m_calib_result.success) return;

    Gtk::FileChooserDialog dialog(*this, "Save Calibration Output",
                                  Gtk::FILE_CHOOSER_ACTION_SELECT_FOLDER);
    dialog.add_button("Cancel", Gtk::RESPONSE_CANCEL);
    dialog.add_button("Save", Gtk::RESPONSE_OK);
    Gtk::CheckButton save_raw_check("Also save raw calibration images in a dated directory");
    save_raw_check.set_active(false);
    save_raw_check.show();
    dialog.set_extra_widget(save_raw_check);

    if (dialog.run() == Gtk::RESPONSE_OK) {
        std::string folder = dialog.get_filename();

        // Save ROS 2 CameraInfo pair (left.yaml and right.yaml)
        bool ros_ok = CalibrationIO::save_ros_camera_info_pair(folder, m_calib_result);

        // Save consolidated dVRK calibration YAML
        std::string dvrk_file = (std::filesystem::path(folder) / "stereo_calibration.yaml").string();
        bool dvrk_ok = CalibrationIO::save_stereo_calibration_file(dvrk_file, m_calib_result);

        std::string archive_dir, archive_error;
        bool archive_ok = true;
        if (ros_ok && dvrk_ok && save_raw_check.get_active()) {
            archive_ok = CalibrationIO::save_calibration_archive(
                folder, m_calibrated_frames, m_charuco_config, archive_dir, archive_error);
        }

        if (ros_ok && dvrk_ok && archive_ok) {
            Gtk::MessageDialog success_dialog(*this, "Calibration Saved", false,
                                              Gtk::MESSAGE_INFO, Gtk::BUTTONS_OK);
            std::string message = "Exported left.yaml, right.yaml, and stereo_calibration.yaml to:\n" + folder;
            if (!archive_dir.empty()) message += "\n\nRaw calibration images, board.json, and YAML copies saved to:\n" + archive_dir;
            success_dialog.set_secondary_text(message);
            success_dialog.run();
        } else {
            Gtk::MessageDialog err_dialog(*this, "Save Failed", false,
                                          Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
            err_dialog.set_secondary_text(ros_ok && dvrk_ok
                ? "Calibration YAML files were saved, but the raw image archive failed:\n" + archive_error
                : "Failed to write calibration files to: " + folder);
            err_dialog.run();
        }
    }
}

void CalibrationWindow::on_display_mode_changed() {
    std::string id = m_display_mode_combo.get_active_id();
    m_video_area.set_navigation_enabled(id == "validation");
    m_capture_button.set_sensitive(!m_is_calibrating.load() && id != "validation");
    if (id != "validation")
        m_validation_label.set_text("After calibration, click a sample to validate its grid.");
    if (id == "validation") {
        m_display_mode = DisplayMode::GRID_VALIDATION;
        m_video_area.set_active_epipolar_y(-1);
        show_selected_grid();
    } else if (id == "heatmap") {
        m_display_mode = DisplayMode::COVERAGE_HEATMAP;
    } else if (id == "rectified") {
        m_display_mode = DisplayMode::RECTIFIED_EPIPOLAR;
    } else if (id == "anaglyph") {
        m_display_mode = DisplayMode::ANAGLYPH_3D;
    } else {
        m_display_mode = DisplayMode::LIVE_TRACKING;
    }
}

void CalibrationWindow::show_selected_grid() {
    if (m_is_calibrating.load()) return;
    auto selected = m_frames_tree_view.get_selection()->get_selected();
    if (!m_calib_result.success || !selected) {
        m_validation_label.set_text(m_calib_result.success
            ? "Select a sample in the captured frames list."
            : "Run stereo calibration, then click a sample to validate its grid.");
        cv::Mat blank(360, 1280, CV_8UC3, cv::Scalar(16, 16, 16));
        m_video_area.set_image(blank);
        return;
    }
    const int id = (*selected)[m_frame_cols.col_id];
    const auto &frames = m_calibrator->get_frames();
    const auto it = std::find_if(frames.begin(), frames.end(),
                                 [id](const CalibrationFrame &frame) { return frame.id == id; });
    if (it == frames.end()) return;
    GridReprojection projection;
    std::string error;
    cv::Mat display;
    if (StereoCalibrator::reproject_grid(*it, m_charuco_config, m_calib_result, projection, error)) {
        cv::hconcat(projection.left_overlay, projection.right_overlay, display);
        std::ostringstream ss;
        ss << "Sample #" << id << " | RMS L: " << std::fixed << std::setprecision(3)
           << projection.left_rms_px << " px, R: " << projection.right_rms_px
           << " px\nGreen: projected grid. Yellow: detected corners. Red: residuals."
           << "\nPose fitted in left eye; right uses stereo R/T."
           << "\nWheel: zoom. Left drag: pan. Double-click: fit.";
        m_validation_label.set_text(ss.str());
    } else {
        cv::hconcat(it->left_image, it->right_image, display);
        m_validation_label.set_text(error);
    }
    m_video_area.set_image(display);
    m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#007bff'>VALIDATING SAMPLE #" +
                                  std::to_string(id) + "</span>");
}

void CalibrationWindow::update_coverage_ui() {
    CoverageStats stats = m_coverage_tracker->get_stats();

    m_big_spatial_progress.set_fraction(stats.spatial_coverage_ratio);
    m_big_spatial_progress.set_text(std::to_string(static_cast<int>(stats.spatial_coverage_ratio * 100)) + "%");

    m_big_tilt_progress.set_fraction(stats.tilt_coverage_ratio);
    m_big_tilt_progress.set_text(std::to_string(static_cast<int>(stats.tilt_coverage_ratio * 100)) + "%");
}

void CalibrationWindow::update_stats_ui() {
    auto stats = m_receiver->get_stats();
    const std::string &src_name = m_receiver_config.stereo_input;

    if (stats.is_connected) {
        m_source_info_label.set_markup("Source: <b>" + Glib::Markup::escape_text(src_name) + "</b> (<span foreground='#5cb85c'>Connected</span>)");
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << stats.fps << " fps ("
           << stats.single_eye_width << "x" << stats.single_eye_height << " per eye)";
        m_fps_label.set_text(ss.str());
    } else {
        m_source_info_label.set_markup("Source: <b>" + Glib::Markup::escape_text(src_name) + "</b> (<span foreground='#d9534f'>" + Glib::Markup::escape_text(stats.status_message) + "</span>)");
        m_fps_label.set_text("-- fps (Waiting for stream...)");
    }
}

void CalibrationWindow::update_calib_status_ui() {
    if (m_is_calibrating.load()) {
        return;
    }
    if (!m_calib_result.success) {
        m_calib_progress_bar.set_fraction(0.0);
        if (m_calibrator->frame_count() >= 5) {
            m_calib_progress_bar.set_text("Ready to calibrate");
        } else {
            m_calib_progress_bar.set_text("Not calibrated yet");
        }
    }
}

bool CalibrationWindow::on_timer_tick() {
    update_stats_ui();

    if (m_is_calibrating.load() || m_display_mode == DisplayMode::GRID_VALIDATION) {
        return true;
    }

    auto now = std::chrono::steady_clock::now();
    double time_since_frame;
    {
        std::lock_guard<std::mutex> lock(m_frame_mutex);
        time_since_frame = (m_last_frame_time.time_since_epoch().count() > 0)
            ? std::chrono::duration<double>(now - m_last_frame_time).count()
            : 999.0;
        if (time_since_frame > 1.0) {
            m_frame_is_stale = true;
            m_current_detection = StereoDetection();
        }
    }

    // If stream has stopped producing frames (> 1.0 second since last frame)
    if (time_since_frame > 1.0) {
        m_still_frames_count = 0;

        auto stats = m_receiver->get_stats();
        if (!stats.is_connected) {
            m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#888888'>WAITING FOR SOURCE...</span>");
            m_stationarity_badge.set_markup("<b>Status: " + Glib::Markup::escape_text(stats.status_message) + "</b>");
        } else {
            m_big_status_label.set_markup("<span font_weight='bold' font_size='xx-large' foreground='#dc3545'>STREAM PAUSED / NO FRAMES</span>");
            m_stationarity_badge.set_markup("<span foreground='#dc3545'><b>No frames received for " +
                                            std::to_string(static_cast<int>(time_since_frame)) + "s</b></span>");
        }
    }

    return true;
}

} // namespace dvrk_data::calibration
