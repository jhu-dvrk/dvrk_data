// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#ifndef DVRK_DATA_CALIBRATION_CALIBRATION_WINDOW_HPP
#define DVRK_DATA_CALIBRATION_CALIBRATION_WINDOW_HPP

#include "dvrk_data/calibration/charuco_detector.hpp"
#include "dvrk_data/calibration/coverage_tracker.hpp"
#include "dvrk_data/calibration/gst_stereo_receiver.hpp"
#include "dvrk_data/calibration/stereo_calibrator.hpp"
#include "dvrk_data/calibration/calibration_io.hpp"
#include "dvrk_data/calibration/board_config.hpp"

#include <gtkmm.h>
#include <mutex>
#include <memory>
#include <string>
#include <chrono>

namespace dvrk_data::calibration {

enum class DisplayMode {
    LIVE_TRACKING,
    COVERAGE_HEATMAP,
    RECTIFIED_EPIPOLAR,
    GRID_VALIDATION,
    ANAGLYPH_3D
};

class VideoArea : public Gtk::DrawingArea {
public:
    VideoArea();
    ~VideoArea() override = default;

    void set_image(const cv::Mat &bgr_mat);
    void set_active_epipolar_y(int y);
    void set_navigation_enabled(bool enabled);

protected:
    bool on_draw(const Cairo::RefPtr<Cairo::Context> &cr) override;
    bool on_button_press_event(GdkEventButton *event) override;
    bool on_button_release_event(GdkEventButton *event) override;
    bool on_motion_notify_event(GdkEventMotion *event) override;
    bool on_scroll_event(GdkEventScroll *event) override;

private:
    void image_transform(double &scale, double &x, double &y) const;
    void reset_navigation(); // Caller holds m_mutex.
    bool m_navigation_enabled = false;
    bool m_dragging = false;
    double m_zoom = 1.0;
    double m_pan_x = 0.0, m_pan_y = 0.0;
    double m_drag_x = 0.0, m_drag_y = 0.0;
    std::mutex m_mutex;
    Glib::RefPtr<Gdk::Pixbuf> m_pixbuf;
    int m_img_width = 0;
    int m_img_height = 0;
    int m_active_y = -1;
};

class CalibrationWindow : public Gtk::Window {
public:
    CalibrationWindow(const std::string &board_config_file,
                      const ReceiverConfig &receiver_cfg = ReceiverConfig(),
                      const CharucoConfig &charuco_cfg = CharucoConfig());
    ~CalibrationWindow() override;

private:
    // UI Signal Handlers
    void on_frame_dispatched();
    void on_capture_clicked();
    void on_delete_frame_clicked();
    void on_clear_frames_clicked();
    void on_calibrate_clicked();
    void on_filter_outliers_clicked();
    void on_calib_progress_dispatched();
    void on_calib_finished_dispatched();
    void on_save_clicked();
    void on_display_mode_changed();
    void show_selected_grid();

    // Stream frame callback
    void handle_new_frame(const cv::Mat &left, const cv::Mat &right, uint64_t ts);

    // Helpers
    void update_coverage_ui();
    void update_stats_ui();
    void update_calib_status_ui();
    bool on_timer_tick();
    bool capture_frame(const cv::Mat &left, const cv::Mat &right,
                       const StereoDetection &detection);

    // Configuration & Core Objects
    std::string m_board_config_file;
    ReceiverConfig m_receiver_config;
    CharucoConfig m_charuco_config;
    CalibratorOptions m_calib_options;

    std::unique_ptr<GstStereoReceiver> m_receiver;
    std::unique_ptr<CharucoDetector> m_detector;
    std::unique_ptr<CoverageTracker> m_coverage_tracker;
    std::unique_ptr<StereoCalibrator> m_calibrator;

    StereoCalibrationResult m_calib_result;
    // Retain the exact inputs of the last successful calibration for export.
    std::vector<CalibrationFrame> m_pending_calibration_frames;
    std::vector<CalibrationFrame> m_calibrated_frames;
    DisplayMode m_display_mode = DisplayMode::LIVE_TRACKING;

    // Frame synchronization & dispatch
    Glib::Dispatcher m_dispatcher;
    sigc::connection m_timer_connection;
    std::mutex m_frame_mutex;
    cv::Mat m_current_left;
    cv::Mat m_current_right;
    StereoDetection m_current_detection;
    uint64_t m_current_ts = 0;
    bool m_has_new_frame = false;
    std::chrono::steady_clock::time_point m_last_frame_time;
    bool m_frame_is_stale = true;

    // Auto capture state
    int m_still_frames_count = 0;
    int m_auto_capture_cooldown = 0;
    bool m_auto_capture_enabled = true;

    // --- GUI Widgets ---
    Gtk::Box m_main_box;
    Gtk::Paned m_paned;

    // Video pane (with Top Banner above video)
    Gtk::Box m_video_box;
    Gtk::Frame m_top_banner_frame;
    Gtk::Box m_top_banner_box;
    Gtk::Label m_big_status_label;
    Gtk::Separator m_top_sep1;
    Gtk::Label m_big_samples_label;
    Gtk::Separator m_top_sep2;
    Gtk::Box m_top_progress_box;
    Gtk::Box m_progress_spatial_box;
    Gtk::Label m_progress_spatial_title;
    Gtk::ProgressBar m_big_spatial_progress;
    Gtk::Box m_progress_tilt_box;
    Gtk::Label m_progress_tilt_title;
    Gtk::ProgressBar m_big_tilt_progress;

    VideoArea m_video_area;

    // Sidebar controls
    Gtk::ScrolledWindow m_sidebar_scroll;
    Gtk::Box m_sidebar_box;

    // Source Frame
    Gtk::Frame m_source_frame;
    Gtk::Box m_source_box;
    Gtk::Label m_source_info_label;
    Gtk::Label m_fps_label;

    // Board Settings Frame (Loaded from JSON)
    Gtk::Frame m_board_frame;
    Gtk::Box m_board_box;
    Gtk::Label m_board_file_label;
    Gtk::Label m_board_info_label;

    // Capture & Guidance Frame
    Gtk::Frame m_capture_frame;
    Gtk::Box m_capture_box;
    Gtk::Label m_stationarity_badge;
    Gtk::Label m_corners_count_label;
    Gtk::CheckButton m_auto_capture_check;
    Gtk::Button m_capture_button;
    Gtk::Label m_frames_count_label;
    Gtk::ScrolledWindow m_frames_list_scroll;
    Gtk::TreeView m_frames_tree_view;
    Glib::RefPtr<Gtk::ListStore> m_frames_list_store;
    struct FrameColumns : public Gtk::TreeModel::ColumnRecord {
        FrameColumns() {
            add(col_id);
            add(col_corners);
            add(col_error);
        }
        Gtk::TreeModelColumn<int> col_id;
        Gtk::TreeModelColumn<int> col_corners;
        Gtk::TreeModelColumn<Glib::ustring> col_error;
    } m_frame_cols;
    Gtk::Box m_frame_buttons_box;
    Gtk::Button m_delete_frame_button;
    Gtk::Button m_clear_frames_button;

    // Calibration & Output Frame
    Gtk::Frame m_calib_frame;
    Gtk::Box m_calib_box;
    Gtk::ComboBoxText m_dist_model_combo;
    Gtk::Button m_calibrate_button;
    Gtk::Button m_filter_outliers_button;
    Gtk::ProgressBar m_calib_progress_bar;
    Gtk::Label m_rms_label;
    Gtk::Label m_baseline_label;
    Gtk::Label m_epipolar_err_label;

    // Display & Validation Frame
    Gtk::Frame m_display_frame;
    Gtk::Box m_display_box;
    Gtk::ComboBoxText m_display_mode_combo;
    Gtk::Button m_save_button;
    Gtk::Label m_validation_label;

    // Asynchronous Background Calibration
    std::atomic<bool> m_is_calibrating{false};
    std::thread m_calib_thread;
    Glib::Dispatcher m_calib_progress_dispatcher;
    Glib::Dispatcher m_calib_finished_dispatcher;
    std::mutex m_calib_state_mutex;
    double m_calib_progress_frac = 0.0;
    std::string m_calib_progress_stage;
    bool m_calib_done_ok = false;
    StereoCalibrationResult m_calib_done_result;
    std::string m_calib_done_error;
};

} // namespace dvrk_data::calibration

#endif // DVRK_DATA_CALIBRATION_CALIBRATION_WINDOW_HPP
