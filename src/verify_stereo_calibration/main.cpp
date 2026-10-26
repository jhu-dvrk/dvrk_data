// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/calibration_io.hpp"
#include "calibration_window.hpp"
#include "dvrk_data/calibration/gst_stereo_receiver.hpp"
#include "dvrk_data/calibration/stereo_calibrator.hpp"
#include <dvrk_data/dvrk_gst_socket.hpp>

#include <gtkmm.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

namespace dvrk_data::calibration {

class VerificationWindow : public Gtk::Window {
public:
    VerificationWindow(const std::string &calib_file,
                       const ReceiverConfig &receiver_cfg)
        : m_main_box(Gtk::ORIENTATION_VERTICAL, 4),
          m_top_bar(Gtk::ORIENTATION_HORIZONTAL, 8),
          m_anaglyph_check("3D Anaglyph Mode"),
          m_receiver_config(receiver_cfg) {

        set_title("dVRK Stereo Calibration Verification");
        set_default_size(1280, 720);
        set_border_width(6);

        if (!CalibrationIO::load_stereo_calibration_file(calib_file, m_calib)) {
            std::cerr << "Failed to load calibration file: " << calib_file << std::endl;
        }

        add(m_main_box);

        // Top control bar
        m_top_bar.pack_start(*Gtk::manage(new Gtk::Label("Calibration: " + calib_file)), Gtk::PACK_SHRINK);

        std::ostringstream ss;
        ss << " | Baseline: " << std::fixed << std::setprecision(2) << m_calib.baseline_mm
           << " mm | RMS: " << std::setprecision(3) << m_calib.rms_error
           << " px | Epipolar Err: " << m_calib.avg_epipolar_error_px << " px";
        m_summary = m_calib.success ? ss.str() : "Calibration unavailable — showing raw stereo";
        m_info_label.set_text(m_summary);
        m_top_bar.pack_start(m_info_label, Gtk::PACK_EXPAND_WIDGET);

        m_anaglyph_check.signal_toggled().connect([this] {
            m_anaglyph_mode = m_anaglyph_check.get_active();
        });
        m_top_bar.pack_start(m_anaglyph_check, Gtk::PACK_SHRINK);

        m_main_box.pack_start(m_top_bar, Gtk::PACK_SHRINK);
        m_main_box.pack_start(m_video_area, Gtk::PACK_EXPAND_WIDGET);

        m_dispatcher.connect(sigc::mem_fun(*this, &VerificationWindow::on_frame_dispatched));

        m_receiver = std::make_unique<GstStereoReceiver>();
        if (m_receiver->start(m_receiver_config)) {
            m_receiver->set_callback([this](const cv::Mat &l, const cv::Mat &r, uint64_t ts) {
                (void)ts;
                std::lock_guard<std::mutex> lock(m_mutex);
                m_raw_l = l;
                m_raw_r = r;
                m_has_frame = true;
                m_dispatcher.emit();
            });
        }

        show_all_children();
    }

    ~VerificationWindow() override {
        if (m_receiver) m_receiver->stop();
    }

private:
    void on_frame_dispatched() {
        cv::Mat l, r;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (!m_has_frame) return;
            l = m_raw_l;
            r = m_raw_r;
            m_has_frame = false;
        }

        cv::Mat rect_l, rect_r;
        if (m_calib.image_size.width > 0 && l.size() != m_calib.image_size) {
            std::ostringstream ss;
            ss << "RESOLUTION MISMATCH: Stream " << l.cols << "x" << l.rows
               << " != Calib " << m_calib.image_size.width << "x" << m_calib.image_size.height
               << " (Rectification skipped)";
            m_info_label.set_text(ss.str());
            rect_l = l;
            rect_r = r;
        } else {
            m_info_label.set_text(m_summary);
            StereoCalibrator::rectify_stereo(l, r, rect_l, rect_r, m_calib);
        }

        cv::Mat display_img;
        if (m_anaglyph_mode) {
            display_img = StereoCalibrator::make_anaglyph(rect_l, rect_r);
        } else {
            display_img = StereoCalibrator::make_side_by_side_epipolar(rect_l, rect_r, 32);
        }

        m_video_area.set_image(display_img);
    }

    Gtk::Box m_main_box;
    Gtk::Box m_top_bar;
    Gtk::Label m_info_label;
    std::string m_summary;
    Gtk::CheckButton m_anaglyph_check;
    VideoArea m_video_area;

    Glib::Dispatcher m_dispatcher;
    std::mutex m_mutex;
    cv::Mat m_raw_l, m_raw_r;
    bool m_has_frame = false;
    bool m_anaglyph_mode = false;

    ReceiverConfig m_receiver_config;
    StereoCalibrationResult m_calib;
    std::unique_ptr<GstStereoReceiver> m_receiver;
};

} // namespace dvrk_data::calibration

int main(int argc, char *argv[]) {
    std::string calib_file = "stereo_calibration.yaml";
    dvrk_data::calibration::ReceiverConfig receiver_cfg;
    bool config_seen = false, input_seen = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: verify_stereo_calibration [options]\n"
                      << "  -c, --calibration <file>  Calibration (default: stereo_calibration.yaml)\n"
                      << "  -i, --input <stream>      Side-by-side input (default: @dvrk:stereo_source:stereo)\n"
                      << "  -t, --testsrc             Synthetic input\n";
            return 0;
        } else if ((arg == "-c" || arg == "--calibration") && i + 1 < argc && !config_seen) {
            calib_file = argv[++i]; config_seen = true;
        } else if ((arg == "-i" || arg == "--input") && i + 1 < argc && !input_seen) {
            receiver_cfg.stereo_input = argv[++i]; input_seen = true;
        } else if (arg == "-t" || arg == "--testsrc") {
            receiver_cfg.use_testsrc = true;
        } else {
            std::cerr << "Unknown argument or missing option value: " << arg << "\nUse --help for named options.\n";
            return 1;
        }
    }
    auto app = Gtk::Application::create("org.dvrk.verify_stereo_calibration");
    dvrk_data::calibration::VerificationWindow window(calib_file, receiver_cfg);
    return app->run(window);
}
