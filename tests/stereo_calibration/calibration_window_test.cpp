#include "calibration_window.hpp"
#include "calibration_archive_fixture.hpp"
#include <gst/gst.h>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

void check(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
void pump(int milliseconds) {
    const auto end = g_get_monotonic_time() + milliseconds * 1000;
    while (g_get_monotonic_time() < end) {
        while (g_main_context_iteration(nullptr, false)) {}
        g_usleep(1000);
    }
}
std::vector<Gtk::Widget *> descendants(Gtk::Container &container) {
    std::vector<Gtk::Widget *> result;
    for (auto *child : container.get_children()) {
        result.push_back(child);
        if (auto *nested = dynamic_cast<Gtk::Container *>(child)) {
            auto widgets = descendants(*nested);
            result.insert(result.end(), widgets.begin(), widgets.end());
        }
    }
    return result;
}
class VideoAreaProbe : public dvrk_data::calibration::VideoArea {
public:
    using VideoArea::on_button_press_event;
    using VideoArea::on_button_release_event;
    using VideoArea::on_motion_notify_event;
    using VideoArea::on_scroll_event;
};

int main(int argc, char **argv) {
    gst_init(&argc, &argv);
    if (!gtk_init_check(&argc, &argv)) return 77;
    Gtk::Main gtk(argc, argv);
    CalibrationArchiveFixture fixture;
    using namespace dvrk_data::calibration;
    VideoAreaProbe probe;
    probe.set_image(cv::Mat(48,64,CV_8UC3,cv::Scalar(0,0,0)));
    GdkEventButton click{};
    click.button = 1; click.type = GDK_BUTTON_PRESS; click.x = 20; click.y = 20;
    check(!probe.on_button_press_event(&click), "Image clicks still trigger epipolar interaction");
    Gtk::Window navigation_window;
    navigation_window.add(probe);
    navigation_window.show_all();
    pump(50);
    probe.set_navigation_enabled(true);
    probe.set_hover_enabled(true);
    double hover_x = 0, hover_y = 0;
    probe.signal_image_hovered.connect([&](double x,double y) { hover_x=x; hover_y=y; });
    const double cx = probe.get_allocated_width()/2.0, cy = probe.get_allocated_height()/2.0;
    GdkEventMotion motion{}; motion.x=cx+20; motion.y=cy;
    probe.on_motion_notify_event(&motion);
    const double fitted_x = hover_x;
    GdkEventScroll scroll{}; scroll.direction=GDK_SCROLL_UP; scroll.x=cx; scroll.y=cy;
    check(probe.on_scroll_event(&scroll), "Heatmap zoom event rejected");
    probe.on_motion_notify_event(&motion);
    const double zoomed_x = hover_x;
    check(zoomed_x < fitted_x && zoomed_x > 32, "Zoom did not preserve pointer coordinates");
    probe.set_image(cv::Mat(48,64,CV_8UC3,cv::Scalar(50,80,120)));
    probe.on_motion_notify_event(&motion);
    check(std::abs(hover_x-zoomed_x) < 1e-6, "Live frame update reset heatmap zoom");
    click.x=cx; click.y=cy; probe.on_button_press_event(&click);
    motion.x=cx+30; motion.y=cy+15; motion.state=GDK_BUTTON1_MASK;
    probe.on_motion_notify_event(&motion);
    click.x=motion.x; click.y=motion.y; probe.on_button_release_event(&click);
    motion.x=cx+20; motion.y=cy; motion.state=0;
    probe.on_motion_notify_event(&motion);
    check(hover_x < zoomed_x && hover_y < 24, "Heatmap pan or transformed hover failed");
    click.type=GDK_BUTTON_PRESS; click.x=cx; click.y=cy;
    probe.on_button_press_event(&click); probe.on_button_release_event(&click);
    click.type=GDK_2BUTTON_PRESS;
    probe.on_button_press_event(&click); probe.on_button_release_event(&click);
    probe.on_motion_notify_event(&motion);
    check(std::abs(hover_x-fitted_x) < 1e-6, "Double-click failed to fit heatmap");
    click.type=GDK_BUTTON_PRESS;
    probe.on_button_press_event(&click); probe.on_button_release_event(&click);
    probe.on_motion_notify_event(&motion);
    check(std::abs(hover_x-fitted_x) < 1e-6, "Short click changed the coverage view");
    navigation_window.hide();
    ReceiverConfig receiver;
    receiver.stereo_input = "@dvrk:test:calibration_window_unused";
    CalibrationWindow window((std::filesystem::path(fixture.archive) / "board.json").string(), receiver, fixture.board);
    window.show();
    Gtk::ComboBoxText *view = nullptr;
    Gtk::TreeView *tree = nullptr;
    Gtk::SpinButton *alpha = nullptr;
    Gtk::SpinButton *density_target = nullptr;
    Gtk::Button *load = nullptr;
    Gtk::Button *calibrate = nullptr;
    Gtk::Button *filter = nullptr;
    Gtk::Label *filter_report = nullptr;
    Gtk::CheckButton *save_images = nullptr;
    Gtk::Button *save = nullptr;
    int collapsed_details = 0;
    for (auto *widget : descendants(window)) {
        if (auto *combo = dynamic_cast<Gtk::ComboBoxText *>(widget))
            if (combo->get_active_id() == "heatmap") view = combo;
        if (auto *candidate = dynamic_cast<Gtk::TreeView *>(widget)) tree = candidate;
        if (auto *candidate = dynamic_cast<Gtk::SpinButton *>(widget)) {
            if (candidate->get_name() == "coverage-density-target") density_target = candidate;
            else if (candidate->get_tooltip_text().find("Residual") == Glib::ustring::npos) alpha = candidate;
        }
        if (auto *label = dynamic_cast<Gtk::Label *>(widget)) {
            if (label->get_name() == "outlier-filter-report") filter_report = label;
        }
        if (auto *button = dynamic_cast<Gtk::Button *>(widget)) {
            check(button->get_label() != "Guided collection" && button->get_label() != "Next target",
                  "Removed guided collection controls are still present");
            if (button->get_label() == "Load Calibration Images...") load = button;
            if (button->get_label() == "Run Stereo Calibration") calibrate = button;
            if (button->get_label() == "Filter Outliers & Recalibrate") filter = button;
        }
        if (auto *button = dynamic_cast<Gtk::CheckButton *>(widget)) {
            if (button->get_label() == "Include raw calibration images") save_images = button;
        }
        if (auto *button = dynamic_cast<Gtk::Button *>(widget))
            if (button->get_label() == "Save Calibration & Images...") save = button;
        if (auto *expander = dynamic_cast<Gtk::Expander *>(widget)) {
            check(!expander->get_expanded(), "Source or board details are expanded by default");
            expander->set_expanded(true);
            check(expander->get_expanded(), "Details cannot be expanded");
            expander->set_expanded(false);
            ++collapsed_details;
        }
    }
    check(view && tree && alpha && load, "Missing calibrator controls or incorrect startup view");
    check(density_target && density_target->get_value() == 2,
          "Missing target density or incorrect default");
    check(alpha->get_value() == 0.2, "GUI alpha does not default to 0.2");
    check(collapsed_details == 2, "Source and board details must be collapsible");
    check(save_images && save_images->get_active() && save,
          "Saving raw images is not discoverable or enabled by default");
    save_images->set_active(false);
    check(save->get_label() == "Save Calibration...", "Save button does not reflect image export choice");
    save_images->set_active(true);
    auto load_archive = [&](const std::string &directory, size_t count) {
        bool chose_folder = false;
        auto choose = Glib::signal_timeout().connect([&] {
            for (auto *top : Gtk::Window::list_toplevels()) {
                auto *dialog = dynamic_cast<Gtk::FileChooserDialog *>(top);
                if (!dialog || dialog->get_title() != "Load Calibration Image Archive") continue;
                if (!chose_folder) {
                    dialog->set_filename(directory);
                    chose_folder = true;
                    return true;
                }
                if (dialog->get_filename() == directory) {
                    dialog->response(Gtk::RESPONSE_OK);
                    return false;
                }
            }
            return true;
        }, 100);
        load->clicked();
        choose.disconnect();
        for (int i = 0; i < 300 && tree->get_model()->children().size() != count; ++i) pump(30);
        check(tree->get_model()->children().size() == count, "Archive images did not reach the GUI");
    };
    auto label_contains = [&](const char *text) {
        for (auto *widget : descendants(window))
            if (auto *label = dynamic_cast<Gtk::Label *>(widget))
                if (label->get_text().find(text) != Glib::ustring::npos) return true;
        return false;
    };
    view->set_active_id("distortion");
    check(view->get_active_id() == "distortion" && label_contains("Run stereo calibration"),
          "Lens view must explain when no calibration is available");
    view->set_active_id("heatmap");
    load_archive(fixture.archive, 2);
    check(view->get_active_id() == "validation" && tree->get_selection()->get_selected(),
          "Reload did not display an imported sample");
    bool raw_sample_shown = false;
    for (auto *widget : descendants(window))
        if (auto *label = dynamic_cast<Gtk::Label *>(widget))
            raw_sample_shown |= label->get_text().find("Raw images. Run stereo calibration") != Glib::ustring::npos;
    check(raw_sample_shown, "Uncalibrated sample view is blank instead of showing raw images");
    view->set_active_id("heatmap");
    tree->get_selection()->unselect_all();
    view->set_active_id("validation");
    check(tree->get_selection()->get_selected(), "Reprojection view did not select a sample automatically");

    check(view->get_model()->children().size() == 6, "Unexpected display modes in the selector");
    view->set_active_id("validation");
    bool projection_modes = false, scale_control = false;
    for (auto *widget : descendants(window)) {
        if (auto *combo = dynamic_cast<Gtk::ComboBoxText *>(widget)) {
            if (combo->get_tooltip_text().find("source of the board pose") != Glib::ustring::npos) {
                combo->set_active_id("independent");
                check(combo->get_active_id() == "independent", "Independent fit mode missing");
                combo->set_active_id("right");
                check(combo->get_active_id() == "right", "Reverse projection mode missing");
                projection_modes = true;
            }
        }
        if (auto *spin = dynamic_cast<Gtk::SpinButton *>(widget)) {
            if (spin->get_tooltip_text().find("Residual") != Glib::ustring::npos) {
                check(spin->get_value() == 3.0, "Residual scale default/state incorrect");
                spin->set_value(2.0);
                scale_control = true;
            }
        }
    }
    check(projection_modes && scale_control, "Missing validation diagnostic controls");
    auto model = Glib::RefPtr<Gtk::ListStore>::cast_dynamic(tree->get_model());
    auto first_id = [&] { int id = 0; model->children().begin()->get_value(0, id); return id; };
    tree->get_column(0)->clicked();
    check(first_id() == 10, "ID header did not toggle descending numeric sort");
    tree->get_column(0)->clicked();
    check(first_id() == 2, "ID header did not toggle ascending numeric sort");
    for (auto row : model->children()) {
        int id = 0; row.get_value(0, id);
        row.set_value(1, id == 2 ? 20 : 6);
        row.set_value(2, Glib::ustring(id == 2 ? "10.00" : "2.00"));
        row.set_value(3, id == 2 ? 10.0 : 2.0);
    }
    tree->get_column(1)->clicked();
    check(first_id() == 10, "Corners header did not sort numerically");
    tree->get_column(2)->clicked();
    check(first_id() == 10, "Error header sorted text instead of numeric error");
    tree->get_column(2)->clicked();
    check(first_id() == 2, "Error header did not reverse sort order");
    // Exercise the complete calibrated viewer with diverse rendered board poses.
    check(calibrate, "Missing calibration action");
    const auto diverse_root = fixture.root / "diverse";
    std::filesystem::create_directory(diverse_root);
    for (const char *name : {"left.yaml", "right.yaml", "stereo_calibration.yaml"})
        std::ofstream(diverse_root / name) << "unused by raw image reload\n";
    CharucoDetector detector(fixture.board);
    cv::Mat board_image;
    detector.draw_board({800,600},board_image,0);
    const cv::Mat K = (cv::Mat_<double>(3,3) << 750,0,320,0,750,240,0,0,1);
    const float width = fixture.board.squares_x*fixture.board.square_length_mm;
    const float height = fixture.board.squares_y*fixture.board.square_length_mm;
    const std::vector<cv::Point3f> outline{{0,0,0},{width,0,0},{width,height,0},{0,height,0}};
    const std::vector<cv::Point2f> source{{0,0},{799,0},{799,599},{0,599}};
    std::vector<CalibrationFrame> frames;
    for (int i = 0; i < 8; ++i) {
        CalibrationFrame f; f.id = 20+i;
        const cv::Vec3d rotation(0.12*((i%3)-1),0.18*((i%4)-1.5),0.04*i);
        for (int eye = 0; eye < 2; ++eye) {
            std::vector<cv::Point2f> projected;
            cv::projectPoints(outline, rotation, cv::Vec3d(-20+(i%3)*4-eye*6,-15+(i%2)*5,115+i*4),
                              K, cv::Mat(), projected);
            cv::Mat gray, color;
            cv::warpPerspective(board_image,gray,cv::getPerspectiveTransform(source,projected),
                                {640,480},cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255));
            cv::cvtColor(gray,color,cv::COLOR_GRAY2BGR);
            if (eye) f.right_image = color; else f.left_image = color;
        }
        frames.push_back(f);
    }
    std::string diverse_archive, error;
    check(CalibrationIO::save_calibration_archive(diverse_root.string(),frames,fixture.board,diverse_archive,error),
          "Cannot prepare diverse calibration fixture");
    load_archive(diverse_archive,8);
    const auto view_before_calibration=view->get_active_id();
    int selected_before=0;
    tree->get_selection()->get_selected()->get_value(0,selected_before);
    calibrate->clicked();
    for (int i=0;i<500 && !calibrate->get_sensitive();++i) pump(30);
    check(calibrate->get_sensitive() && view->get_active_id()==view_before_calibration,
          "Calibration changed the selected reprojection view");
    int selected_after=0;
    tree->get_selection()->get_selected()->get_value(0,selected_after);
    check(selected_before==selected_after,"Calibration changed the selected sample");
    check(filter && filter->get_sensitive() && filter_report,"Missing coverage-preserving filter controls");
    bool filter_dialog_seen=false;
    auto close_filter_dialog=Glib::signal_timeout().connect([&] {
        for (auto *top:Gtk::Window::list_toplevels()) {
            if (auto *dialog=dynamic_cast<Gtk::MessageDialog *>(top)) {
                filter_dialog_seen=true; dialog->response(Gtk::RESPONSE_OK); return false;
            }
        }
        return true;
    },30);
    filter->clicked();
    close_filter_dialog.disconnect();
    check(filter_dialog_seen && filter_report->get_text().find("Removed 0 samples and 0 paired corners")!=Glib::ustring::npos,
          "Rendered collection with broad fit disagreement should produce a no-removal audit report");
    check(tree->get_model()->children().size()==8 && view->get_active_id()==view_before_calibration,
          "Conservative filtering changed the protected collection or current view");
    view->set_active_id("heatmap");
    calibrate->clicked();
    for (int i=0;i<500 && !calibrate->get_sensitive();++i) pump(30);
    check(calibrate->get_sensitive() && view->get_active_id()=="heatmap",
          "Recalibration switched away from live coverage");
    view->set_active_id("distortion");
    check(label_contains("Lens distortion only") && label_contains("Same color scale in both eyes"),
          "Calibrated lens view is missing its chart or legend");
    tree->get_selection()->unselect_all();
    tree->get_selection()->select(tree->get_model()->children().begin());
    check(view->get_active_id() == "distortion", "Sample selection replaced the lens-only chart");
    calibrate->clicked();
    for (int i=0;i<500 && !calibrate->get_sensitive();++i) pump(30);
    check(calibrate->get_sensitive() && view->get_active_id() == "distortion" && label_contains("Lens distortion only"),
          "Recalibration did not refresh the selected lens view");
    pump(300);
    check(label_contains("Lens distortion only"), "Stream timeout replaced the static lens chart");
    view->set_active_id("validation");
    Gtk::ComboBoxText *projection = nullptr, *corner = nullptr;
    for (auto *widget : descendants(window)) {
        if (auto *combo = dynamic_cast<Gtk::ComboBoxText *>(widget)) {
            if (combo->get_tooltip_text().find("source of the board pose") != Glib::ustring::npos) projection = combo;
            if (combo->get_tooltip_text().find("Select a ChArUco") != Glib::ustring::npos) corner = combo;
        }
    }
    check(projection && corner && corner->get_sensitive(), "Calibrated controls are disabled");
    for (const char *mode : {"independent","left","right"}) {
        projection->set_active_id(mode);
        corner->set_active(1);
        bool metrics = false, details = false;
        for (auto *widget : descendants(window)) if (auto *label = dynamic_cast<Gtk::Label *>(widget)) {
            metrics |= label->get_text().find("R -> L prediction RMS:") != Glib::ustring::npos;
            details |= label->get_text().find("magnitude") != Glib::ustring::npos;
        }
        check(metrics && details, "Calibrated viewer missing shared metrics or selected corner details");
    }
    view->set_active_id("heatmap");
    VideoArea *video = nullptr;
    for (auto *widget : descendants(window))
        if (auto *area = dynamic_cast<VideoArea *>(widget)) video = area;
    check(video, "Missing live coverage overlay");
    video->signal_image_hovered.emit(5,5);
    check(video->get_tooltip_text().find("retained images") != Glib::ustring::npos,
          "Live coverage hover does not show observation counts");
    tree->get_selection()->select(tree->get_model()->children().begin());
    check(view->get_active_id() == "heatmap", "Sample selection unexpectedly left live coverage");
    Gtk::Button *remove = nullptr;
    for (auto *widget : descendants(window)) if (auto *button = dynamic_cast<Gtk::Button *>(widget))
        if (button->get_label() == "Delete Selected") remove = button;
    check(remove, "Missing sample deletion control");
    remove->clicked();
    check(tree->get_model()->children().size() == 7 && view->get_active_id() == "heatmap",
          "Deleting a sample did not rebuild the active coverage chart");
    Gtk::Button *clear_all = nullptr;
    for (auto *widget : descendants(window)) if (auto *button = dynamic_cast<Gtk::Button *>(widget))
        if (button->get_label() == "Clear All") clear_all = button;
    check(clear_all, "Missing clear-samples control");
    view->set_active_id("distortion");
    clear_all->clicked();
    check(tree->get_model()->children().empty(), "Clear All left retained samples");
    check(view->get_active_id() == "heatmap", "Clearing calibration left a stale lens chart");
    CoverageTracker empty_coverage;
    const auto empty_overlay = CoverageTracker::draw_nested_overlay(
        cv::Mat(480,640,CV_8UC3,cv::Scalar()),empty_coverage.nested_coverage());
    for (const auto &band : empty_overlay.tiles) {
        video->signal_image_hovered.emit(band.bounds.x+1,band.bounds.y+1);
        check(video->get_tooltip_text().find("0 retained images, 0 detected corners") != Glib::ustring::npos,
              "Clear All left stale coverage in the cached heatmap");
    }
    window.hide();
    // A live source must keep updating beneath the default coverage overlay.
    {
    ReceiverConfig live_receiver;
    live_receiver.use_testsrc = true;
    live_receiver.width = 1280; live_receiver.height = 480;
    CalibrationWindow live_window((std::filesystem::path(fixture.archive)/"board.json").string(),live_receiver,fixture.board);
    live_window.show();
    pump(400);
    VideoArea *live_video = nullptr;
    for (auto *widget : descendants(live_window))
        if (auto *area = dynamic_cast<VideoArea *>(widget)) live_video = area;
    check(live_video, "Live coverage video missing");
    auto snapshot = [&] {
        auto surface = Cairo::ImageSurface::create(Cairo::FORMAT_RGB24,
            live_video->get_allocated_width(),live_video->get_allocated_height());
        auto context = Cairo::Context::create(surface);
        gtk_widget_draw(GTK_WIDGET(live_video->gobj()),context->cobj());
        surface->flush();
        return cv::Mat(surface->get_height(),surface->get_width(),CV_8UC4,
                       surface->get_data(),surface->get_stride()).clone();
    };
    const auto first_live = snapshot();
    pump(200);
    const auto second_live = snapshot();
    check(cv::norm(first_live,second_live,cv::NORM_L1) > 1000,
          "Coverage overlay froze the live stream instead of updating camera images");
    std::vector<cv::Mat> channels;
    cv::split(second_live,channels);
    cv::Mat chroma;
    cv::absdiff(channels[0],channels[1],chroma);
    check(cv::mean(chroma)[0] > 5,
          "Coverage view shows a chart or placeholder instead of live camera colors");
    Gtk::ComboBoxText *live_view = nullptr;
    for (auto *widget : descendants(live_window)) if (auto *combo = dynamic_cast<Gtk::ComboBoxText *>(widget))
        if (combo->get_active_id() == "heatmap") live_view = combo;
    check(live_view, "Missing live view selector");
    live_view->set_active_id("distortion");
    pump(60);
    const auto static_lens = snapshot();
    pump(150);
    check(cv::norm(static_lens,snapshot(),cv::NORM_L1) == 0,
          "Live camera frames overwrote the static lens view");
    live_view->set_active_id("validation");
    pump(60);
    live_view->set_active_id("heatmap");
    pump(150);
    const auto resumed_live = snapshot();
    check(cv::norm(resumed_live,second_live,cv::NORM_L1) > 1000,
          "Frame notification coalescing prevented live refresh after switching views");
    if (std::getenv("DVRK_CALIBRATION_PREVIEWS")) cv::imwrite("/tmp/charuco-build/live-coverage.png",second_live);
    live_window.hide();
    }
    std::cout << "Startup, reload, sorting, calibration, bidirectional overlays, corner details, nested coverage and image deletion passed\n";
}
