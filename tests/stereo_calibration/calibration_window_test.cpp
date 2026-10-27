#include "calibration_window.hpp"
#include "calibration_archive_fixture.hpp"
#include <gst/gst.h>
#include <iostream>

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
int main(int argc, char **argv) {
    gst_init(&argc, &argv);
    if (!gtk_init_check(&argc, &argv)) return 77;
    Gtk::Main gtk(argc, argv);
    CalibrationArchiveFixture fixture;
    using namespace dvrk_data::calibration;
    ReceiverConfig receiver;
    receiver.stereo_input = "@dvrk:test:calibration_window_unused";
    CalibrationWindow window((std::filesystem::path(fixture.archive) / "board.json").string(), receiver, fixture.board);
    window.show();
    Gtk::ComboBoxText *view = nullptr;
    Gtk::TreeView *tree = nullptr;
    Gtk::SpinButton *alpha = nullptr;
    Gtk::Button *load = nullptr;
    Gtk::CheckButton *save_images = nullptr;
    Gtk::Button *save = nullptr;
    int collapsed_details = 0;
    for (auto *widget : descendants(window)) {
        if (auto *combo = dynamic_cast<Gtk::ComboBoxText *>(widget))
            if (combo->get_active_id() == "heatmap") view = combo;
        if (auto *candidate = dynamic_cast<Gtk::TreeView *>(widget)) tree = candidate;
        if (auto *candidate = dynamic_cast<Gtk::SpinButton *>(widget)) alpha = candidate;
        if (auto *button = dynamic_cast<Gtk::Button *>(widget))
            if (button->get_label() == "Load Calibration Images...") load = button;
        if (auto *button = dynamic_cast<Gtk::CheckButton *>(widget))
            if (button->get_label() == "Include raw calibration images") save_images = button;
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
    check(alpha->get_value() == 0.2, "GUI alpha does not default to 0.2");
    check(collapsed_details == 2, "Source and board details must be collapsible");
    check(save_images && save_images->get_active() && save,
          "Saving raw images is not discoverable or enabled by default");
    save_images->set_active(false);
    check(save->get_label() == "Save Calibration...", "Save button does not reflect image export choice");
    save_images->set_active(true);
    bool chose_folder = false;
    sigc::connection choose;
    choose = Glib::signal_timeout().connect([&] {
        for (auto *top : Gtk::Window::list_toplevels()) {
            auto *dialog = dynamic_cast<Gtk::FileChooserDialog *>(top);
            if (!dialog || dialog->get_title() != "Load Calibration Image Archive") continue;
            if (!chose_folder) {
                dialog->set_filename(fixture.archive);
                chose_folder = true;
                return true;
            }
            if (dialog->get_filename() == fixture.archive) {
                dialog->response(Gtk::RESPONSE_OK);
                return false;
            }
        }
        return true;
    }, 100);
    load->clicked();
    choose.disconnect();
    for (int i = 0; i < 100 && tree->get_model()->children().size() != 2; ++i) pump(30);
    check(tree->get_model()->children().size() == 2, "Archive images did not reach the GUI");
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
    window.hide();
    std::cout << "Startup heatmap/alpha, archive reload, sample selection and numeric header sorting passed\n";
}
