#ifndef DVRK_DATA_VIDEO_TAG_VIDEO_VIEW_HPP
#define DVRK_DATA_VIDEO_TAG_VIDEO_VIEW_HPP

#include <gtkmm.h>
#include <gst/video/video.h>
#include "view_transform.hpp"

// UI-thread-only view. Samples keep their native resolution as Cairo transforms
// zoom and pan the display; paused frames can be navigated without a new seek.
class VideoView : public Gtk::DrawingArea {
public:
    VideoView();
    ~VideoView() override;
    void set_sample(GstSample* sample);

protected:
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
    bool on_button_press_event(GdkEventButton* event) override;
    bool on_button_release_event(GdkEventButton* event) override;
    bool on_motion_notify_event(GdkEventMotion* event) override;
    bool on_scroll_event(GdkEventScroll* event) override;

private:
    GstSample* sample_ = nullptr;
    GstVideoInfo info_{};
    VideoViewTransform transform_;
    bool dragging_ = false;
    double drag_x_ = 0.0, drag_y_ = 0.0;
};

#endif
