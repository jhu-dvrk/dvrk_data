#include "video_view.hpp"

VideoView::VideoView() {
    set_size_request(800, 450);
    set_hexpand(true);
    set_vexpand(true);
    set_tooltip_text("Wheel: zoom. Left drag: pan. Double-click: fit.");
    add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK |
               Gdk::POINTER_MOTION_MASK | Gdk::SCROLL_MASK | Gdk::SMOOTH_SCROLL_MASK);
}

VideoView::~VideoView() {
    if (sample_) gst_sample_unref(sample_);
}

void VideoView::set_sample(GstSample* sample) {
    GstVideoInfo info;
    gst_video_info_init(&info);
    const GstVideoFormat expected = G_BYTE_ORDER == G_LITTLE_ENDIAN
        ? GST_VIDEO_FORMAT_BGRx : GST_VIDEO_FORMAT_xRGB;
    if (!sample || !gst_video_info_from_caps(&info, gst_sample_get_caps(sample)) ||
        GST_VIDEO_INFO_FORMAT(&info) != expected) return;
    if (info.width != info_.width || info.height != info_.height) {
        transform_.reset();
        dragging_ = false;
    }
    GstSample* next = gst_sample_ref(sample);
    if (sample_) gst_sample_unref(sample_);
    sample_ = next;
    info_ = info;
    queue_draw();
}

bool VideoView::on_draw(const Cairo::RefPtr<Cairo::Context>& cr) {
    cr->set_source_rgb(0.06, 0.06, 0.06);
    cr->paint();
    if (!sample_) return true;
    GstVideoFrame frame;
    if (!gst_video_frame_map(&frame, &info_, gst_sample_get_buffer(sample_), GST_MAP_READ)) return true;
    {
        const auto view = transform_.placement(get_allocated_width(), get_allocated_height(),
                                                info_.width, info_.height);
        if (view.scale > 0) {
            auto surface = Cairo::ImageSurface::create(
                static_cast<unsigned char*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
                Cairo::FORMAT_RGB24, info_.width, info_.height,
                GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0));
            surface->mark_dirty();
            cr->save();
            cr->translate(view.x, view.y);
            cr->scale(view.scale, view.scale);
            cr->set_source(surface, 0, 0);
            cr->rectangle(0, 0, info_.width, info_.height);
            cr->fill();
            cr->restore();
        }
    }
    gst_video_frame_unmap(&frame);
    return true;
}

bool VideoView::on_button_press_event(GdkEventButton* event) {
    if (!sample_ || event->button != 1) return false;
    if (event->type == GDK_2BUTTON_PRESS) {
        transform_.reset();
        dragging_ = false;
        queue_draw();
    } else {
        dragging_ = true;
        drag_x_ = event->x;
        drag_y_ = event->y;
    }
    return true;
}

bool VideoView::on_button_release_event(GdkEventButton* event) {
    if (event->button != 1) return false;
    dragging_ = false;
    return true;
}

bool VideoView::on_motion_notify_event(GdkEventMotion* event) {
    if (!dragging_) return false;
    if (!(event->state & GDK_BUTTON1_MASK)) {
        dragging_ = false;
        return false;
    }
    transform_.pan_x += event->x - drag_x_;
    transform_.pan_y += event->y - drag_y_;
    drag_x_ = event->x;
    drag_y_ = event->y;
    queue_draw();
    return true;
}

bool VideoView::on_scroll_event(GdkEventScroll* event) {
    if (!sample_) return false;
    double delta = 0;
    if (event->direction == GDK_SCROLL_UP) delta = -1;
    else if (event->direction == GDK_SCROLL_DOWN) delta = 1;
    else if (event->direction == GDK_SCROLL_SMOOTH) delta = event->delta_y;
    transform_.scroll(delta, event->x, event->y, get_allocated_width(), get_allocated_height(),
                      info_.width, info_.height);
    queue_draw();
    return true;
}
