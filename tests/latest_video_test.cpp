#include <dvrk_data/latest_video.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <iostream>
#include <stdexcept>
#include <cstring>

bool planar = false;
void check(bool good, const char *message) { if (!good) throw std::runtime_error(message); }
void push(GstElement *source, unsigned value, gint64 stamp, GstClockTime pts) {
  auto *buffer = gst_buffer_new_allocate(nullptr, planar ? 8 * 8 * 3 / 2 : 8 * 8 * 4, nullptr);
  GstMapInfo map;
  check(gst_buffer_map(buffer, &map, GST_MAP_WRITE), "Cannot map input");
  if (planar) {
    std::memset(map.data, value, 8 * 8);
    std::memset(map.data + 8 * 8, 128, 8 * 8 / 2);
  } else for (size_t i = 0; i < map.size; i += 4) {
    map.data[i] = value; map.data[i + 1] = 0; map.data[i + 2] = 0; map.data[i + 3] = 255;
  }
  gst_buffer_unmap(buffer, &map);
  GST_BUFFER_PTS(buffer) = pts;
  GST_BUFFER_DURATION(buffer) = GST_SECOND / 30;
  DcFrameTimestamps times;
  times.left_source_ts = stamp;
  times.right_source_ts = stamp + 1;
  times.source_stereo_output_ts = stamp;
  times.left_reference_timestamps = {{"timestamp/test-left", static_cast<guint64>(stamp), 1}};
  times.right_reference_timestamps = {{"timestamp/test-right", static_cast<guint64>(stamp + 1), 1}};
  dc_buffer_set_frame_timestamps(buffer, times);
  check(gst_app_src_push_buffer(GST_APP_SRC(source), buffer) == GST_FLOW_OK, "Input rejected");
}
DcFrameTimestamps expect(GstElement *sink, int left, int right) {
  for (int attempt = 0; attempt < 20; ++attempt) {
    auto *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 250 * GST_MSECOND);
    check(sample, "Latest mixer waited for the other eye or its timestamp");
    auto *buffer = gst_sample_get_buffer(sample);
    GstVideoInfo info;
    gst_video_info_from_caps(&info, gst_sample_get_caps(sample));
    GstVideoFrame frame;
    check(gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ), "Cannot map output");
    auto *data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    const bool matches = data[0] == left && data[planar ? 8 : 8 * 4] == right;
    gst_video_frame_unmap(&frame);
    auto times = dc_buffer_get_frame_timestamps(buffer);
    gst_sample_unref(sample);
    if (matches) return times;
  }
  throw std::runtime_error("Output pixels do not contain latest eyes");
}
int main(int argc, char **argv) {
  planar = argc > 1 && std::string(argv[1]) == "--i420";
  gst_init(&argc, &argv);
  dc_frame_timestamps_meta_register();
  const std::string format = planar ? "I420" : "RGBA";
  const auto description = std::string("appsrc name=left is-live=true format=time ! ") +
      "video/x-raw,format=" + format + ",width=8,height=8,framerate=30/1 ! " + dvrk_video::LatestVideoMixers::sink("left_cache") +
      " appsrc name=right is-live=true format=time ! video/x-raw,format=" + format + ",width=8,height=8,framerate=1/1 ! " +
      dvrk_video::LatestVideoMixers::sink("right_cache") + " " + dvrk_video::LatestVideoMixers::source("mixed") +
      " ! appsink name=result sync=false max-buffers=1 drop=true wait-on-eos=false";
  GError *error = nullptr;
  auto *pipeline = gst_parse_launch(description.c_str(), &error);
  check(pipeline && !error, "Cannot parse pipeline");
  auto *left = gst_bin_get_by_name(GST_BIN(pipeline), "left");
  auto *right = gst_bin_get_by_name(GST_BIN(pipeline), "right");
  auto *sink = gst_bin_get_by_name(GST_BIN(pipeline), "result");
  dvrk_video::LatestVideoSpec spec{"mixed", 16, 8, !planar, dvrk_video::OutputStage::Alignment, {
      {"left_cache", 0, 0, 8, 8, dvrk_video::CameraContent::Left},
      {"right_cache", 8, 0, 8, 8, dvrk_video::CameraContent::Right}}};
  dvrk_video::LatestVideoMixers latest;
  std::string message;
  check(latest.start(pipeline, {spec}, message), message.c_str());
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  push(left, 11, 1011, 3600 * GST_SECOND);
  auto times = expect(sink, 11, planar ? 16 : 0);
  check(times.left_source_ts == 1011 && times.right_source_ts == 0, "Missing eye got invented metadata");
  push(left, 22, 1022, 0);
  times = expect(sink, 22, planar ? 16 : 0);
  check(times.left_source_ts == 1022 && times.left_reference_timestamps[0].timestamp == 1022, "Pixels/provenance disagree");
  push(right, 33, 2033, 7200 * GST_SECOND);
  times = expect(sink, 22, 33);
  check(times.right_source_ts == 2034 && times.right_reference_timestamps[0].timestamp == 2034, "Right provenance lost");
  check(times.source_stereo_output_ts == 0 && times.alignment_output_ts > 0,
        "Different source pairs invented a common stage timestamp");
  gst_app_src_end_of_stream(GST_APP_SRC(right));
  push(left, 44, 1044, 0);
  times = expect(sink, 44, 33);
  check(times.left_source_ts == 1044 && times.right_source_ts == 2034, "EOS eye blocked fresh eye");
  // A downstream stall must retain only the newest completed composition.
  for (int i = 0; i < 50; ++i) push(left, 50 + i, 1050 + i, 0);
  times = expect(sink, 99, 33);
  check(times.left_source_ts == 1099, "Backlog defeated latest-frame selection");
  gst_app_src_end_of_stream(GST_APP_SRC(left));
  gst_element_set_state(pipeline, GST_STATE_NULL);
  latest.stop();
  gst_object_unref(left); gst_object_unref(right); gst_object_unref(sink); gst_object_unref(pipeline);
  std::cout << "Independent latest eyes, missing/stalled/EOS peer, arbitrary PTS, backlog and provenance passed\n";
}
