#include <dvrk_data/stereo_timestamp_meta.hpp>
#include <dvrk_data/dvrk_gst_socket.hpp>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>

void check(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
}
GstElement *parse(const std::string &description) {
    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(description.c_str(), &error);
    if (error) throw std::runtime_error(error->message);
    check(pipeline, "Missing pipeline");
    return pipeline;
}
void stop(GstElement *pipeline) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}
void roundtrip(GstBuffer *buffer) {
    const auto expected = dc_buffer_get_frame_timestamps(buffer);
    auto *meta = gst_buffer_get_custom_meta(buffer, DC_FRAME_TIMESTAMPS_META_NAME);
    auto *bytes = g_byte_array_new();
    check(gst_meta_serialize_simple(&meta->meta, bytes), "Custom reference timestamps cannot be serialized");
    GstBuffer *copy = gst_buffer_new();
    guint32 consumed = 0;
    check(gst_meta_deserialize(copy, bytes->data, bytes->len, &consumed), "Deserialization failed");
    check(consumed == bytes->len, "Incomplete metadata deserialization");
    const auto result = dc_buffer_get_frame_timestamps(copy);
    check(result.left_reference_timestamps.size() == expected.left_reference_timestamps.size(), "Lost left reference metadata");
    for (size_t i = 0; i < expected.left_reference_timestamps.size(); ++i) {
        const auto &a = expected.left_reference_timestamps[i], &b = result.left_reference_timestamps[i];
        check(a.reference == b.reference && a.timestamp == b.timestamp && a.duration == b.duration,
              "Reference timestamps changed during serialization");
    }
    check(result.channel == expected.channel, "Channel changed during serialization");
    check(result.source_stereo_output_ts == expected.source_stereo_output_ts &&
          result.alignment_output_ts == expected.alignment_output_ts &&
          result.overlay_output_ts == expected.overlay_output_ts, "Stage times changed during serialization");
    gst_buffer_unref(copy); g_byte_array_unref(bytes);
}

int main(int argc, char **argv) {
    const bool camera_only = argc > 1 && std::string(argv[1]) == "--camera";
    const bool gl = camera_only || (argc > 1 && std::string(argv[1]) == "--gl");
    const bool source_stage = argc > 1 && std::string(argv[1]) == "--source";
    gst_init(&argc, &argv);
    dc_frame_timestamps_meta_register();
    // Old metadata must remain readable without inventing hardware timestamps.
    GstBuffer *legacy = gst_buffer_new();
    gst_structure_set(dc_buffer_get_frame_timestamps_structure(legacy), "left-source-ts", G_TYPE_INT64, gint64(42), nullptr);
    check(dc_buffer_get_frame_timestamps(legacy).left_reference_timestamps.empty(), "Missing hardware time must remain unavailable");
    gst_buffer_unref(legacy);

    const std::string upload = gl ? " ! glupload" : "";
    const std::string compositor = gl ? "glvideomixer" : "compositor";
    const std::string output = gl
        ? " ! video/x-raw(memory:GLMemory),width=16,height=8,framerate=10/1 ! gldownload ! video/x-raw,format=RGBA"
        : " ! video/x-raw,format=RGBA,width=16,height=8,framerate=10/1";
    auto *pipeline = parse(
        "appsrc name=left format=time ! video/x-raw,format=RGBA,width=8,height=8,framerate=10/1 ! queue" + upload + " ! mix.sink_0 "
        "appsrc name=right format=time ! video/x-raw,format=RGBA,width=8,height=8,framerate=5/1 ! queue" + upload + " ! mix.sink_1 " +
        compositor + " name=mix sink_1::xpos=8" + output + " ! appsink name=result sync=false");
    auto *mix = gst_bin_get_by_name(GST_BIN(pipeline), "mix");
    auto *sink = gst_bin_get_by_name(GST_BIN(pipeline), "result");
    DcStereoTimestampState state;
    if (source_stage) state.stage = DcStereoTimestampStage::Source;
    check(gl ? dc_attach_mixer_timestamps(mix, camera_only) : dc_attach_stereo_timestamps(mix, &state),
          "Cannot attach compositor provenance");
    gst_object_unref(mix);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    for (int side = 0; side < 2; ++side) {
        auto *source = gst_bin_get_by_name(GST_BIN(pipeline), side ? "right" : "left");
        for (int i = 0; i < 6; i += side ? 2 : 1) {
            auto *buffer = gst_buffer_new_allocate(nullptr, 8 * 8 * 4, nullptr);
            GstMapInfo map;
            check(gst_buffer_map(buffer, &map, GST_MAP_WRITE), "Map failed");
            for (size_t p = 0; p < map.size; p += 4) {
                map.data[p] = i; map.data[p+1] = 0; map.data[p+2] = 0; map.data[p+3] = 255;
            }
            gst_buffer_unmap(buffer, &map);
            GST_BUFFER_PTS(buffer) = i * (GST_SECOND / 10);
            GST_BUFFER_DURATION(buffer) = GST_SECOND / (side ? 5 : 10);
            auto *hardware = gst_caps_from_string(side ? "timestamp/x-decklink-hardware, device-id=(int)1"
                                                      : "timestamp/x-decklink-hardware, device-id=(int)0");
            auto *stream = gst_caps_from_string("timestamp/x-decklink-stream");
            gst_buffer_add_reference_timestamp_meta(buffer, hardware, i * 1000 + side * 100, GST_CLOCK_TIME_NONE);
            gst_buffer_add_reference_timestamp_meta(buffer, stream, i * 2000, GST_BUFFER_DURATION(buffer));
            gst_caps_unref(hardware); gst_caps_unref(stream);
            DcFrameTimestamps timestamps;
            timestamps.source_stereo_output_ts = 777;
            timestamps.alignment_output_ts = 888;
            timestamps.stereo_output_ts = 888;
            if (side == 0) {
                timestamps.left_source_ts = 1000 + i;
                timestamps.left_reference_timestamps = dc_buffer_get_reference_timestamps(buffer);
                if (camera_only) {
                    timestamps.right_source_ts = 2000 + i;
                    timestamps.right_reference_timestamps = timestamps.left_reference_timestamps;
                    timestamps.right_reference_timestamps[0].timestamp += 100;
                }
            } else {
                timestamps.right_source_ts = 2000 + i;
                timestamps.right_reference_timestamps = dc_buffer_get_reference_timestamps(buffer);
            }
            check(dc_buffer_set_frame_timestamps(buffer, timestamps), "Cannot attach custom metadata");
            check(gst_app_src_push_buffer(GST_APP_SRC(source), buffer) == GST_FLOW_OK, "Cannot push input");
        }
        gst_app_src_end_of_stream(GST_APP_SRC(source)); gst_object_unref(source);
    }
    int frames = 0;
    DcFrameTimestamps last;
    while (auto *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 5 * GST_SECOND)) {
        auto *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        check(gst_buffer_map(buffer, &map, GST_MAP_READ), "Cannot map stereo frame");
        const unsigned left = map.data[0], right = camera_only ? left : map.data[8 * 4];
        gst_buffer_unmap(buffer, &map);
        last = dc_buffer_get_frame_timestamps(buffer);
        check(last.left_reference_timestamps.size() == 2 && last.right_reference_timestamps.size() == 2, "Composition lost reference metadata");
        check(last.left_reference_timestamps[0].timestamp == left * 1000 &&
              last.right_reference_timestamps[0].timestamp == right * 1000 + 100, "Hardware timestamps do not match composed pixels");
        check(last.left_source_ts == 1000 + left && last.right_source_ts == 2000 + right, "CPU timestamps do not match composed pixels");
        if (!gl) check(last.channel == "stereo-LR", "Stereo channel metadata was lost");
        check((source_stage ? last.source_stereo_output_ts > 777 : last.source_stereo_output_ts == 777) &&
              ((gl || source_stage) ? last.alignment_output_ts == 888 : last.alignment_output_ts > 888) &&
              last.stereo_output_ts == last.alignment_output_ts, "Composition lost stage timing");
        roundtrip(buffer);
        ++frames; gst_sample_unref(sample);
    }
    check(frames == 6, "Unexpected number of composed frames");
    gst_object_unref(sink); stop(pipeline);

    // Exercise the real unixfd serializer with the composed metadata.
    const std::string socket = "@dvrk:test:hardware_metadata_" + std::to_string(getpid());
    auto *producer = parse("videotestsrc is-live=true ! video/x-raw,format=RGBA,width=16,height=8,framerate=30/1 "
                           "! identity name=stamp ! " + dvrk_gst::build_sink(socket));
    auto *stamp = gst_bin_get_by_name(GST_BIN(producer), "stamp");
    auto *pad = gst_element_get_static_pad(stamp, "src");
    last.overlay_output_ts = 999999;
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
        [](GstPad *, GstPadProbeInfo *info, gpointer data) {
            auto *buffer = gst_buffer_make_writable(GST_PAD_PROBE_INFO_BUFFER(info));
            GST_PAD_PROBE_INFO_DATA(info) = buffer;
            dc_buffer_set_frame_timestamps(buffer, *static_cast<DcFrameTimestamps *>(data));
            return GST_PAD_PROBE_OK;
        }, &last, nullptr);
    gst_object_unref(pad); gst_object_unref(stamp);
    gst_element_set_state(producer, GST_STATE_PLAYING);
    gst_element_get_state(producer, nullptr, nullptr, 2 * GST_SECOND);
    auto *consumer = parse(dvrk_gst::build_src(socket) + " ! appsink name=result sync=false");
    sink = gst_bin_get_by_name(GST_BIN(consumer), "result");
    gst_element_set_state(consumer, GST_STATE_PLAYING);
    auto *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 5 * GST_SECOND);
    check(sample, "No unixfd sample received");
    const auto received = dc_buffer_get_frame_timestamps(gst_sample_get_buffer(sample));
    check(received.left_reference_timestamps.size() == 2 && received.right_reference_timestamps.size() == 2,
          "unixfd dropped per-eye reference timestamps");
    check(received.left_reference_timestamps[0].timestamp == last.left_reference_timestamps[0].timestamp &&
          received.right_reference_timestamps[0].reference == last.right_reference_timestamps[0].reference &&
          received.overlay_output_ts == last.overlay_output_ts && received.alignment_output_ts == last.alignment_output_ts,
          "unixfd changed reference or stage timing");
    gst_sample_unref(sample); gst_object_unref(sink); stop(consumer); stop(producer);
    std::cout << "Selected-frame CPU/hardware timing, stage preservation, serialization and unixfd transport passed\n";
}
