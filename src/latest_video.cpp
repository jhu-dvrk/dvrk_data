#include <dvrk_data/latest_video.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <gst/allocators/gstshmallocator.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <gst/video/video-blend.h>
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace dvrk_video {
namespace {

struct SampleRef {
  GstSample *sample = nullptr;
  explicit SampleRef(GstSample *s = nullptr) : sample(s) {}
  ~SampleRef() { if (sample) gst_sample_unref(sample); }
  SampleRef(const SampleRef &) = delete;
  SampleRef &operator=(const SampleRef &) = delete;
  SampleRef(SampleRef &&o) noexcept : sample(o.sample) { o.sample = nullptr; }
};

struct FrameMap {
  GstVideoFrame frame{};
  bool mapped = false;
  ~FrameMap() { if (mapped) gst_video_frame_unmap(&frame); }
  void map(const GstVideoInfo &info, GstBuffer *buffer, GstMapFlags flags) {
    mapped = gst_video_frame_map(&frame, &info, buffer, flags);
    if (!mapped) throw std::runtime_error("Cannot map latest-video frame");
  }
};

class Mixer {
public:
  explicit Mixer(const LatestVideoSpec &spec) : m_spec(spec), m_inputs(spec.inputs.size()) {
    if (spec.width <= 0 || spec.height <= 0 || spec.inputs.empty())
      throw std::runtime_error("Invalid latest-video canvas: " + spec.name);
    for (size_t i = 0; i < m_inputs.size(); ++i) {
      m_inputs[i].owner = this;
      m_inputs[i].index = i;
    }
  }
  ~Mixer() { stop(); }

  const std::string &name() const { return m_spec.name; }

  bool update_positions(const std::vector<LatestVideoPosition> &positions) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto candidate = m_positions;
    std::vector<bool> seen(m_inputs.size(), false);
    for (const auto &position : positions) {
      const auto it = std::find_if(m_spec.inputs.begin(), m_spec.inputs.end(),
          [&](const auto &input) { return input.name == position.input; });
      if (it == m_spec.inputs.end()) return false;
      const auto index = static_cast<size_t>(it - m_spec.inputs.begin());
      if (seen[index] || position.x < 0 || position.y < 0 ||
          position.x + static_cast<int64_t>(it->width) > m_spec.width ||
          position.y + static_cast<int64_t>(it->height) > m_spec.height) return false;
      seen[index] = true;
      candidate[index] = {position.x, position.y};
    }
    m_positions = std::move(candidate);
    ++m_revision;
    m_changed.notify_one();
    return true;
  }

  bool start(GstElement *pipeline, std::string &error) {
    for (const auto &input : m_spec.inputs) m_positions.emplace_back(input.x, input.y);
    m_output = gst_bin_get_by_name(GST_BIN(pipeline), m_spec.name.c_str());
    if (!m_output || !GST_IS_APP_SRC(m_output)) {
      error = "Missing latest-video appsrc: " + m_spec.name;
      return false;
    }
    for (auto &input : m_inputs) {
      const auto &spec = m_spec.inputs[input.index];
      if (spec.width <= 0 || spec.height <= 0 || spec.x < 0 || spec.y < 0 ||
          spec.x + static_cast<gint64>(spec.width) > m_spec.width ||
          spec.y + static_cast<gint64>(spec.height) > m_spec.height) {
        error = "Invalid latest-video rectangle: " + spec.name;
        return false;
      }
      input.sink = gst_bin_get_by_name(GST_BIN(pipeline), spec.name.c_str());
      if (!input.sink || !GST_IS_APP_SINK(input.sink)) {
        error = "Missing latest-video appsink: " + spec.name;
        return false;
      }
      GstAppSinkCallbacks callbacks{};
      callbacks.new_sample = [](GstAppSink *sink, gpointer data) {
        auto &in = *static_cast<Input *>(data);
        auto *sample = gst_app_sink_pull_sample(sink);
        if (!sample) return GST_FLOW_EOS;
        GstSample *previous;
        {
          std::lock_guard<std::mutex> lock(in.owner->m_mutex);
          previous = in.latest;
          in.latest = sample;
          in.eos = false;
          ++in.owner->m_revision;
        }
        if (previous) gst_sample_unref(previous);
        in.owner->m_changed.notify_one();
        return GST_FLOW_OK;
      };
      callbacks.eos = [](GstAppSink *, gpointer data) {
        auto &in = *static_cast<Input *>(data);
        {
          std::lock_guard<std::mutex> lock(in.owner->m_mutex);
          in.eos = true;
          ++in.owner->m_revision;
        }
        in.owner->m_changed.notify_one();
      };
      gst_app_sink_set_callbacks(GST_APP_SINK(input.sink), &callbacks, &input, nullptr);
      input.pad = gst_element_get_static_pad(input.sink, "sink");
      input.probe = gst_pad_add_probe(input.pad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM,
        [](GstPad *, GstPadProbeInfo *info, gpointer data) {
          auto &in = *static_cast<Input *>(data);
          if (GST_EVENT_TYPE(GST_PAD_PROBE_INFO_EVENT(info)) == GST_EVENT_STREAM_START) {
            GstSample *previous;
            {
              std::lock_guard<std::mutex> lock(in.owner->m_mutex);
              previous = in.latest;
              in.latest = nullptr;
              in.eos = false;
            }
            if (previous) gst_sample_unref(previous);
          }
          return GST_PAD_PROBE_OK;
        }, &input, nullptr);
    }
    m_worker = std::thread([this] { run(); });
    return true;
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_stopping = true;
    }
    m_changed.notify_one();
    if (m_worker.joinable()) m_worker.join();
    for (auto &input : m_inputs) {
      if (input.pad) {
        gst_pad_remove_probe(input.pad, input.probe);
        gst_object_unref(input.pad);
        input.pad = nullptr;
      }
      if (input.sink) {
        GstAppSinkCallbacks callbacks{};
        gst_app_sink_set_callbacks(GST_APP_SINK(input.sink), &callbacks, nullptr, nullptr);
        gst_object_unref(input.sink);
        input.sink = nullptr;
      }
      if (input.latest) { gst_sample_unref(input.latest); input.latest = nullptr; }
      if (input.converter) { gst_video_converter_free(input.converter); input.converter = nullptr; }
      if (input.scratch) { gst_buffer_unref(input.scratch); input.scratch = nullptr; }
    }
    if (m_pool) {
      gst_buffer_pool_set_active(m_pool, FALSE);
      gst_object_unref(m_pool);
      m_pool = nullptr;
    }
    if (m_output) { gst_object_unref(m_output); m_output = nullptr; }
  }

private:
  struct Input {
    Mixer *owner = nullptr;
    size_t index = 0;
    GstElement *sink = nullptr;
    GstPad *pad = nullptr;
    gulong probe = 0;
    GstSample *latest = nullptr;
    bool eos = false;
    GstVideoInfo info{};
    GstVideoConverter *converter = nullptr;
    GstVideoInfo scratch_info{};
    GstBuffer *scratch = nullptr;
    int x = 0, y = 0;
  };

  void configure_output(const std::vector<SampleRef> &samples) {
    gst_video_info_set_format(&m_info, m_spec.rgba ? GST_VIDEO_FORMAT_RGBA : GST_VIDEO_FORMAT_I420,
                              m_spec.width, m_spec.height);
    gst_video_colorimetry_from_string(&m_info.colorimetry,
        m_spec.rgba ? "sRGB" : (m_spec.height > 576 ? "bt709" : "bt601"));
    for (const auto &sample : samples) {
      GstVideoInfo info;
      if (sample.sample && gst_video_info_from_caps(&info, gst_sample_get_caps(sample.sample))) {
        if (static_cast<bool>(GST_VIDEO_INFO_IS_RGB(&info)) == m_spec.rgba) {
          m_info.colorimetry = info.colorimetry;
          m_info.chroma_site = info.chroma_site;
        }
        break;
      }
    }
    // Updates are driven by arrival, not a negotiated compositor clock/rate.
    m_info.fps_n = 0;
    m_info.fps_d = 1;
    auto *caps = gst_video_info_to_caps(&m_info);
    gst_app_src_set_caps(GST_APP_SRC(m_output), caps);
    gst_shm_allocator_init_once();
    auto *allocator = gst_shm_allocator_get();
    m_pool = gst_video_buffer_pool_new();
    auto *config = gst_buffer_pool_get_config(m_pool);
    gst_buffer_pool_config_set_params(config, caps, m_info.size, 0, 0);
    gst_buffer_pool_config_set_allocator(config, allocator, nullptr);
    gst_buffer_pool_config_add_option(config, GST_BUFFER_POOL_OPTION_VIDEO_META);
    gst_object_unref(allocator);
    gst_caps_unref(caps);
    if (!gst_buffer_pool_set_config(m_pool, config) || !gst_buffer_pool_set_active(m_pool, TRUE))
      throw std::runtime_error("Cannot allocate shared latest-video frames");
  }

  void configure_input(Input &input, const GstVideoInfo &info) {
    if (input.converter) { gst_video_converter_free(input.converter); input.converter = nullptr; }
    if (input.scratch) { gst_buffer_unref(input.scratch); input.scratch = nullptr; }
    input.info = info;
    const auto &region = m_spec.inputs[input.index];
    int width = region.width, height = region.height;
    input.x = region.x; input.y = region.y;
    if (region.preserve_aspect) {
      const double aspect = static_cast<double>(info.width) * info.par_n / (info.height * info.par_d);
      width = std::min(width, std::max(1, static_cast<int>(std::round(height * aspect))));
      height = std::min(height, std::max(1, static_cast<int>(std::round(width / aspect))));
      input.x += (region.width - width) / 2;
      input.y += static_cast<int>((region.height - height) * region.yalign);
    }
    auto *config = gst_structure_new("latest-video-converter",
        GST_VIDEO_CONVERTER_OPT_FILL_BORDER, G_TYPE_BOOLEAN, FALSE,
        GST_VIDEO_CONVERTER_OPT_DEST_X, G_TYPE_INT, region.blend ? 0 : input.x,
        GST_VIDEO_CONVERTER_OPT_DEST_Y, G_TYPE_INT, region.blend ? 0 : input.y,
        GST_VIDEO_CONVERTER_OPT_DEST_WIDTH, G_TYPE_INT, width,
        GST_VIDEO_CONVERTER_OPT_DEST_HEIGHT, G_TYPE_INT, height, nullptr);
    const GstVideoInfo *destination = &m_info;
    if (region.blend) {
      if (!m_spec.rgba) throw std::runtime_error("Alpha blending requires RGBA");
      gst_video_info_set_format(&input.scratch_info, GST_VIDEO_FORMAT_RGBA, width, height);
      input.scratch_info.colorimetry = m_info.colorimetry;
      input.scratch = gst_buffer_new_allocate(nullptr, input.scratch_info.size, nullptr);
      destination = &input.scratch_info;
    }
    input.converter = gst_video_converter_new(&input.info, destination, config);
    if (!input.converter) throw std::runtime_error("Cannot convert latest-video input");
  }

  void run() {
    uint64_t rendered = 0;
    bool eos_sent = false;
    try {
      while (true) {
        std::vector<SampleRef> samples;
        std::vector<std::pair<int, int>> positions;
        bool all_eos;
        {
          std::unique_lock<std::mutex> lock(m_mutex);
          m_changed.wait(lock, [&] { return m_stopping || m_revision != rendered; });
          if (m_stopping) break;
          rendered = m_revision;
          positions = m_positions;
          all_eos = true;
          for (const auto &input : m_inputs) {
            samples.emplace_back(input.latest ? gst_sample_ref(input.latest) : nullptr);
            all_eos = all_eos && input.eos;
          }
        }
        // Only this worker touches rendering specs and converters. Position
        // requests share the sample mutex but never wait for conversion.
        for (size_t i = 0; i < positions.size(); ++i) {
          auto &region = m_spec.inputs[i];
          if (region.x == positions[i].first && region.y == positions[i].second) continue;
          region.x = positions[i].first;
          region.y = positions[i].second;
          auto &input = m_inputs[i];
          if (input.converter) { gst_video_converter_free(input.converter); input.converter = nullptr; }
        }
        if (std::any_of(samples.begin(), samples.end(), [](const SampleRef &s) { return s.sample; })) {
          if (!m_pool) configure_output(samples);
          render(samples);
        }
        if (all_eos && !eos_sent) {
          gst_app_src_end_of_stream(GST_APP_SRC(m_output));
          eos_sent = true;
        }
        if (!all_eos) eos_sent = false;
      }
    } catch (const std::exception &error) {
      GST_ELEMENT_ERROR(m_output, STREAM, FAILED, ("%s", error.what()), (nullptr));
    }
  }

  void render(const std::vector<SampleRef> &samples) {
    GstBuffer *buffer = nullptr;
    if (gst_buffer_pool_acquire_buffer(m_pool, &buffer, nullptr) != GST_FLOW_OK) return;
    std::unique_ptr<GstBuffer, decltype(&gst_buffer_unref)> pending(buffer, gst_buffer_unref);
    FrameMap output_map;
    output_map.map(m_info, buffer, GST_MAP_READWRITE);
    auto &output = output_map.frame;
    if (m_spec.rgba) {
      auto *data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(&output, 0));
      for (int y = 0; y < m_info.height; ++y) {
        auto *row = data + y * GST_VIDEO_FRAME_PLANE_STRIDE(&output, 0);
        std::memset(row, 0, m_info.width * 4);
        for (int x = 0; x < m_info.width; ++x) row[4 * x + 3] = 255;
      }
    } else {
      const int black = m_info.colorimetry.range == GST_VIDEO_COLOR_RANGE_0_255 ? 0 : 16;
      for (int plane = 0; plane < 3; ++plane) {
        auto *data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(&output, plane));
        for (int y = 0; y < GST_VIDEO_FRAME_COMP_HEIGHT(&output, plane); ++y)
          std::memset(data + y * GST_VIDEO_FRAME_PLANE_STRIDE(&output, plane), plane ? 128 : black,
                      GST_VIDEO_FRAME_COMP_WIDTH(&output, plane));
      }
    }
    DcFrameTimestamps timestamps;
    bool common_set = false;
    unsigned selected_eyes = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
      if (!samples[i].sample) continue;
      auto &input = m_inputs[i];
      GstVideoInfo info;
      if (!gst_video_info_from_caps(&info, gst_sample_get_caps(samples[i].sample))) continue;
      info.fps_n = 0; info.fps_d = 1;
      if (!input.converter || !gst_video_info_is_equal(&input.info, &info)) configure_input(input, info);
      auto *frame_buffer = gst_sample_get_buffer(samples[i].sample);
      FrameMap frame;
      frame.map(info, frame_buffer, GST_MAP_READ);
      if (m_spec.inputs[i].blend) {
        FrameMap scratch;
        scratch.map(input.scratch_info, input.scratch, GST_MAP_READWRITE);
        gst_video_converter_frame(input.converter, &frame.frame, &scratch.frame);
        if (!gst_video_blend(&output, &scratch.frame, input.x, input.y, 1.0f))
          throw std::runtime_error("Cannot blend latest-video artwork");
      } else {
        gst_video_converter_frame(input.converter, &frame.frame, &output);
      }
      const auto camera = m_spec.inputs[i].camera;
      if (camera == CameraContent::None) continue;
      const auto selected = dc_buffer_get_frame_timestamps(frame_buffer);
      selected_eyes |= camera == CameraContent::Left ? 1 : camera == CameraContent::Right ? 2 : 3;
      if (camera == CameraContent::Stereo) {
        timestamps = selected;
        common_set = true;
        continue;
      }
      if (!common_set) {
        timestamps.source_stereo_output_ts = selected.source_stereo_output_ts;
        timestamps.alignment_output_ts = selected.alignment_output_ts;
        timestamps.stereo_output_ts = selected.stereo_output_ts;
        timestamps.overlay_output_ts = selected.overlay_output_ts;
        common_set = true;
      } else {
        if (timestamps.source_stereo_output_ts != selected.source_stereo_output_ts) timestamps.source_stereo_output_ts = 0;
        if (timestamps.alignment_output_ts != selected.alignment_output_ts) timestamps.alignment_output_ts = 0;
        if (timestamps.stereo_output_ts != selected.stereo_output_ts) timestamps.stereo_output_ts = 0;
        if (timestamps.overlay_output_ts != selected.overlay_output_ts) timestamps.overlay_output_ts = 0;
      }
      if (camera == CameraContent::Left) {
        timestamps.left_source_ts = selected.left_source_ts;
        timestamps.left_reference_timestamps = selected.left_reference_timestamps;
      } else {
        timestamps.right_source_ts = selected.right_source_ts;
        timestamps.right_reference_timestamps = selected.right_reference_timestamps;
      }
    }
    gst_video_frame_unmap(&output);
    output_map.mapped = false;
    if (selected_eyes != 3) {
      timestamps.source_stereo_output_ts = 0;
      timestamps.alignment_output_ts = 0;
      timestamps.stereo_output_ts = 0;
      timestamps.overlay_output_ts = 0;
    }
    timestamps.channel = "stereo-LR";
    if (m_spec.stage == OutputStage::Source) {
      timestamps.source_stereo_output_ts = dc_clock_realtime_ns();
    } else if (m_spec.stage == OutputStage::Alignment) {
      timestamps.alignment_output_ts = dc_clock_realtime_ns();
      timestamps.stereo_output_ts = timestamps.alignment_output_ts;
    }
    dc_buffer_set_frame_timestamps(buffer, timestamps);
    GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE;
    gst_app_src_push_buffer(GST_APP_SRC(m_output), pending.release());
  }

  LatestVideoSpec m_spec;
  std::vector<Input> m_inputs;
  GstElement *m_output = nullptr;
  GstBufferPool *m_pool = nullptr;
  GstVideoInfo m_info{};
  std::mutex m_mutex;
  std::condition_variable m_changed;
  std::thread m_worker;
  bool m_stopping = false;
  uint64_t m_revision = 0;
  std::vector<std::pair<int, int>> m_positions;
};
} // namespace

struct LatestVideoMixers::Impl { std::vector<std::unique_ptr<Mixer>> mixers; };
LatestVideoMixers::LatestVideoMixers() : m_impl(std::make_unique<Impl>()) {}
LatestVideoMixers::~LatestVideoMixers() = default;
std::string LatestVideoMixers::sink(const std::string &name) {
  return "appsink name=" + name + " sync=false async=false max-buffers=1 drop=true"
      " enable-last-sample=false wait-on-eos=false";
}
std::string LatestVideoMixers::source(const std::string &name) {
  return "appsrc name=" + name + " is-live=true format=time do-timestamp=true"
      " block=false max-buffers=1 max-bytes=0 leaky-type=downstream";
}
bool LatestVideoMixers::start(GstElement *pipeline, const std::vector<LatestVideoSpec> &specs,
                             std::string &error) {
  stop();
  try {
    for (const auto &spec : specs) {
      auto mixer = std::make_unique<Mixer>(spec);
      if (!mixer->start(pipeline, error)) { stop(); return false; }
      m_impl->mixers.push_back(std::move(mixer));
    }
    return true;
  } catch (const std::exception &exception) {
    error = exception.what();
    stop();
    return false;
  }
}
void LatestVideoMixers::stop() { m_impl->mixers.clear(); }
bool LatestVideoMixers::update_positions(const std::string &name,
    const std::vector<LatestVideoPosition> &positions) {
  for (const auto &mixer : m_impl->mixers)
    if (mixer->name() == name) return mixer->update_positions(positions);
  return false;
}
} // namespace dvrk_video
