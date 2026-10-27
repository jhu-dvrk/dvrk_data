#ifndef DVRK_DATA_LATEST_VIDEO_HPP
#define DVRK_DATA_LATEST_VIDEO_HPP

#include <gst/gst.h>
#include <memory>
#include <string>
#include <vector>

namespace dvrk_video {

enum class CameraContent { None, Left, Right, Stereo };
enum class OutputStage { Preserve, Source, Alignment };

struct LatestVideoInput {
  std::string name;
  int x = 0, y = 0, width = 0, height = 0;
  CameraContent camera = CameraContent::None;
  bool preserve_aspect = false;
  double yalign = 0.5;
  bool blend = false;
};

struct LatestVideoSpec {
  std::string name;
  int width = 0, height = 0;
  bool rgba = false;
  OutputStage stage = OutputStage::Preserve;
  std::vector<LatestVideoInput> inputs;
};

struct LatestVideoPosition {
  std::string input;
  int x, y;
};

// Event-driven spatial composition. Each input owns one cached sample; a new
// sample from ANY input renders the newest available images. Input PTS, cadence,
// and EOS on the other inputs never gate selection. Missing images are black.
// Source CPU/reference timestamps describe the selected pixels, not the new PTS.
class LatestVideoMixers {
public:
  LatestVideoMixers();
  ~LatestVideoMixers();
  LatestVideoMixers(const LatestVideoMixers &) = delete;
  LatestVideoMixers &operator=(const LatestVideoMixers &) = delete;

  static std::string sink(const std::string &name);
  static std::string source(const std::string &name);
  bool start(GstElement *pipeline, const std::vector<LatestVideoSpec> &specs,
             std::string &error);
  // Atomically queue positions for one mixer and redraw cached samples. Safe
  // during rendering; callers must serialize this with start()/stop(). Invalid
  // names, duplicate inputs or out-of-canvas rectangles leave positions intact.
  bool update_positions(const std::string &mixer,
                        const std::vector<LatestVideoPosition> &positions);
  // Call after the pipeline reaches NULL, before releasing it or restarting.
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};
} // namespace dvrk_video
#endif
