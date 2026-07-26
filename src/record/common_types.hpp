#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

struct FrameData {
    long long cpu_realtime_recorder_reception_ns = 0;
    long long cpu_monotonic_recorder_reception_ns = 0;
    long long cpu_realtime_mono_source_ns = 0;
    long long cpu_realtime_left_source_ns = 0;
    long long cpu_realtime_right_source_ns = 0;
    long long cpu_realtime_stereo_output_ns = 0;
    long long cpu_realtime_overlay_output_ns = 0;
    long long gst_pts_ns = GST_CLOCK_TIME_NONE;
    long long gst_dts_ns = GST_CLOCK_TIME_NONE;
    long long gst_duration_ns = GST_CLOCK_TIME_NONE;
    long long gst_running_time_ns = GST_CLOCK_TIME_NONE;
    long long gst_stream_time_ns = GST_CLOCK_TIME_NONE;
    long long gst_clock_time_ns = GST_CLOCK_TIME_NONE;
};

#endif // COMMON_TYPES_HPP
