// Python access to the same timestamp contract used by the native video tools.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pygobject.h>

#include <dvrk_data/cpu_timestamp_meta.hpp>

#include <optional>
#include <stdexcept>

namespace py = pybind11;

namespace {
GstBuffer *require_buffer(const py::object &object) {
    if (!pyg_boxed_check(object.ptr(), GST_TYPE_BUFFER))
        throw py::type_error("expected a gi.repository.Gst.Buffer");
    return pyg_boxed_get(object.ptr(), GstBuffer);
}
}

PYBIND11_MODULE(_video_timestamps, module) {
    module.doc() = "Frame timestamp metadata for PyGObject Gst.Buffer objects";
    auto gobject = py::reinterpret_steal<py::object>(pygobject_init(3, 0, 0));
    if (!gobject) throw py::error_already_set();
    gst_init(nullptr, nullptr);
    dc_frame_timestamps_meta_register();

    py::class_<DcReferenceTimestamp>(module, "ReferenceTimestamp")
        .def(py::init<>())
        .def_readwrite("reference", &DcReferenceTimestamp::reference)
        .def_readwrite("timestamp", &DcReferenceTimestamp::timestamp)
        .def_readwrite("duration", &DcReferenceTimestamp::duration);
    py::class_<DcFrameTimestamps>(module, "FrameTimestamps")
        .def(py::init<>())
        .def_readwrite("channel", &DcFrameTimestamps::channel)
        .def_readwrite("mono_source_ts", &DcFrameTimestamps::mono_source_ts)
        .def_readwrite("left_source_ts", &DcFrameTimestamps::left_source_ts)
        .def_readwrite("right_source_ts", &DcFrameTimestamps::right_source_ts)
        .def_readwrite("source_stereo_output_ts", &DcFrameTimestamps::source_stereo_output_ts)
        .def_readwrite("alignment_output_ts", &DcFrameTimestamps::alignment_output_ts)
        .def_readwrite("stereo_output_ts", &DcFrameTimestamps::stereo_output_ts)
        .def_readwrite("overlay_output_ts", &DcFrameTimestamps::overlay_output_ts)
        .def_readwrite("mono_reference_timestamps", &DcFrameTimestamps::mono_reference_timestamps)
        .def_readwrite("left_reference_timestamps", &DcFrameTimestamps::left_reference_timestamps)
        .def_readwrite("right_reference_timestamps", &DcFrameTimestamps::right_reference_timestamps);

    module.def("set_frame_timestamps", [](const py::object &object, const DcFrameTimestamps &timestamps) {
        GstBuffer *buffer = require_buffer(object);
        if (!gst_buffer_is_writable(buffer))
            throw py::value_error("Gst.Buffer must be writable before setting timestamps");
        if (!dc_buffer_set_frame_timestamps(buffer, timestamps))
            throw std::runtime_error("could not attach frame timestamp metadata");
    }, py::arg("buffer"), py::arg("timestamps"));
    module.def("get_frame_timestamps", [](const py::object &object) -> std::optional<DcFrameTimestamps> {
        GstBuffer *buffer = require_buffer(object);
        if (!dc_buffer_has_frame_timestamps(buffer)) return std::nullopt;
        return dc_buffer_get_frame_timestamps(buffer);
    }, py::arg("buffer"));
    module.def("register_frame_timestamps", [] { dc_frame_timestamps_meta_register(); });
    module.def("clock_realtime_ns", &dc_clock_realtime_ns);
}
