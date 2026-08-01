// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2024-2026 Johns Hopkins University, LCSR
//
// dvrk_gst_socket.hpp — Abstract Unix socket naming utilities for dvrk GStreamer pipelines.
//
// Convention
// ----------
// Every inter-process video socket follows the pattern:
//   @dvrk:<role>:<name>
//
// The leading '@' is our notation for a Linux abstract-namespace socket.
// In GStreamer pipeline strings the '@' is stripped:
//   unixfdsink socket-path=dvrk:<role>:<name> socket-type=abstract
//   unixfdsrc  socket-path=dvrk:<role>:<name> socket-type=abstract
//
// Abstract sockets are not visible in the file system; they are automatically
// released by the kernel when the last file descriptor that references them is
// closed.  They appear in /proc/net/unix with their name prefixed by '@'.
//
// Fixed roles
// -----------
//   stereo_source    — camera capture app (produces left / right frames)
//   stereo_alignment — alignment / compositing app (consumes left+right, produces stereo)
//   stereo_display   — display app (produces stereo / overlay for downstream)
//
// Short-name resolution in JSON
// -----------------------------
// JSON configs use "gst_input" and "gst_output" fields.  Socket values use:
//   "@dvrk:stereo_source:left"     → used as-is
//
// Discovery
// ---------
// list_sockets()  — returns all @dvrk:* abstract sockets currently in the kernel
// print_sockets() — pretty-prints the list
// check_socket()  — verifies a socket exists; prints alternatives on failure

#ifndef DVRK_GST_SOCKET_HPP
#define DVRK_GST_SOCKET_HPP

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace dvrk_gst {

// ── Fixed role names ────────────────────────────────────────────────────────

static constexpr const char *ROLE_STEREO_SOURCE    = "stereo_source";
static constexpr const char *ROLE_STEREO_ALIGNMENT = "stereo_alignment";
static constexpr const char *ROLE_STEREO_DISPLAY   = "stereo_display";

// Prefix shared by all dvrk GStreamer abstract sockets.
static constexpr const char *PREFIX = "@dvrk:";

// ── Name construction ────────────────────────────────────────────────────────

/// Build a fully-qualified abstract socket name: "@dvrk:<role>:<name>".
inline std::string make(const std::string &role, const std::string &name) {
    return std::string(PREFIX) + role + ":" + name;
}

// ── Resolution ───────────────────────────────────────────────────────────────

/// Resolve an @ value from a JSON field to a fully-qualified abstract
/// socket name.
///
/// Only the canonical "@dvrk:role:name" form is accepted.
///
/// @param socket_field  Raw value from a JSON gst_input/gst_output key.
/// @param default_role  Application role context; canonical references do not need it.
inline bool is_socket_reference(const std::string &value) {
    return value.rfind(PREFIX, 0) == 0;
}

inline std::string resolve(const std::string &socket_field,
                           const std::string &default_role = {}) {
    (void)default_role;
    if (socket_field.empty()) return {};

    if (!is_socket_reference(socket_field)) {
        return {};
    }
    return socket_field;
}

// ── GStreamer integration ─────────────────────────────────────────────────────

/// Convert a fully-qualified abstract name ("@dvrk:role:name") to the
/// value used in GStreamer's socket-path= property (strip the leading '@').
inline std::string to_gst_path(const std::string &abstract_name) {
    if (!abstract_name.empty() && abstract_name[0] == '@') {
        return abstract_name.substr(1);
    }
    return abstract_name;
}

/// Build a complete GStreamer unixfdsink pipeline fragment.
///
/// @param abstract_name  Fully-qualified name, e.g. "@dvrk:stereo_source:left".
/// @param sync           Value for the GStreamer sync= property (default false).
inline std::string build_sink(const std::string &abstract_name,
                              bool sync = false) {
    return "unixfdsink socket-path=" + to_gst_path(abstract_name) +
           " socket-type=abstract sync=" + (sync ? "true" : "false") +
           " async=false";
}

/// Build a complete GStreamer unixfdsrc pipeline fragment.
///
/// @param abstract_name  Fully-qualified name, e.g. "@dvrk:stereo_source:left".
/// @param width          Caps width.  No caps filter added when width <= 0.
/// @param height         Caps height.  No caps filter added when height <= 0.
/// @param format         Pixel format for the caps filter (default "I420").
inline std::string build_src(const std::string &abstract_name,
                             int width = 0, int height = 0,
                             const std::string &format = "I420") {
    std::string s = "unixfdsrc socket-path=" + to_gst_path(abstract_name) +
                    " socket-type=abstract do-timestamp=true";
    if (width > 0 && height > 0) {
        s += " ! video/x-raw,format=" + format +
             ",width=" + std::to_string(width) +
             ",height=" + std::to_string(height);
    }
    return s;
}

/// Return a plain pipeline unchanged, or expand an @ socket reference to src.
inline std::string build_input(const std::string &gst_input,
                               const std::string &default_role,
                               int width = 0, int height = 0,
                               const std::string &format = "I420") {
    if (!is_socket_reference(gst_input)) return gst_input;
    const auto socket_name = resolve(gst_input, default_role);
    if (socket_name.empty()) return {};
    return build_src(socket_name, width, height, format);
}

// ── Discovery ────────────────────────────────────────────────────────────────

/// Return all @dvrk:* abstract socket names currently registered in the
/// kernel, by parsing /proc/net/unix.
///
/// Abstract sockets appear in /proc/net/unix with their name prefixed by '@'
/// (representing the leading null byte of the abstract-namespace address).
/// Each name is returned in the "@dvrk:role:name" form.
inline std::vector<std::string> list_sockets() {
    std::vector<std::string> result;
    std::ifstream f("/proc/net/unix");
    if (!f.is_open()) return result;

    std::string line;
    std::getline(f, line); // skip header

    while (std::getline(f, line)) {
        // The Path column is the last whitespace-separated token.
        // Abstract socket paths start with '@' in /proc/net/unix.
        auto at = line.rfind('@');
        if (at == std::string::npos) continue;
        // Make sure the '@' is preceded by a space/tab (not part of a longer token).
        if (at > 0 && line[at - 1] != ' ' && line[at - 1] != '\t') continue;

        std::string name = line.substr(at);
        // Trim trailing whitespace.
        while (!name.empty() && std::isspace(static_cast<unsigned char>(name.back()))) {
            name.pop_back();
        }

        if (name.rfind(PREFIX, 0) == 0) {
            result.push_back(name);
        }
    }

    // Remove duplicates (server + connected client entries both appear).
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

/// Print all existing @dvrk sockets to the given stream.
inline void print_sockets(std::ostream &out = std::cout) {
    auto sockets = list_sockets();
    if (sockets.empty()) {
        out << "No @dvrk abstract sockets found." << std::endl;
        return;
    }
    out << "Available @dvrk sockets:" << std::endl;
    for (const auto &s : sockets) {
        out << "  " << s << std::endl;
    }
}

/// Check whether a socket exists.
///
/// Returns true if @p abstract_name is found in /proc/net/unix.
/// On failure, prints the expected name and lists alternatives on @p err.
inline bool check_socket(const std::string &abstract_name,
                         std::ostream &err = std::cerr) {
    auto sockets = list_sockets();
    for (const auto &s : sockets) {
        if (s == abstract_name) return true;
    }

    err << "Socket not found: " << abstract_name << std::endl;
    if (sockets.empty()) {
        err << "No @dvrk sockets are currently active." << std::endl;
    } else {
        err << "Available @dvrk sockets:" << std::endl;
        for (const auto &s : sockets) {
            err << "  " << s << std::endl;
        }
    }
    return false;
}

} // namespace dvrk_gst

#endif // DVRK_GST_SOCKET_HPP
