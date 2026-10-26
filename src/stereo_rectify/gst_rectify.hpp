// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#ifndef DVRK_DATA_GST_RECTIFY_HPP
#define DVRK_DATA_GST_RECTIFY_HPP

// Register the process-local dvrkrectify GStreamer element. It accepts BGR
// frames and properties calibration-file (path) and right-eye (boolean).
bool register_rectify_element();
#endif
