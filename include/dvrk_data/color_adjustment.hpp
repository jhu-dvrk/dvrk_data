// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#ifndef DVRK_DATA_COLOR_ADJUSTMENT_HPP
#define DVRK_DATA_COLOR_ADJUSTMENT_HPP
#include <dvrk_data/config.hpp>
#include <string>

namespace dvrk_data {
// Load an explicitly named file, or discover stereo_color_adjustment.json next
// to the processor configuration. No discovered file means neutral correction;
// an explicitly named missing file or an invalid file is always an error.
std::string load_stereo_color_adjustment(AppConfig &config,
    const std::string &processor_config, const std::string &explicit_file = "");
}
#endif
