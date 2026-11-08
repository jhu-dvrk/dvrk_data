// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/lens_distortion.hpp"
#include "dvrk_data/calibration/stereo_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace dvrk_data::calibration {
namespace {

bool drawable(const cv::Point2f &point) {
    // Bound coordinates before converting to integer OpenCV drawing coordinates.
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::max(std::abs(point.x), std::abs(point.y)) < 1e7f;
}

cv::Scalar displacement_color(double displacement, double maximum) {
    const double fraction = maximum > 0 ? std::clamp(displacement / maximum, 0.0, 1.0) : 0.0;
    return {0, 255 * std::min(1.0, 2 * (1 - fraction)), 255 * std::min(1.0, 2 * fraction)};
}

std::vector<int> grid_coordinates(int length, int divisions) {
    std::vector<int> coordinates;
    for (int i = 0; i <= divisions; ++i) {
        const int value = i * (length - 1) / divisions;
        if (coordinates.empty() || coordinates.back() != value) coordinates.push_back(value);
    }
    return coordinates;
}

} // namespace

bool render_lens_distortion(const StereoCalibrationResult &calibration,
                            LensDistortionView &output, std::string &error) {
    output = LensDistortionView();
    error.clear();
    try {
        const auto size = calibration.image_size;
        if (!calibration.success || size.width <= 0 || size.height <= 0)
            throw std::runtime_error("Run stereo calibration to display lens distortion.");

        std::array<cv::Mat, 2> maps;
        for (int eye = 0; eye < 2; ++eye) {
            const auto &intrinsics = eye ? calibration.K2 : calibration.K1;
            const auto &distortion = eye ? calibration.D2 : calibration.D1;
            if (intrinsics.size() != cv::Size(3, 3) || intrinsics.channels() != 1 ||
                !cv::checkRange(intrinsics) || distortion.empty() || distortion.channels() != 1 ||
                (distortion.rows != 1 && distortion.cols != 1) || !cv::checkRange(distortion))
                throw std::runtime_error("Missing or invalid lens intrinsics/distortion coefficients.");
            cv::Mat K;
            intrinsics.convertTo(K, CV_64F);
            if (K.at<double>(0, 0) <= 0 || K.at<double>(1, 1) <= 0 ||
                K.at<double>(2, 0) != 0 || K.at<double>(2, 1) != 0 || K.at<double>(2, 2) != 1)
                throw std::runtime_error("Invalid camera matrix for lens distortion.");

            // The remap's destination is an ideal pixel and its value is the
            // corresponding distorted sensor pixel. Identity R and unchanged K
            // deliberately exclude stereo alignment and rectification cropping.
            cv::initUndistortRectifyMap(K, distortion, cv::Mat::eye(3, 3, CV_64F), K,
                                       size, CV_32FC2, maps[eye], cv::noArray());
            for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
                const auto point = maps[eye].at<cv::Point2f>(y, x);
                if (!drawable(point)) { ++output.invalid_pixels[eye]; continue; }
                output.max_displacement_px[eye] = std::max(output.max_displacement_px[eye],
                    cv::norm(point - cv::Point2f(x, y)));
            }
            if (output.invalid_pixels[eye] == size.area())
                throw std::runtime_error("Lens model has no drawable pixels.");
        }
        output.color_max_px = std::max(output.max_displacement_px[0], output.max_displacement_px[1]);
        // Float remap coordinates can produce subpixel roundoff for zero distortion.
        if (output.color_max_px < 1e-4) output.color_max_px = 0.0;

        constexpr int header = 34, footer = 36;
        output.image = cv::Mat(size.height + header + footer, 2 * size.width,
                               CV_8UC3, cv::Scalar(22, 22, 22));
        const auto xs = grid_coordinates(size.width, 10);
        const auto ys = grid_coordinates(size.height, 8);
        for (int eye = 0; eye < 2; ++eye) {
            auto panel = output.image(cv::Rect(eye * size.width, header, size.width, size.height));
            for (int x : xs) cv::line(panel, {x, 0}, {x, size.height - 1}, {65, 65, 65});
            for (int y : ys) cv::line(panel, {0, y}, {size.width - 1, y}, {65, 65, 65});
            auto segment = [&](cv::Point2f a, cv::Point2f b, const cv::Scalar &color, bool arrow) {
                if (!drawable(a) || !drawable(b)) return;
                cv::Point first = a, last = b;
                if (!cv::clipLine(size, first, last)) return;
                if (arrow) cv::arrowedLine(panel, first, last, color, 1, cv::LINE_AA, 0, 0.25);
                else cv::line(panel, first, last, color, 1, cv::LINE_AA);
            };
            auto curve = [&](cv::Point a, cv::Point b) {
                const auto warped_a = maps[eye].at<cv::Point2f>(a);
                const auto warped_b = maps[eye].at<cv::Point2f>(b);
                segment(warped_a, warped_b,
                        displacement_color(cv::norm(warped_b - cv::Point2f(b)), output.color_max_px), false);
            };
            for (int x : xs) for (int y = 1; y < size.height; ++y) curve({x, y - 1}, {x, y});
            for (int y : ys) for (int x = 1; x < size.width; ++x) curve({x - 1, y}, {x, y});
            for (int y : ys) for (int x : xs) {
                const auto warped = maps[eye].at<cv::Point2f>(y, x);
                if (!drawable(warped)) continue;
                const auto color = displacement_color(cv::norm(warped - cv::Point2f(x, y)), output.color_max_px);
                segment(cv::Point2f(x, y), warped, color, true);
            }
            std::ostringstream label;
            label << (eye ? "RIGHT" : "LEFT") << " | max " << std::fixed << std::setprecision(2)
                  << output.max_displacement_px[eye] << " px";
            cv::putText(output.image, label.str(), {eye * size.width + 8, 23},
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, {230, 230, 230}, 1, cv::LINE_AA);
        }
        cv::line(output.image, {size.width, 0}, {size.width, output.image.rows - 1}, {100, 100, 100});
        const int bar_width = std::min(160, size.width / 2);
        for (int x = 0; x < bar_width; ++x) {
            const auto color = displacement_color(output.color_max_px * x / std::max(1, bar_width - 1), output.color_max_px);
            cv::line(output.image, {8 + x, header + size.height + 10}, {8 + x, header + size.height + 22}, color);
        }
        std::ostringstream legend;
        legend << "0 - " << std::fixed << std::setprecision(2) << output.color_max_px << " px | ideal -> distorted";
        cv::putText(output.image, legend.str(), {bar_width + 18, header + size.height + 22},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, {230, 230, 230}, 1, cv::LINE_AA);
        return true;
    } catch (const std::exception &exception) {
        output = LensDistortionView();
        error = exception.what();
        return false;
    }
}

} // namespace dvrk_data::calibration
