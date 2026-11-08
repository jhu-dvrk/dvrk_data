// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR

#include "dvrk_data/calibration/lens_distortion.hpp"
#include "dvrk_data/calibration/stereo_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>

using namespace dvrk_data::calibration;

namespace {
void check(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

// Independent forward lens formula, including tangential and rational terms.
double expected_max(const cv::Mat &K, const cv::Mat &D, cv::Size size) {
    const auto *d = D.ptr<double>();
    double maximum = 0;
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        const double nx = (x - K.at<double>(0, 2)) / K.at<double>(0, 0);
        const double ny = (y - K.at<double>(1, 2)) / K.at<double>(1, 1);
        const double r2 = nx * nx + ny * ny;
        const double radial = (1 + d[0]*r2 + d[1]*r2*r2 + d[4]*r2*r2*r2) /
            (D.total() == 8 ? 1 + d[5]*r2 + d[6]*r2*r2 + d[7]*r2*r2*r2 : 1);
        const double dx = K.at<double>(0, 0) * (nx*(radial - 1) + 2*d[2]*nx*ny + d[3]*(r2 + 2*nx*nx));
        const double dy = K.at<double>(1, 1) * (ny*(radial - 1) + d[2]*(r2 + 2*ny*ny) + 2*d[3]*nx*ny);
        maximum = std::max(maximum, std::hypot(dx, dy));
    }
    return maximum;
}
}

int main() {
    try {
        StereoCalibrationResult calibration;
        calibration.success = true;
        calibration.image_size = {320, 240};
        calibration.K1 = (cv::Mat_<double>(3, 3) << 200,0,160,0,210,120,0,0,1);
        calibration.K2 = calibration.K1.clone();
        calibration.D1 = cv::Mat::zeros(1, 5, CV_64F);
        calibration.D2 = calibration.D1.clone();
        LensDistortionView view;
        std::string error;
        check(render_lens_distortion(calibration, view, error), "Zero-distortion view failed");
        check(view.color_max_px == 0 && view.max_displacement_px[0] < 1e-4 &&
              view.invalid_pixels[0] == 0 && view.image.cols == 640 && view.image.rows > 240,
              "Zero distortion must leave the reference grid unchanged");

        for (int coefficients : {5, 8}) {
            calibration.D1 = cv::Mat::zeros(1, coefficients, CV_64F);
            calibration.D1.at<double>(0) = -0.12;
            calibration.D1.at<double>(2) = 0.003;
            calibration.D1.at<double>(3) = -0.002;
            calibration.D2 = calibration.D1 * 2;
            if (coefficients == 8) {
                calibration.D1.at<double>(5) = 0.04;
                calibration.D2.at<double>(5) = 0.08;
            }
            check(render_lens_distortion(calibration, view, error), "Distorted grid failed");
            for (int eye = 0; eye < 2; ++eye)
                check(std::abs(view.max_displacement_px[eye] - expected_max(
                    eye ? calibration.K2 : calibration.K1, eye ? calibration.D2 : calibration.D1,
                    calibration.image_size)) < 1e-3, "Incorrect forward lens displacement");
            check(view.color_max_px == view.max_displacement_px[1] &&
                  view.max_displacement_px[1] > view.max_displacement_px[0],
                  "Both eyes must use the same scale, including the stronger lens");
        }
        if (const char *path = std::getenv("DVRK_LENS_DISTORTION_PREVIEW"))
            check(cv::imwrite(path, view.image), "Cannot save lens preview");
        const auto before = view.image.clone();
        calibration.R = cv::Mat::zeros(3, 3, CV_64F);
        calibration.T = cv::Mat::ones(3, 1, CV_64F) * 100;
        calibration.R1 = calibration.R2 = cv::Mat::ones(3, 3, CV_64F);
        calibration.P1 = calibration.P2 = cv::Mat::ones(3, 4, CV_64F);
        calibration.map1_l = cv::Mat::ones(2, 2, CV_32F);
        check(render_lens_distortion(calibration, view, error) && cv::norm(before, view.image) == 0,
              "Stereo alignment, cropping or cached maps changed the lens-only chart");

        const auto valid = calibration;
        calibration.success = false;
        check(!render_lens_distortion(calibration, view, error) && view.image.empty() && !error.empty(),
              "Uncalibrated view must report why it is unavailable");
        calibration = valid;
        calibration.K2 = valid.K2.clone(); calibration.K2.at<double>(0, 0) = 0;
        check(!render_lens_distortion(calibration, view, error) && view.image.empty(), "Invalid focal length accepted");
        calibration = valid;
        calibration.D1 = valid.D1.clone(); calibration.D1.at<double>(0) = std::numeric_limits<double>::quiet_NaN();
        check(!render_lens_distortion(calibration, view, error), "Non-finite coefficients accepted");

        calibration = valid;
        calibration.image_size = {65, 49};
        calibration.K1 = calibration.K2 = (cv::Mat_<double>(3,3) << 32,0,32,0,32,24,0,0,1);
        calibration.D1 = cv::Mat::zeros(1, 8, CV_64F); calibration.D1.at<double>(5) = -1;
        calibration.D2 = calibration.D1;
        check(render_lens_distortion(calibration, view, error) && view.invalid_pixels[0] > 0 &&
              view.invalid_pixels[1] > 0 && std::isfinite(view.color_max_px),
              "Singular rational model must skip and report undefined coordinates");
        std::cout << "Lens distortion geometry, shared scale and rectification independence passed\n";
        return 0;
    } catch (const std::exception &exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
