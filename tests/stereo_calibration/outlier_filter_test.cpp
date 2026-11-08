// SPDX-License-Identifier: BSD-2-Clause
#include "dvrk_data/calibration/stereo_calibrator.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <algorithm>
#include <stdexcept>
using namespace dvrk_data::calibration;
void check(bool v,const char *message) { if (!v) throw std::runtime_error(message); }
StereoCalibrationResult model() {
    StereoCalibrationResult c; c.success=true; c.image_size={640,480};
    c.K1=c.K2=(cv::Mat_<double>(3,3)<<500,0,260,0,500,180,0,0,1);
    c.D1=c.D2=cv::Mat::zeros(1,5,CV_64F);
    c.R=cv::Mat::eye(3,3,CV_64F); c.T=(cv::Mat_<double>(3,1)<<-6,0,0);
    cv::stereoRectify(c.K1,c.D1,c.K2,c.D2,c.image_size,c.R,c.T,c.R1,c.R2,c.P1,c.P2,c.Q);
    return c;
}
StereoCalibrator collection(const StereoCalibrationResult &c,int corrupt_corner=-1,int shifted_samples=0,int total=12,cv::Point2f corner_delta={7,5},bool unique_orientation=false) {
    StereoCalibrator calibrator;
    const cv::Mat image(c.image_size,CV_8UC3,cv::Scalar(128,128,128));
    for (int sample=0;sample<total;++sample) {
        StereoDetection d; d.is_stereo_valid=true;
        for (int y=0;y<4;++y) for (int x=0;x<6;++x) {
            d.common_ids.push_back(y*6+x); d.object_points.emplace_back(2*x-5,2*y-3,0);
        }
        const cv::Vec3d rv=(unique_orientation && sample==0) ? cv::Vec3d(.65,.65,0) : cv::Vec3d(.25,-.3,0);
        const cv::Vec3d tv(0,0,120);
        cv::projectPoints(d.object_points,rv,tv,c.K1,c.D1,d.left_common_corners);
        cv::projectPoints(d.object_points,rv,tv+cv::Vec3d(-6,0,0),c.K2,c.D2,d.right_common_corners);
        if (sample==0 && corrupt_corner>=0) d.right_common_corners[corrupt_corner]+=corner_delta;
        if (sample<shifted_samples) for (auto &p:d.right_common_corners) p.y+=8;
        check(calibrator.add_frame(image,image,d,sample+1)>0,"Cannot add synthetic sample");
    }
    return calibrator;
}
int main() {
    try {
        auto c=model();
        auto isolated=collection(c,7);
        const auto before=isolated.get_frames()[0];
        const auto diagnostic=StereoCalibrator::diagnose_sample(before,c,true);
        check(diagnostic.left_fit.valid && diagnostic.right_fit.valid && diagnostic.epipolar_by_corner_id.at(7)>4,
              "Need independent, bidirectional and corner epipolar diagnostics");
        auto report=isolated.filter_outliers(c,2);
        std::cerr << "Isolated: " << report.summary() << '\n';
        check(report.removed_corners==1 && report.removed_samples==0,"An isolated one-eye error should remove only its paired corner");
        const auto &trimmed=isolated.get_frames()[0];
        check(trimmed.charuco_ids.size()==23 && trimmed.left_corners.size()==23 && trimmed.right_corners.size()==23 &&
              trimmed.object_points.size()==23 && std::find(trimmed.charuco_ids.begin(),trimmed.charuco_ids.end(),7)==trimmed.charuco_ids.end(),
              "Corner trimming must keep both eyes and object/ID arrays aligned");
        check(cv::norm(trimmed.left_image,before.left_image)==0,"Filtering must preserve raw image pixels");
        check(!isolated.filter_outliers(c,2).removed_corners,"A second pass must not trim clean corners");
        auto unique_corner=collection(c,7,0,12,{100,0});
        report=unique_corner.filter_outliers(c,2);
        check(report.removed_corners==0 && report.protected_corners==1,
              "A corner that alone occupies a spatial/5D bin must be protected");
        auto cumulative=collection(c,-1,2);
        report=cumulative.filter_outliers(c,11);
        check(report.removed_samples==1 && report.protected_samples==1,
              "Sequential sample removals must share a cumulative coverage ledger");
        auto redundant=collection(c,-1,1);
        report=redundant.filter_outliers(c,1);
        std::cerr << "Whole: " << report.summary() << '\n';
        check(report.removed_samples==1 && redundant.frame_count()==11 && report.removed_corners==0,
              "A redundant pair with coherent bidirectional and epipolar disagreement should be removed whole");
        auto unique_angle=collection(c,-1,1,12,{7,5},true);
        report=unique_angle.filter_outliers(c,1);
        check(report.removed_samples==0 && report.protected_samples==1,
              "A unique pan/tilt view must be protected even when X/Y coverage is redundant");
        auto scarce=collection(c,-1,1);
        report=scarce.filter_outliers(c,20);
        check(report.removed_samples==0 && report.protected_samples==1,"A sample below density target must be protected");
        auto minimum=collection(c,-1,1,5);
        report=minimum.filter_outliers(c,1);
        check(minimum.frame_count()==5 && report.removed_samples==0,"Filtering must retain at least five samples");
        auto widespread=collection(c,-1,6);
        report=widespread.filter_outliers(c,1);
        check(report.widespread_disagreement && !report.removed_samples && !report.removed_corners,
              "Widespread stereo disagreement must trigger review rather than mass deletion");
        auto bad_model=c; bad_model.T=(cv::Mat_<double>(3,1)<<-10,0,0);
        auto clean=collection(c);
        report=clean.filter_outliers(bad_model,1);
        check(report.widespread_disagreement && !report.removed_samples && !report.removed_corners,
              "A poor global stereo model must not masquerade as bad corner detections");
        auto invalid=c; invalid.success=false;
        check(!clean.filter_outliers(invalid,1).removed_corners,"Unavailable calibration must not trigger filtering");
        std::cout << "Coverage-preserving outlier filtering passed\n";
    } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
