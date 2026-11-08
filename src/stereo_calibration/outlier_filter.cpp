// SPDX-License-Identifier: BSD-2-Clause
#include "dvrk_data/calibration/stereo_calibrator.hpp"
#include "dvrk_data/calibration/coverage_tracker.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <sstream>

namespace dvrk_data::calibration {
namespace {
double median(std::vector<double> values) {
    if (values.empty()) return 0;
    std::sort(values.begin(),values.end());
    const size_t n=values.size(); return (values[n/2]+values[(n-1)/2])/2;
}
double cutoff(const std::vector<double> &values,double floor) {
    const double center=median(values);
    std::vector<double> deviations;
    for (double v:values) deviations.push_back(std::abs(v-center));
    return std::max(floor,center+4.5*1.4826*median(deviations));
}
using Errors=std::array<double,5>; // independent L/R, cross into L/R, epipolar
struct Evidence {
    SampleDiagnostics diagnostics;
    std::vector<Errors> errors;
    Errors medians{};
    std::vector<std::vector<int>> keys;
    std::map<int,int> occupancy;
    bool valid=false, full_candidate=false;
};
bool spans_plane(const CalibrationFrame &f,const std::set<int> &removed) {
    std::vector<cv::Point3f> points;
    for (size_t i=0;i<f.charuco_ids.size();++i)
        if (!removed.count(f.charuco_ids[i])) points.push_back(f.object_points[i]);
    if (points.size()<6) return false;
    for (size_t i=1;i<points.size();++i) for (size_t j=i+1;j<points.size();++j)
        if (cv::norm((points[i]-points[0]).cross(points[j]-points[0]))>1e-6) return true;
    return false;
}
}
std::string OutlierFilterReport::summary() const {
    std::ostringstream ss;
    ss << "Removed " << removed_samples << " samples and " << removed_corners << " paired corners.\n"
       << "Coverage protected: " << protected_samples << " samples, " << protected_corners << " corners.\n"
       << "Review: " << review_samples << " samples.";
    if (widespread_disagreement) ss << " Widespread disagreement: check calibration/model before deleting samples.";
    return ss.str();
}
OutlierFilterReport StereoCalibrator::filter_outliers(const StereoCalibrationResult &calib,int target_density) {
    OutlierFilterReport report;
    if (!calib.success || m_frames.size()<5) {
        report.decisions.push_back("Need a successful calibration and at least five samples."); return report;
    }
    target_density=std::clamp(target_density,1,20);
    std::vector<Evidence> evidence(m_frames.size());
    CoverageTracker coverage;
    std::array<std::vector<double>,5> population, sample_population;
    for (size_t i=0;i<m_frames.size();++i) {
        const auto &f=m_frames[i]; auto &e=evidence[i];
        e.diagnostics=diagnose_sample(f,calib,true);
        const auto &d=e.diagnostics;
        e.valid=d.left_fit.valid && d.right_fit.valid && d.left_to_right.valid && d.right_to_left.valid &&
                d.epipolar_by_corner_id.size()==f.charuco_ids.size();
        if (!e.valid) {
            ++report.review_samples;
            report.decisions.push_back("Sample #"+std::to_string(f.id)+": unavailable diagnostics; retained for review.");
            // Unknown geometry prevents safe coverage accounting for this pass.
            continue;
        }
        coverage.add_frame(f.left_corners,f.charuco_ids,f.object_points,f.left_image.size(),
            d.left_fit.rvec,true,f.id,f.right_corners,d.left_fit.tvec,true,
            {d.right_fit.rvec,d.right_fit.tvec,true});
        std::array<std::vector<double>,5> local;
        for (size_t k=0;k<f.charuco_ids.size();++k) {
            Errors v{{d.left_fit.corners[k].magnitude_px,d.right_fit.corners[k].magnitude_px,
                d.right_to_left.corners[k].magnitude_px,d.left_to_right.corners[k].magnitude_px,
                d.epipolar_by_corner_id.at(f.charuco_ids[k])}};
            e.errors.push_back(v);
            for (size_t c=0;c<5;++c) { local[c].push_back(v[c]); population[c].push_back(v[c]); }
        }
        for (size_t c=0;c<5;++c) { e.medians[c]=median(local[c]); sample_population[c].push_back(e.medians[c]); }
    }
    // Insufficient comparable diagnostics can masquerade as a consensus.
    if (sample_population[0].size()<5) {
        report.decisions.push_back("Fewer than five fully diagnosed samples; no filtering applied."); return report;
    }
    // Require a reasonably consistent majority before trusting this model to
    // identify detection outliers. Otherwise even localized residuals may be
    // artifacts of bad intrinsics/extrinsics, especially with planar targets.
    if (median(sample_population[0])>1.5 || median(sample_population[1])>1.5 ||
        median(sample_population[2])>2.5 || median(sample_population[3])>2.5 ||
        median(sample_population[4])>1.5) {
        report.widespread_disagreement=true;
        report.review_samples=static_cast<int>(m_frames.size());
        report.decisions.push_back("Typical sample errors exceed the model-consistency gate (independent/epipolar 1.5 px, cross projection 2.5 px). No data removed; improve or inspect the calibration first.");
        return report;
    }
    Errors thresholds{}, sample_thresholds{};
    for (size_t c=0;c<5;++c) {
        const double floor=(c==2 || c==3) ? 2.5 : 1.5;
        thresholds[c]=cutoff(population[c],floor);
        sample_thresholds[c]=cutoff(sample_population[c],floor);
    }
    {
        std::ostringstream thresholds_note;
        thresholds_note << "Corner cutoffs (px), independent L/R, transfer into L/R, epipolar: ";
        for (double threshold:thresholds) thresholds_note << threshold << ' ';
        thresholds_note << "; protected density target " << target_density << ".";
        report.decisions.push_back(thresholds_note.str());
    }
    // Freeze the existing range, poses and bins for the entire pass. Re-fitting
    // after deletion could move the goalposts and conceal lost coverage.
    const auto left=coverage.nested_coverage(false,target_density), right=coverage.nested_coverage(true,target_density);
    const int bins=static_cast<int>(left.samples.size()), spatial=left.cols*left.rows;
    std::map<int,int> counts, required;
    for (size_t i=0;i<m_frames.size();++i) {
        const auto &f=m_frames[i]; auto &e=evidence[i];
        if (!e.valid) continue;
        e.keys.resize(f.charuco_ids.size());
        for (int eye=0;eye<2;++eye) {
            const auto &d=eye ? right : left;
            const auto &fit=eye ? e.diagnostics.right_fit : e.diagnostics.left_fit;
            const auto &pixels=eye ? f.right_corners : f.left_corners;
            const auto indices=CoverageTracker::corner_coverage_bins(d,pixels,f.object_points,{fit.rvec,fit.tvec,true});
            const int offset=eye*(bins+spatial);
            for (size_t k=0;k<pixels.size();++k) {
                if (indices[k]>=0) e.keys[k].push_back(offset+indices[k]);
                const auto &p=pixels[k];
                if (p.x>=0 && p.y>=0 && p.x<d.image_size.width && p.y<d.image_size.height)
                    e.keys[k].push_back(offset+bins+int(p.y*d.rows/d.image_size.height)*d.cols+int(p.x*d.cols/d.image_size.width));
            }
        }
        for (const auto &keys:e.keys) for (int key:keys) ++e.occupancy[key];
        for (const auto &[key,n]:e.occupancy) { (void)n; ++counts[key]; }
    }
    for (const auto &[key,n]:counts) required[key]=std::min(n,target_density);
    int full_candidates=0;
    for (auto &e:evidence) if (e.valid) {
        // Coherent cross-eye + epipolar disagreement, anomalous relative to
        // other samples. Independent RMS alone never removes a whole sample.
        e.full_candidate=e.medians[2]>sample_thresholds[2] && e.medians[3]>sample_thresholds[3] &&
                         e.medians[4]>sample_thresholds[4];
        if (e.full_candidate) ++full_candidates;
    }
    report.widespread_disagreement=full_candidates>std::max(1,int(m_frames.size()/3));
    std::set<size_t> removed_frames;
    std::vector<std::set<int>> removed_ids(m_frames.size());
    auto permitted = [&](const Evidence &e,const std::vector<int> &keys,bool whole) {
        for (int key:keys)
            if ((whole || e.occupancy.at(key)==1) && counts[key]-1<required[key]) return false;
        return true;
    };
    std::vector<size_t> order;
    for (size_t i=0;i<m_frames.size();++i) if (evidence[i].valid) order.push_back(i);
    std::stable_sort(order.begin(),order.end(),[&](size_t a,size_t b) { return evidence[a].medians[4]>evidence[b].medians[4]; });
    for (size_t i:order) {
        auto &e=evidence[i]; const auto &f=m_frames[i];
        const auto prefix="Sample #"+std::to_string(f.id)+": ";
        if (e.full_candidate && !report.widespread_disagreement) {
            std::vector<int> keys; for (const auto &[key,n]:e.occupancy) { (void)n; keys.push_back(key); }
            if (order.size()-removed_frames.size()>5 && permitted(e,keys,true)) {
                for (int key:keys) --counts[key];
                removed_frames.insert(i); ++report.removed_samples;
                report.decisions.push_back(prefix+"removed: bidirectional transfer and epipolar disagreement.");
                continue;
            }
            ++report.protected_samples;
            report.decisions.push_back(prefix+"retained: coverage density or five-sample minimum would be lost.");
        }
        std::array<std::vector<double>,2> local;
        for (const auto &v:e.errors) for (size_t c=0;c<2;++c) local[c].push_back(v[c]);
        const double local_l=cutoff(local[0],1.5), local_r=cutoff(local[1],1.5);
        std::vector<size_t> candidates;
        for (size_t k=0;k<e.errors.size();++k) {
            const auto &v=e.errors[k];
            const bool bad_l=v[0]>thresholds[0] && v[0]>local_l && (v[2]>thresholds[2] || v[4]>thresholds[4]);
            const bool bad_r=v[1]>thresholds[1] && v[1]>local_r && (v[3]>thresholds[3] || v[4]>thresholds[4]);
            if (bad_l || bad_r) candidates.push_back(k);
        }
        // Many bad corners may indicate model/pose trouble. Do not hollow out a sample.
        if (candidates.size()>e.errors.size()/4 || (report.widespread_disagreement && (e.medians[2]>2.5 || e.medians[3]>2.5))) {
            ++report.review_samples;
            report.decisions.push_back(prefix+"widespread residuals; retained for model/detection review."); continue;
        }
        if (candidates.empty() && !e.full_candidate && !report.widespread_disagreement &&
            e.medians[2]>sample_thresholds[2] && e.medians[3]>sample_thresholds[3]) {
            ++report.review_samples;
            report.decisions.push_back(prefix+"cross projections disagree without epipolar corroboration; retained for review.");
        }
        std::stable_sort(candidates.begin(),candidates.end(),[&](size_t a,size_t b) {
            return std::max(e.errors[a][0],e.errors[a][1])>std::max(e.errors[b][0],e.errors[b][1]);
        });
        for (size_t k:candidates) {
            const int id=f.charuco_ids[k]; auto trial=removed_ids[i]; trial.insert(id);
            if (!spans_plane(f,trial) || !permitted(e,e.keys[k],false)) {
                ++report.protected_corners;
                report.decisions.push_back(prefix+"corner "+std::to_string(id)+" retained: coverage or corner geometry minimum."); continue;
            }
            for (int key:e.keys[k]) if (--e.occupancy[key]==0) --counts[key];
            removed_ids[i].insert(id); ++report.removed_corners;
            report.decisions.push_back(prefix+"corner "+std::to_string(id)+" removed in both eyes: localized independent and stereo residuals.");
        }
    }
    std::vector<CalibrationFrame> retained;
    for (size_t i=0;i<m_frames.size();++i) {
        if (removed_frames.count(i)) continue;
        auto f=m_frames[i];
        if (!removed_ids[i].empty()) {
            f.charuco_ids.clear(); f.left_corners.clear(); f.right_corners.clear(); f.object_points.clear();
            f.epipolar_rms_px=std::numeric_limits<double>::quiet_NaN();
            for (size_t k=0;k<m_frames[i].charuco_ids.size();++k) if (!removed_ids[i].count(m_frames[i].charuco_ids[k])) {
                f.charuco_ids.push_back(m_frames[i].charuco_ids[k]); f.left_corners.push_back(m_frames[i].left_corners[k]);
                f.right_corners.push_back(m_frames[i].right_corners[k]); f.object_points.push_back(m_frames[i].object_points[k]);
            }
        }
        retained.push_back(std::move(f));
    }
    m_frames=std::move(retained);
    return report;
}
} // namespace dvrk_data::calibration
