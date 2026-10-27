// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#include "gst_rectify.hpp"
#include "pipeline.hpp"
#include <dvrk_data/calibration/calibration_io.hpp>
#include <dvrk_data/calibration/rectification.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/latest_video.hpp>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <chrono>

namespace cal = dvrk_data::calibration;
namespace {
void check(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}
struct TemporaryDirectory {
    std::filesystem::path path;
    TemporaryDirectory() {
        char name[] = "/tmp/dvrk-rectify-XXXXXX";
        auto *result = mkdtemp(name);
        check(result, "Cannot create temporary directory");
        path = result;
    }
    ~TemporaryDirectory() { std::filesystem::remove_all(path); }
};
cal::StereoCalibrationResult calibration() {
    cal::StereoCalibrationResult r;
    r.success = true; r.image_size = {10, 8};
    r.calibration_date = "2026-10-26T18:30:00Z";
    r.K1 = (cv::Mat_<double>(3,3) << 12,0,5,0,12,4,0,0,1);
    r.K2 = r.K1.clone();
    r.D1 = cv::Mat::zeros(1,5,CV_64F); r.D2 = r.D1.clone();
    r.R = cv::Mat::eye(3,3,CV_64F);
    r.T = (cv::Mat_<double>(3,1) << -6,0,0);
    r.R1 = r.R.clone(); r.R2 = r.R.clone();
    r.P1 = cv::Mat::zeros(3,4,CV_64F); r.K1.copyTo(r.P1(cv::Rect(0,0,3,3)));
    r.P2 = r.P1.clone(); r.P2.at<double>(0,3) = -72;
    r.Q = cv::Mat::eye(4,4,CV_64F);
    r.E = cv::Mat::zeros(3,3,CV_64F); r.F = r.E.clone();
    r.baseline_mm = 6;
    return r;
}
GstElement *parse(const std::string &description) {
    GError *error = nullptr;
    auto *pipeline = gst_parse_launch(description.c_str(), &error);
    std::string message = error ? error->message : "Cannot parse pipeline";
    if (error) g_error_free(error);
    check(pipeline && message == "Cannot parse pipeline", message);
    return pipeline;
}
struct Pipeline {
    GstElement *value;
    explicit Pipeline(const std::string &description) : value(parse(description)) {}
    ~Pipeline() { gst_element_set_state(value, GST_STATE_NULL); gst_object_unref(value); }
};
void configure(GstElement *pipeline, const char *name, const std::string &file) {
    auto *element = gst_bin_get_by_name(GST_BIN(pipeline), name);
    check(element, std::string("Missing element ")+name);
    g_object_set(element, "calibration-file", file.c_str(), nullptr);
    gst_object_unref(element);
}
GstBuffer *image_buffer(int width, int height, int value, const DcFrameTimestamps &times, cv::Mat *pixels=nullptr) {
    GstVideoInfo info;
    gst_video_info_set_format(&info, GST_VIDEO_FORMAT_BGR, width, height);
    auto *buffer = gst_buffer_new_allocate(nullptr, info.size, nullptr);
    GstVideoFrame frame;
    check(gst_video_frame_map(&frame, &info, buffer, GST_MAP_WRITE), "Cannot map input");
    cv::Mat image(height, width, CV_8UC3, GST_VIDEO_FRAME_PLANE_DATA(&frame,0), GST_VIDEO_FRAME_PLANE_STRIDE(&frame,0));
    for (int y=0; y<height; ++y) for(int x=0; x<width; ++x)
        image.at<cv::Vec3b>(y,x) = cv::Vec3b((value+x*7+y*3)%256, (value+x*2+y*5)%256, (value+x*3+y*2)%256);
    if (pixels) *pixels = image.clone();
    gst_video_frame_unmap(&frame);
    GST_BUFFER_PTS(buffer) = 5*GST_SECOND;
    GST_BUFFER_DURATION(buffer) = GST_SECOND/30;
    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_MARKER);
    dc_buffer_set_frame_timestamps(buffer, times);
    auto *reference = gst_caps_from_string("timestamp/test-hardware");
    gst_buffer_add_reference_timestamp_meta(buffer, reference, 1234567, 100);
    gst_caps_unref(reference);
    return buffer;
}
void push(GstElement *source, int width, int height, int value, const DcFrameTimestamps &times, cv::Mat *pixels=nullptr) {
    check(gst_app_src_push_buffer(GST_APP_SRC(source), image_buffer(width,height,value,times,pixels)) == GST_FLOW_OK,
          "Input buffer rejected");
}
DcFrameTimestamps timestamps() {
    DcFrameTimestamps t;
    t.left_source_ts=101; t.right_source_ts=202; t.source_stereo_output_ts=303;
    t.left_reference_timestamps={{"timestamp/left",1111,10}};
    t.right_reference_timestamps={{"timestamp/right",2222,20}};
    return t;
}
void filter_pixels(const std::string &file, const cal::StereoCalibrationResult &r, bool right) {
    Pipeline pipe(std::string("appsrc name=input is-live=true format=time ! ")+
        "dvrkrectify name=rectify right-eye="+(right?"true":"false")+" ! appsink name=output sync=false wait-on-eos=false");
    configure(pipe.value,"rectify",file);
    auto *source=gst_bin_get_by_name(GST_BIN(pipe.value),"input");
    auto *sink=gst_bin_get_by_name(GST_BIN(pipe.value),"output");
    auto *initial_caps=gst_caps_from_string("video/x-raw,format=BGR,width=10,height=8,framerate=30/1");
    gst_app_src_set_caps(GST_APP_SRC(source),initial_caps); gst_caps_unref(initial_caps);
    check(gst_element_set_state(pipe.value,GST_STATE_PLAYING)!=GST_STATE_CHANGE_FAILURE,"Cannot start filter");
    cv::Mat input;
    push(source,10,8,30,timestamps(),&input);
    auto *sample=gst_app_sink_try_pull_sample(GST_APP_SINK(sink),2*GST_SECOND);
    check(sample,"No rectified frame");
    GstVideoInfo info;
    check(gst_video_info_from_caps(&info,gst_sample_get_caps(sample)),"Output has no video caps");
    GstVideoFrame frame;
    auto *buffer=gst_sample_get_buffer(sample);
    check(gst_video_frame_map(&frame,&info,buffer,GST_MAP_READ),"Cannot map output");
    cv::Mat actual(8,10,CV_8UC3,GST_VIDEO_FRAME_PLANE_DATA(&frame,0),GST_VIDEO_FRAME_PLANE_STRIDE(&frame,0));
    cv::Mat mx,my,expected;
    cv::initUndistortRectifyMap(right?r.K2:r.K1,right?r.D2:r.D1,right?r.R2:r.R1,right?r.P2:r.P1,
                               r.image_size,CV_32FC1,mx,my);
    cv::remap(input,expected,mx,my,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
    check(cv::norm(actual,expected,cv::NORM_INF)==0,"Rectified pixels differ from the OpenCV reference (including padded rows)");
    gst_video_frame_unmap(&frame);
    auto out=dc_buffer_get_frame_timestamps(buffer);
    check(out.left_source_ts==101 && out.right_source_ts==202 && out.source_stereo_output_ts==303,"Source timestamps changed");
    check(out.left_reference_timestamps.size()==1 && out.right_reference_timestamps.size()==1 &&
          out.left_reference_timestamps[0].timestamp==1111 && out.right_reference_timestamps[0].timestamp==2222,"Per-eye hardware references lost");
    check(GST_BUFFER_PTS(buffer)==5*GST_SECOND && GST_BUFFER_DURATION(buffer)==GST_SECOND/30,"PTS or duration changed");
    check(GST_BUFFER_FLAG_IS_SET(buffer,GST_BUFFER_FLAG_MARKER),"Buffer flags lost");
    check(!dc_buffer_get_reference_timestamps(buffer).empty(),"GstReferenceTimestampMeta lost");
    gst_sample_unref(sample);
    // A later caps change must fail instead of passing unrectified pixels.
    auto *caps=gst_caps_from_string("video/x-raw,format=BGR,width=12,height=8,framerate=30/1");
    gst_app_src_set_caps(GST_APP_SRC(source),caps); gst_caps_unref(caps);
    push(source,12,8,30,timestamps());
    auto *bus=gst_element_get_bus(pipe.value);
    auto *message=gst_bus_timed_pop_filtered(bus,2*GST_SECOND,GST_MESSAGE_ERROR);
    check(message,"Resolution changes must generate a pipeline error");
    gst_message_unref(message); gst_object_unref(bus);
    gst_object_unref(source); gst_object_unref(sink);
}
void stream_pipeline(const std::string &file) {
    RectifyConfig config;
    config.calibration_file=file;
    config.streams.stereo.gst_input="appsrc name=input is-live=true format=time ! video/x-raw,format=BGR,width=20,height=8,framerate=30/1";
    auto description=build_rectify_pipeline(config,{10,8});
    description.replace(description.find("fakesink sync=false"),std::string("fakesink sync=false").size(),
                         "appsink name=output sync=false max-buffers=1 drop=true wait-on-eos=false");
    Pipeline pipe(description);
    configure(pipe.value,"left_rectify",file); configure(pipe.value,"right_rectify",file);
    dvrk_video::LatestVideoSpec spec;
    spec.name="rectified";spec.width=20;spec.height=8;spec.stage=dvrk_video::OutputStage::Alignment;
    spec.inputs={{"left_cache",0,0,10,8,dvrk_video::CameraContent::Left},
                 {"right_cache",10,0,10,8,dvrk_video::CameraContent::Right}};
    dvrk_video::LatestVideoMixers mixer;
    std::string error;
    check(mixer.start(pipe.value,{spec},error),error);
    auto *sink=gst_bin_get_by_name(GST_BIN(pipe.value),"output");
    check(gst_element_set_state(pipe.value,GST_STATE_PLAYING)!=GST_STATE_CHANGE_FAILURE,"Cannot start processing pipeline");
    auto expect=[&](gint64 left,gint64 right) {
        for(int attempt=0;attempt<10;++attempt) {
            auto *sample=gst_app_sink_try_pull_sample(GST_APP_SINK(sink),GST_SECOND);
            check(sample,"Processing stalled waiting for its peer");
            auto t=dc_buffer_get_frame_timestamps(gst_sample_get_buffer(sample));
            GstVideoInfo info;
            gst_video_info_from_caps(&info,gst_sample_get_caps(sample));
            check(info.width==20 && info.height==8 && GST_VIDEO_INFO_FORMAT(&info)==GST_VIDEO_FORMAT_I420,"Output caps incompatible with display");
            gst_sample_unref(sample);
            if(t.left_source_ts==left && t.right_source_ts==right) {
                check(t.alignment_output_ts>0 && t.stereo_output_ts==t.alignment_output_ts,"Processing-stage timestamp missing");
                check(t.left_reference_timestamps.size()==1 && t.right_reference_timestamps.size()==1,"Mixer lost hardware references");
                return;
            }
        }
        throw std::runtime_error("Processing did not preserve the latest per-eye provenance");
    };
    auto *source=gst_bin_get_by_name(GST_BIN(pipe.value),"input");
    push(source,20,8,30,timestamps()); expect(101,202);
    gst_object_unref(source);
    gst_element_set_state(pipe.value,GST_STATE_NULL); mixer.stop(); gst_object_unref(sink);
}
void config_tests(const TemporaryDirectory &temp, const std::string &file) {
    const auto path=temp.path/"config.json";
    auto save=[&](const Json::Value &v){std::ofstream out(path);out<<v;};
    Json::Value root;
    root["type"]="dvrk_data:stereo_rectify@1.0.0";
    root["calibration_file"]=std::filesystem::path(file).filename().string();
    save(root);
    auto config=load_rectify_config(path.string());
    check(config.calibration_file==file,"Calibration paths must resolve relative to the JSON configuration");
    check(config.streams.stereo.gst_input=="@dvrk:stereo_source:stereo" &&
          config.streams.stereo.gst_output=="@dvrk:stereo_rectify:stereo","Incorrect default endpoints");
    auto rejected=[&](const Json::Value &v) {
        save(v); bool failed=false;
        try { load_rectify_config(path.string()); } catch(const std::exception &) {failed=true;}
        check(failed,"Invalid configuration was accepted");
    };
    auto bad=root; bad["backend"]="cuda"; rejected(bad);
    bad=root;bad["camera"]["left"]["gst_input"]="test";rejected(bad);
    bad=root;bad["camera"]["alignment"]["horizontal_shift_px"]=5;rejected(bad);
    bad=root;bad["camera"]["size"]["width"]=0;bad["camera"]["size"]["height"]=0;rejected(bad);
    bad=root;bad["gst_output"]="@dvrk:incomplete";rejected(bad);
    bad=root;bad["gst_input"]=12;rejected(bad);
}
}
int main(int argc,char **argv) {
    gst_init(&argc,&argv);
    try {
        if(argc==3 && std::string(argv[1])=="--benchmark") {
            cal::Rectification maps;
            std::string error;check(maps.load(argv[2],error),error);
            cv::Mat input(maps.image_size(),CV_8UC3,cv::Scalar(64,128,192)),left,right;
            maps.remap(input,left,false);maps.remap(input,right,true);
            auto begin=std::chrono::steady_clock::now();
            for(int i=0;i<30;++i) {maps.remap(input,left,false);maps.remap(input,right,true);}
            auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()/30;
            std::cout<<maps.image_size()<<" CPU remap: "<<ms<<" ms per stereo pair (excludes transport/color conversion)\n";
            return 0;
        }
        TemporaryDirectory temp;
        auto r=calibration();
        const auto file=(temp.path/"calibration with spaces.yaml").string();
        check(cal::CalibrationIO::save_stereo_calibration_file(file,r),"Cannot write test calibration");
        config_tests(temp,file);
        check(register_rectify_element(),"Cannot register filter");
        filter_pixels(file,r,false);filter_pixels(file,r,true);
        stream_pipeline(file);
        r.model=cal::DistortionModel::RATIONAL_POLYNOMIAL;
        r.D1=cv::Mat::zeros(1,14,CV_64F);r.D2=r.D1.clone();
        r.D1.at<double>(0,0)=0.1;r.D2.at<double>(0,5)=0.08;
        cv::Rodrigues(cv::Vec3d(0,0,0.08),r.R1);cv::Rodrigues(cv::Vec3d(0,0,-0.04),r.R2);
        check(cal::CalibrationIO::save_stereo_calibration_file(file,r),"Cannot write rational calibration");
        filter_pixels(file,r,false);filter_pixels(file,r,true);
        cal::Rectification maps;std::string error;
        check(maps.load(file,error),error);
        cv::Mat wrong(8,12,CV_8UC3),output;bool rejected=false;
        try{maps.remap(wrong,output,false);}catch(const std::exception &){rejected=true;}
        check(rejected,"Strict rectification must reject incompatible frames");
        check(!maps.load((temp.path/"missing.yaml").string(),error),"Missing calibration accepted");
        rejected=false;try{maps.remap(cv::Mat(8,10,CV_8UC3),output,false);}catch(const std::exception &){rejected=true;}
        check(rejected,"A failed reload must invalidate previous maps");
        std::cout<<"Stereo rectification pixels, rational distortion, caps, config and stereo timestamps PASSED\n";
        return 0;
    }catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}
