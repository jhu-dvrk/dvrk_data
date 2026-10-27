#include "calibration_archive_fixture.hpp"
#include <opencv2/imgcodecs.hpp>
#include <iostream>

void check(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
int main() {
    using namespace dvrk_data::calibration;
    namespace fs = std::filesystem;
    CalibrationArchiveFixture fixture;
    CharucoConfig board;
    board.squares_x = 9;
    StereoCalibrator restored;
    std::string error;
    double progress = -1;
    check(CalibrationIO::load_calibration_archive(fixture.archive, board, restored, error,
        [&](double fraction, const std::string &) { progress = fraction; }), error.c_str());
    check(board.squares_x == fixture.board.squares_x && progress == 1.0 && restored.frame_count() == 2,
          "Archive did not restore board, samples or progress");
    const auto &frames = restored.get_frames();
    check(frames[0].id == 2 && frames[1].id == 10 && frames[0].charuco_ids.size() >= 6,
          "Archive lost IDs/numeric order or failed to redetect corners");
    check(cv::norm(frames[0].left_image, fixture.image, cv::NORM_INF) == 0 &&
          frames[0].left_corners.size() == frames[0].right_corners.size(),
          "Reload changed raw pixels or stereo correspondences");
    CharucoDetector detector(board);
    StereoDetection detection;
    check(detector.detect_stereo(fixture.image, fixture.image, detection), "Cannot detect synthetic board");
    check(restored.add_frame(fixture.image, fixture.image, detection) == 11, "New capture reused an imported ID");
    check(restored.add_frame(fixture.image, fixture.image, detection, 2) < 0, "Duplicate restored ID accepted");
    board.motion_threshold_px = 123.0f;
    auto fails_without_changes = [&] {
        check(!CalibrationIO::load_calibration_archive(fixture.archive, board, restored, error) && !error.empty(),
              "Malformed archive was accepted");
        check(restored.frame_count() == 3 && restored.get_frames().back().id == 11 && board.motion_threshold_px == 123.0f,
              "Failed reload changed the existing collection or board");
    };
    const fs::path archive(fixture.archive);
    const auto right = archive / "right" / "sample_000010.png";
    fs::rename(right, archive / "right" / "sample_000011.png");
    fails_without_changes();
    fs::rename(archive / "right" / "sample_000011.png", right);
    cv::imwrite(right.string(), cv::Mat(80, 80, CV_8UC3, cv::Scalar(255, 255, 255)));
    fails_without_changes();
    cv::imwrite(right.string(), cv::Mat(fixture.image.size(), CV_8UC3, cv::Scalar(255, 255, 255)));
    fails_without_changes();
    cv::imwrite(right.string(), fixture.image);
    fs::copy_file(archive / "left" / "sample_000002.png", archive / "left" / "sample_2.png");
    fails_without_changes();
    fs::remove(archive / "left" / "sample_2.png");
    fs::remove(archive / "board.json");
    fails_without_changes();
    check(CalibratorOptions().alpha_rectification == 0.2, "Incorrect default rectification alpha");
    std::cout << "Archive board, raw pixels, redetection, IDs, append and transactional failures passed\n";
}
