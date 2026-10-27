cmake_minimum_required(VERSION 3.16)
project(dvrk_data VERSION 1.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco videoio imgcodecs)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0 gstreamer-allocators-1.0)
pkg_check_modules(GTK REQUIRED gtk+-3.0)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

add_library(dvrk_data
  src/latest_video.cpp
  src/config.cpp
  src/color_adjustment.cpp
  src/collection_config.cpp
  src/tags.cpp
  src/gst_utils.cpp
)
target_include_directories(dvrk_data PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${JSONCPP_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
target_compile_options(dvrk_data PRIVATE
  ${JSONCPP_CFLAGS_OTHER}
  ${GST_CFLAGS_OTHER}
)
target_link_libraries(dvrk_data
  ${JSONCPP_LIBRARIES}
  ${GST_LIBRARIES}
)
target_link_libraries(dvrk_data
  ament_index_cpp::ament_index_cpp
)

add_executable(record
  src/record/main.cpp
  src/record/app_data.cpp
  src/record/gst_helpers.cpp
  src/record/video_stream.cpp
  src/record/audio_stream.cpp
  src/record/ui.cpp
  src/record/ros_node.cpp
)
target_link_libraries(record
  rclcpp::rclcpp
  std_msgs::std_msgs
  rosbag2_cpp::rosbag2_cpp
)
target_include_directories(record PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GTK_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(record
  ${GST_LIBRARIES}
  ${GTK_LIBRARIES}
  ${GTKMM_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  dvrk_data
)

add_executable(video_tag
  src/video_tag/tag_main.cpp
  src/video_tag/tag_ui.cpp
)
target_include_directories(video_tag PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GTK_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(video_tag
  ${GST_LIBRARIES}
  ${GTK_LIBRARIES}
  ${GTKMM_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  gstvideo-1.0
  dvrk_data
)

add_executable(video_latency
  src/video_latency/main.cpp
)
target_include_directories(video_latency PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GTK_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(video_latency
  ${GST_LIBRARIES}
  ${GTK_LIBRARIES}
  ${GTKMM_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  gstvideo-1.0
  dvrk_data
)

add_executable(stereo_source
  src/stereo_source/main.cpp
)
target_link_libraries(stereo_source
  rclcpp::rclcpp
)
target_include_directories(stereo_source PRIVATE
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_compile_options(stereo_source PRIVATE
  ${GST_CFLAGS_OTHER}
  ${JSONCPP_CFLAGS_OTHER}
)
target_link_libraries(stereo_source
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  dvrk_data
)

add_executable(stereo_alignment
  src/stereo_alignment/main.cpp
)
target_link_libraries(stereo_alignment
  rclcpp::rclcpp
)
target_include_directories(stereo_alignment PRIVATE
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_compile_options(stereo_alignment PRIVATE
  ${GST_CFLAGS_OTHER}
  ${JSONCPP_CFLAGS_OTHER}
)
target_link_libraries(stereo_alignment
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  dvrk_data
)

# Calibration math/IO is independent of GTK; only the interactive tools link GUI code.
add_library(dvrk_stereo_calibration
  src/stereo_calibration/board_config.cpp
  src/stereo_calibration/calibration_io.cpp
  src/stereo_calibration/charuco_detector.cpp
  src/stereo_calibration/coverage_tracker.cpp
  src/stereo_calibration/gst_stereo_receiver.cpp
  src/stereo_calibration/stereo_calibrator.cpp
  src/stereo_calibration/rectification.cpp
)
target_include_directories(dvrk_stereo_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(dvrk_stereo_calibration PUBLIC dvrk_data ${OpenCV_LIBRARIES})

add_library(dvrk_calibration_gui STATIC src/charuco_stereo_calibrator/calibration_window.cpp)
target_include_directories(dvrk_calibration_gui PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/charuco_stereo_calibrator ${GTKMM_INCLUDE_DIRS})
target_compile_options(dvrk_calibration_gui PRIVATE ${GTKMM_CFLAGS_OTHER})
target_link_libraries(dvrk_calibration_gui PUBLIC dvrk_stereo_calibration ${GTKMM_LIBRARIES})

add_executable(charuco_stereo_calibrator src/charuco_stereo_calibrator/main.cpp)
target_link_libraries(charuco_stereo_calibrator dvrk_calibration_gui)
add_executable(verify_stereo_calibration src/verify_stereo_calibration/main.cpp)
target_link_libraries(verify_stereo_calibration dvrk_calibration_gui)
add_executable(test_stereo_receiver src/test_stereo_receiver/main.cpp)
target_link_libraries(test_stereo_receiver dvrk_stereo_calibration)
add_executable(test_charuco_detector src/test_charuco_detector/main.cpp)
target_link_libraries(test_charuco_detector dvrk_stereo_calibration)

add_library(dvrk_rectify_filter STATIC
  src/stereo_rectify/gst_rectify.cpp src/stereo_rectify/pipeline.cpp)
target_include_directories(dvrk_rectify_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/stereo_rectify)
target_link_libraries(dvrk_rectify_filter PUBLIC dvrk_stereo_calibration)
add_executable(stereo_rectify src/stereo_rectify/main.cpp)
target_link_libraries(stereo_rectify dvrk_rectify_filter rclcpp::rclcpp)

add_executable(video_configurator
  src/video_configurator/video_configurator.cpp
)
target_include_directories(video_configurator PRIVATE
  ${GST_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(video_configurator
  ${GST_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

install(TARGETS dvrk_data dvrk_stereo_calibration
  EXPORT export_dvrk_data
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  record
  stereo_source
  stereo_alignment
  stereo_rectify
  charuco_stereo_calibrator
  verify_stereo_calibration
  test_stereo_receiver
  test_charuco_detector
  video_tag
  video_latency
  video_configurator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/stereo_find_color_adjustment
  scripts/generate_charuco_board.py
  scripts/synthetic_source.py
  scripts/gstreamer_ar.py
  scripts/opencv_ar.py
  scripts/gscam_socket
  scripts/smoke_test_stack
  scripts/extract
  scripts/encord_to_tags
  scripts/stereo_alignment_calibration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY share/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_dvrk_data HAS_LIBRARY_TARGET)
ament_export_libraries(dvrk_data)
ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp OpenCV)

if(BUILD_TESTING)
  enable_testing()
  add_executable(stereo_color_adjustment_test tests/stereo_color_adjustment_test.cpp)
  target_link_libraries(stereo_color_adjustment_test dvrk_data)
  add_test(NAME stereo_color_adjustment COMMAND stereo_color_adjustment_test)
  add_executable(test_stereo_calibration_math tests/stereo_calibration/test_stereo_calibration_math.cpp)
  target_link_libraries(test_stereo_calibration_math dvrk_stereo_calibration)
  add_test(NAME stereo_calibration_math COMMAND test_stereo_calibration_math)
  add_executable(calibration_archive_test tests/stereo_calibration/calibration_archive_test.cpp)
  target_link_libraries(calibration_archive_test dvrk_stereo_calibration)
  add_test(NAME calibration_archive COMMAND calibration_archive_test)
  add_executable(calibration_window_test tests/stereo_calibration/calibration_window_test.cpp)
  target_link_libraries(calibration_window_test dvrk_calibration_gui)
  add_test(NAME calibration_window COMMAND calibration_window_test)
  set_tests_properties(calibration_window PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 20)
  add_executable(test_coverage_tracker tests/stereo_calibration/test_coverage_tracker.cpp)
  target_link_libraries(test_coverage_tracker dvrk_stereo_calibration)
  add_test(NAME calibration_coverage COMMAND test_coverage_tracker)
  add_executable(stereo_rectify_test tests/stereo_calibration/stereo_rectify_test.cpp)
  target_link_libraries(stereo_rectify_test dvrk_rectify_filter)
  add_test(NAME stereo_rectify COMMAND stereo_rectify_test)
  set_tests_properties(stereo_rectify PROPERTIES TIMEOUT 30)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME stereo_workflow COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/stereo_workflow_test.py)
  set_tests_properties(stereo_workflow PROPERTIES ENVIRONMENT "DVRK_DATA_TEST_BUILD=${CMAKE_CURRENT_BINARY_DIR};PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}" TIMEOUT 30)
  add_test(NAME stereo_rectify_socket COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/stereo_calibration/stereo_rectify_socket_test.py
    $<TARGET_FILE:stereo_rectify>)
  set_tests_properties(stereo_rectify_socket PROPERTIES TIMEOUT 45)
  add_executable(latest_video_test tests/latest_video_test.cpp)
  target_link_libraries(latest_video_test dvrk_data)
  add_test(NAME latest_video COMMAND latest_video_test)
  add_test(NAME latest_video_i420 COMMAND latest_video_test --i420)
  set_tests_properties(latest_video latest_video_i420 PROPERTIES TIMEOUT 15)
  add_executable(stereo_timestamp_meta_test tests/stereo_timestamp_meta_test.cpp)
  target_include_directories(stereo_timestamp_meta_test PRIVATE include ${GST_INCLUDE_DIRS})
  target_link_libraries(stereo_timestamp_meta_test ${GST_LIBRARIES})
  add_test(NAME stereo_timestamp_meta COMMAND stereo_timestamp_meta_test)
  add_test(NAME source_timestamp_meta COMMAND stereo_timestamp_meta_test --source)
  add_test(NAME shifted_gl_timestamp_meta COMMAND stereo_timestamp_meta_test --gl-shift)
  set_tests_properties(stereo_timestamp_meta source_timestamp_meta shifted_gl_timestamp_meta PROPERTIES TIMEOUT 25)
endif()

ament_package()
