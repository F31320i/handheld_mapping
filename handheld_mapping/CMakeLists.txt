cmake_minimum_required(VERSION 3.8)
project(handheld_mapping CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# The only compiled piece: the CustomMsg -> PointCloud2 conversion LIVO2 needs.
# Python cannot keep up with 20k points at 10 Hz plus 200 Hz IMU (see the
# comment at the top of the source).
add_executable(livox_livo2_adapter src/livox_livo2_adapter.cpp)
ament_target_dependencies(livox_livo2_adapter rclcpp sensor_msgs livox_ros_driver2)

# Replaces usb_cam for the mapping runs: usb_cam 0.8.1 delivers frames 0.1-1.0 s
# after capture on this machine (see the header of the source).
add_executable(v4l2_capture src/v4l2_capture.cpp)
ament_target_dependencies(v4l2_capture rclcpp sensor_msgs)

install(TARGETS livox_livo2_adapter v4l2_capture DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
        DESTINATION share/${PROJECT_NAME})

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        scripts/check_streams.py
        scripts/watch_imu.py
        scripts/project_points.py
        scripts/set_camera_controls.sh
        scripts/set_mid360_ips.py
        scripts/camera_calib_to_livo2.py
        scripts/save_frame.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
