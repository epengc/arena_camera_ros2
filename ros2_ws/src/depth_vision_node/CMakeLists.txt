cmake_minimum_required(VERSION 3.8)
project(depth_vision_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(rmw REQUIRED)
#---------------------------------------------------------------------------------------
#
# ROS2 RUN DEPTH_VISION_NODE <> EXECUTABLE FILE
#
#---------------------------------------------------------------------------------------

set(target_name_disparity start_disparity)
#set(OpenCV_INCLUDE_DIRS "/usr/local/include/opnecv4/opencv2/")
#set(OpenCV_LIBS "/usr/local/lib/")
# CPP DEPS
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(${target_name_disparity} src/disparityMap.cpp)
ament_target_dependencies(${target_name_disparity} rclcpp rclcpp_components std_msgs std_srvs sensor_msgs cv_bridge message_filters image_transport stereo_msgs rmw)
target_link_libraries(${target_name_disparity} ${OpenCV_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
