cmake_minimum_required(VERSION 3.15)
project(stereo_proc_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
#
# ALL PKGS SETTIGNS
#
# -----------------------------------------------------------------------------

# ------------------
# CUSTOM DEPS PATHS
# ------------------

# arenasdk find file dir
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# ----------
# FIND DEPS
# ----------

# finds dependencies
# - same as the ones mentioned in package.xml under
#   the tags <buildtool_depend>, <build_depend>, <exec_depend>
# - for pakages used in #includes
# - even if a dependency is used in one package, it has to be found
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arena_skd REQUIRED)

# -----------------------------------------------------------------------------
#
# ROS2 RUN STEREO_PROC_NODE <>
#
# -----------------------------------------------------------------------------

set(target_name stereo)

# CPP DEPS
add_executable(${target_name}
    src/main.cpp
)

# ROS DEP
# - this way, it will find our header libs automatically
# - for Arenasdk the cmake way is used no ament
ament_target_dependencies(${target_name}  rclcpp 
                                          std_msgs 
                                          std_srvs
                                          sensor_msgs
                                          arena_sdk
                                          )
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
