cmake_minimum_required(VERSION 3.15)
project(gpe_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(detection_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)


add_library(gpe_utils INTERFACE)
target_compile_features(gpe_utils INTERFACE cxx_std_17)
target_include_directories(gpe_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gpe_utils INTERFACE
  tf2
  geometry_msgs
)

add_library(se2_pose_estimation SHARED
  src/se2_pose_estimation.cpp
)
target_compile_features(se2_pose_estimation PUBLIC cxx_std_17)
target_include_directories(se2_pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(se2_pose_estimation PUBLIC
  rclcpp
  detection_msgs
)
# TODO: Consider changing export visibility to PUBLIC if
#       these libs are expected to be used by consuming targets
target_link_libraries(se2_pose_estimation PUBLIC
  Eigen3::Eigen
  # g2o::core # Core is already exported by other g2o targets
  g2o::types_slam2d
  g2o::solver_cholmod
  PRIVATE
  gpe_utils
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS se2_pose_estimation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  se2_pose_estimation
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  tf2
  geometry_msgs
  g2o
  Eigen3
)

ament_package()
