cmake_minimum_required(VERSION 3.15)
project(gpe_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(detection_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

add_library(se2_pose_estimation SHARED
  src/se2_pose_estimation.cpp
)
target_compile_features(se2_pose_estimation PUBLIC cxx_std_17)  # Require C++17
target_include_directories(se2_pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(se2_pose_estimation
  rclcpp
  detection_msgs
)
target_link_libraries(se2_pose_estimation
  Eigen3::Eigen
  g2o::core
  g2o::types_sba
  g2o::solver_dense
  g2o::solver_cholmod
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(se2_pose_estimation PRIVATE "GPE_CORE_BUILDING_LIBRARY")

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

ament_package()
