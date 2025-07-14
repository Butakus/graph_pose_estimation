cmake_minimum_required(VERSION 3.15)
project(gpe_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gpe_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

add_library(gpe_utils INTERFACE)
target_compile_features(gpe_utils INTERFACE cxx_std_23)
target_sources(gpe_utils PUBLIC
  FILE_SET HEADERS
  BASE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  FILES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/utils.hpp>
)
target_link_libraries(gpe_utils INTERFACE
  Eigen3::Eigen
  tf2::tf2
  ${geometry_msgs_TARGETS}
)

add_library(gpe_types INTERFACE)
target_compile_features(gpe_types INTERFACE cxx_std_23)
target_sources(gpe_types PUBLIC
  FILE_SET HEADERS
  BASE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  FILES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/types.hpp>
)
target_link_libraries(gpe_types INTERFACE
  Eigen3::Eigen
  g2o::types_slam2d
  ${gpe_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

add_library(gpe_hungarian SHARED
  src/hungarian.cpp
)
target_compile_features(gpe_hungarian PUBLIC cxx_std_23)
target_include_directories(gpe_hungarian PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(gpe_se2_pose_estimation SHARED
  src/se2_pose_estimation.cpp
)
target_compile_features(gpe_se2_pose_estimation PUBLIC cxx_std_23)
target_include_directories(gpe_se2_pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gpe_se2_pose_estimation
  PUBLIC
    gpe_types
  PRIVATE
    g2o::solver_cholmod
    gpe_hungarian
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS gpe_se2_pose_estimation gpe_hungarian gpe_utils gpe_types
  EXPORT export_${PROJECT_NAME}
  FILE_SET HEADERS
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

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rosidl_default_runtime
  tf2
  geometry_msgs
  gpe_msgs
  vision_msgs
  g2o
  Eigen3
)

ament_package()
