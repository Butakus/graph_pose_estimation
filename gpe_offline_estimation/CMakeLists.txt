cmake_minimum_required(VERSION 3.15)
project(gpe_offline_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(gpe_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gpe_msgs REQUIRED)
find_package(gpe_landmark_server REQUIRED)


add_library(file_io INTERFACE)
target_compile_features(file_io INTERFACE cxx_std_20)
target_sources(file_io PUBLIC
  FILE_SET HEADERS
  BASE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  FILES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/file_io.hpp>
)
target_link_libraries(file_io INTERFACE
  Eigen3::Eigen
  ${gpe_msgs_TARGETS}
)

add_executable(se2_offline_estimation_node src/se2_offline_estimation_node.cpp)
target_include_directories(se2_offline_estimation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(se2_offline_estimation_node PUBLIC c_std_99 cxx_std_20)
target_link_libraries(se2_offline_estimation_node PRIVATE
  file_io
  Eigen3::Eigen
  rcutils::rcutils
  gpe_core::gpe_utils
  gpe_core::gpe_se2_pose_estimation
  gpe_landmark_server::gpe_landmark_io
  ${gpe_msgs_TARGETS}
)

install(TARGETS se2_offline_estimation_node
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
