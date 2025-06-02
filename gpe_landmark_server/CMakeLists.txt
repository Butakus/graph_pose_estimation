cmake_minimum_required(VERSION 3.15)
project(gpe_landmark_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(gpe_msgs REQUIRED)

add_library(gpe_landmark_io SHARED
  src/landmark_io.cpp
)
target_compile_features(gpe_landmark_io PUBLIC cxx_std_20)
target_include_directories(gpe_landmark_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gpe_landmark_io PUBLIC
  yaml-cpp
  ${gpe_msgs_TARGETS}
)


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS gpe_landmark_io
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

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  yaml_cpp_vendor
  gpe_msgs
)

ament_package()
