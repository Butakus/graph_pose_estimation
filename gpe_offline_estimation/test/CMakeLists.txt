
add_definitions(-DTEST_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}\")

ament_add_gtest(test_file_io test_file_io.cpp)
target_include_directories(test_file_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(test_file_io PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(test_file_io
  gpe_msgs
  gpe_landmark_server
)
target_link_libraries(test_file_io
  file_io
)