ament_add_gtest(test_io test_io.cpp)
target_include_directories(test_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_io
    gpe_landmark_io
)
