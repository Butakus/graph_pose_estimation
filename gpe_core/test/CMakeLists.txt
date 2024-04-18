ament_add_gtest(test_utils test_utils.cpp)
target_include_directories(se2_pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_utils
    gpe_utils
)

ament_add_gtest(test_se2_pose_estimation test_se2_pose_estimation.cpp)
target_include_directories(test_se2_pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_se2_pose_estimation
    se2_pose_estimation
    gpe_utils
)
