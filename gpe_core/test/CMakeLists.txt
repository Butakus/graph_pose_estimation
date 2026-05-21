ament_add_gtest(test_utils test_utils.cpp)
target_include_directories(test_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_utils
  gpe_utils
)

ament_add_gtest(test_hungarian test_hungarian.cpp)
target_include_directories(test_hungarian PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_hungarian
  gpe_hungarian
)

ament_add_gtest(test_kabsch_alignment test_kabsch_alignment.cpp)
target_include_directories(test_kabsch_alignment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_kabsch_alignment
  gpe_kabsch
  gpe_se2_pose_estimation
  gpe_utils
)

ament_add_gtest(test_se2_pose_estimation_xy test_se2_pose_estimation_xy.cpp)
target_include_directories(test_se2_pose_estimation_xy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_se2_pose_estimation_xy
  gpe_se2_pose_estimation
  gpe_utils
)

ament_add_gtest(test_se2_pose_estimation_se2 test_se2_pose_estimation_se2.cpp)
target_include_directories(test_se2_pose_estimation_se2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_se2_pose_estimation_se2
  gpe_se2_pose_estimation
  gpe_utils
)

ament_add_gtest(test_se2_pose_estimation_mixed_types test_se2_pose_estimation_mixed_types.cpp)
target_include_directories(test_se2_pose_estimation_mixed_types PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_se2_pose_estimation_mixed_types
  gpe_se2_pose_estimation
  gpe_utils
)
