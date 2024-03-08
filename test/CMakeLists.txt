# Test for segmentations algorithm
ament_add_gtest(test_segmentations
  test_segmentations.cpp
)
ament_target_dependencies(test_segmentations
  ${dependencies}
)
target_link_libraries(test_segmentations
  ${library_name}
)

# Test for laser_segmentation
ament_add_gtest(test_laser_segmentation
  test_laser_segmentation.cpp
)
ament_target_dependencies(test_laser_segmentation
  ${dependencies}
)
target_link_libraries(test_laser_segmentation
  ${library_name}
)

# Test for laser_segmentation integration
ament_add_gtest(test_integration
  test_integration.cpp
)
ament_target_dependencies(test_integration
  ${dependencies}
)
target_link_libraries(test_integration
  ${library_name}
)