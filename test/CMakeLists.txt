# Test for segmentations algorithm
ament_add_gtest(test_segmentations test_segmentations.cpp)
target_link_libraries(test_segmentations ${library_name} slg_msgs::slg_core)

# Test for laser_segmentation
ament_add_gtest(test_laser_segmentation test_laser_segmentation.cpp)
target_link_libraries(test_laser_segmentation ${library_name} rclcpp::rclcpp)

# Test for laser_segmentation integration
ament_add_gtest(test_integration test_integration.cpp)
target_link_libraries(test_integration
  ${library_name}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)