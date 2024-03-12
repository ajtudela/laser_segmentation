// Copyright (c) 2017 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "laser_segmentation/laser_segmentation.hpp"

TEST(LaserSegmentationTest, integration) {
  rclcpp::init(0, nullptr);
  // Create the scan publisher node
  auto pub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("scan_publisher");
  pub_node->configure();
  // Create a publisher for the scan
  auto scan_pub = pub_node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
  ASSERT_EQ(scan_pub->get_subscription_count(), 0);
  EXPECT_FALSE(scan_pub->is_activated());
  // Activate the publisher
  pub_node->activate();
  EXPECT_TRUE(scan_pub->is_activated());
  scan_pub->on_activate();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

  // Create and configure the laser_segmentation node
  auto seg_node = std::make_shared<laserSegmentation>();
  seg_node->configure();
  seg_node->activate();

  // Create a scan message with three points
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  // Publish the message
  scan_pub->publish(scan);

  // Spin the laser_segmentation node
  // Shouldn get any susbcriptions count
  rclcpp::spin_some(seg_node->get_node_base_interface());

  // Create the segments subscriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segment_subscriber");
  sub_node->configure();
  sub_node->activate();
  // Create a subscriber for the segments
  auto seg_sub = sub_node->create_subscription<slg_msgs::msg::SegmentArray>(
    "segments", 1,
    [&](const slg_msgs::msg::SegmentArray msg) {
      EXPECT_EQ(msg.segments.size(), 1);
      RCLCPP_INFO(sub_node->get_logger(), "Segment received: %ld", msg.segments.size());
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Spin the laser_segmentation node
  rclcpp::spin_some(seg_node->get_node_base_interface());

  // Check the results: now, the scan should have a subscription
  // and the segment should have a publisher
  EXPECT_EQ(scan_pub->get_subscription_count(), 1);
  EXPECT_EQ(seg_sub->get_publisher_count(), 1);

  // Deactivate the nodes
  seg_node->deactivate();
  pub_node->deactivate();
  sub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  sub_thread.join();
  pub_thread.join();
}
