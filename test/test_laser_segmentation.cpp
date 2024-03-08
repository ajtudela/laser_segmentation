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

class laserSegmentationFixture : public laserSegmentation
{
public:
  laserSegmentationFixture()
  : laserSegmentation() {}

  visualization_msgs::msg::MarkerArray create_segment_viz_points(
    std_msgs::msg::Header header, std::vector<slg::Segment2D> segment_list)
  {
    return laserSegmentation::create_segment_viz_points(header, segment_list);
  }

  std_msgs::msg::ColorRGBA get_parula_color(unsigned int index, unsigned int max)
  {
    return laserSegmentation::get_parula_color(index, max);
  }

  std_msgs::msg::ColorRGBA get_palette_color(unsigned int index)
  {
    return laserSegmentation::get_palette_color(index);
  }
};

TEST(LaserSegmentationTest, configure) {
  // Create the node
  auto node = std::make_shared<laserSegmentationFixture>();
  // Set the default segmentation type "jump_distance"
  nav2_util::declare_parameter_if_not_declared(
    node, "segmentation_type", rclcpp::ParameterValue("jump_distance"));
  node->configure();
  node->activate();
  node->deactivate();
  node->cleanup();

  // Set the segmentation type to "jump_distance_merge"
  node->set_parameter(rclcpp::Parameter("segmentation_type", "jump_distance_merge"));
  node->configure();
  node->activate();
  node->deactivate();
  node->cleanup();

  // Set the segmentation type to another value so it fails
  node->set_parameter(rclcpp::Parameter("segmentation_type", "unknown"));
  node->configure();
  node->activate();
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(LaserSegmentationTest, dynamicParameters) {
  // Create and configure the node
  auto node = std::make_shared<laserSegmentationFixture>();
  node->configure();
  node->activate();

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set parameters
  auto results = params->set_parameters_atomically(
  {
    rclcpp::Parameter("min_points_segment", 1),
    rclcpp::Parameter("max_points_segment", 100),
    rclcpp::Parameter("min_avg_distance_from_sensor", 0.1),
    rclcpp::Parameter("max_avg_distance_from_sensor", 10.0),
    rclcpp::Parameter("min_segment_width", 0.1),
    rclcpp::Parameter("max_segment_width", 10.0),
    rclcpp::Parameter("distance_threshold", 0.1),
    rclcpp::Parameter("noise_reduction", 0.1),
    rclcpp::Parameter("method_threshold", "fixed_test")
  });

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("min_points_segment").as_int(), 1);
  EXPECT_EQ(node->get_parameter("max_points_segment").as_int(), 100);
  EXPECT_EQ(node->get_parameter("min_avg_distance_from_sensor").as_double(), 0.1);
  EXPECT_EQ(node->get_parameter("max_avg_distance_from_sensor").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("min_segment_width").as_double(), 0.1);
  EXPECT_EQ(node->get_parameter("max_segment_width").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("distance_threshold").as_double(), 0.1);
  EXPECT_EQ(node->get_parameter("noise_reduction").as_double(), 0.1);
  EXPECT_EQ(node->get_parameter("method_threshold").as_string(), "fixed_test");
}

TEST(LaserSegmentationTest, colorParula) {
  laserSegmentationFixture node;
  std_msgs::msg::ColorRGBA color = node.get_parula_color(0, 10);
  EXPECT_DOUBLE_EQ(color.r, 0.20810000598430634);
  EXPECT_DOUBLE_EQ(color.g, 0.16629999876022339);
  EXPECT_DOUBLE_EQ(color.b, 0.52920001745223999);
  EXPECT_DOUBLE_EQ(color.a, 1.0);
}

using LaserSegmentationColorParam = std::tuple<unsigned int, std::tuple<double, double, double,
    double>>;

class LaserSegmentationColorTest
  : public ::testing::TestWithParam<LaserSegmentationColorParam>
{};

TEST_P(LaserSegmentationColorTest, color)
{
  auto node = std::make_shared<laserSegmentationFixture>();
  auto index = std::get<0>(GetParam());
  auto expected_color = std::get<1>(GetParam());
  auto color = node->get_palette_color(index);
  EXPECT_NEAR(color.r, std::get<0>(expected_color), 1e-6);
  EXPECT_NEAR(color.g, std::get<1>(expected_color), 1e-6);
  EXPECT_NEAR(color.b, std::get<2>(expected_color), 1e-6);
  EXPECT_NEAR(color.a, std::get<3>(expected_color), 1e-6);
}

INSTANTIATE_TEST_SUITE_P(
  LaserSegmentationColorTestSuite,
  LaserSegmentationColorTest,
  ::testing::Values(
    LaserSegmentationColorParam{0, {1.0, 0.16078431, 0.16078431, 1.0}},
    LaserSegmentationColorParam{1, {0.16078431, 1.0, 0.16078431, 1.0}},
    LaserSegmentationColorParam{2, {1.0, 0.60, 0.16078431, 1.0}},
    LaserSegmentationColorParam{3, {0.16078431, 1.0, 0.60, 1.0}},
    LaserSegmentationColorParam{4, {1.0, 1.0, 0.16078431, 1.0}},
    LaserSegmentationColorParam{5, {0.16078431, 0.60, 1.0, 1.0}},
    LaserSegmentationColorParam{6, {0.60, 0.16078431, 0.16078431, 1.0}},
    LaserSegmentationColorParam{7, {1.0, 0.16078431, 1.0, 1.0}}
));

TEST(LaserSegmentationTest, createVizPoints) {
  // Create the node
  auto node = std::make_shared<laserSegmentationFixture>();
  // Set a segment list with one segment of 3 points
  std_msgs::msg::Header header;
  header.frame_id = "laser_frame";
  std::vector<slg::Segment2D> segment_list;
  slg::Segment2D segment;
  segment.add_point(slg::Point2D(0.0, 0.0, slg::BACKGROUND));
  segment.add_point(slg::Point2D(1.0, 1.0, slg::BACKGROUND));
  segment.add_point(slg::Point2D(2.0, 2.0, slg::BACKGROUND));
  segment_list.push_back(segment);
  // Create the visualization message
  auto marker_array = node->create_segment_viz_points(header, segment_list);
  // Check the message:
  // The message should have num_segm + 3 markers (delete, centroid and text)
  EXPECT_EQ(marker_array.markers.size(), segment_list.size() + 3);
  // Check the first marker: deleteall
  EXPECT_EQ(marker_array.markers[0].header.frame_id, "laser_frame");
  EXPECT_EQ(marker_array.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
  // Check the second marker: segment
  EXPECT_EQ(marker_array.markers[1].header.frame_id, "laser_frame");
  EXPECT_EQ(marker_array.markers[1].type, visualization_msgs::msg::Marker::POINTS);
  EXPECT_EQ(marker_array.markers[1].action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(marker_array.markers[1].points.size(), 3);
  EXPECT_EQ(marker_array.markers[1].points[0].x, 0.0);
  EXPECT_EQ(marker_array.markers[1].points[0].y, 0.0);
  EXPECT_EQ(marker_array.markers[1].points[1].x, 1.0);
  EXPECT_EQ(marker_array.markers[1].points[1].y, 1.0);
  EXPECT_EQ(marker_array.markers[1].points[2].x, 2.0);
  EXPECT_EQ(marker_array.markers[1].points[2].y, 2.0);
  // Check the third marker: centroid
  EXPECT_EQ(marker_array.markers[2].header.frame_id, "laser_frame");
  EXPECT_EQ(marker_array.markers[2].type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_EQ(marker_array.markers[2].action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(marker_array.markers[2].pose.position.x, 1.0);
  EXPECT_EQ(marker_array.markers[2].pose.position.y, 1.0);
  // Check the fourth marker: text
  EXPECT_EQ(marker_array.markers[3].header.frame_id, "laser_frame");
  EXPECT_EQ(marker_array.markers[3].type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(marker_array.markers[3].action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(marker_array.markers[3].text, "0");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
