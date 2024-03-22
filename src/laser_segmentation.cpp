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

#include "laser_segmentation/laser_segmentation.hpp"

namespace laser_segmentation
{

LaserSegmentation::LaserSegmentation(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("laser_segmentation", "", options)
{
}

CallbackReturn LaserSegmentation::on_configure(const rclcpp_lifecycle::State &)
{
  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(shared_from_this(), this->get_logger());
  params_ = param_handler_->getParams();

  // Setting for segmentation algorithm
  if (params_->segmentation_type == "jump_distance") {
    segmentation_.reset(new JumpDistanceSegmentation);
  } else if (params_->segmentation_type == "jump_distance_merge") {
    segmentation_.reset(new JumpDistanceSegmentationMerge);
  } else {
    RCLCPP_FATAL(
      this->get_logger(), "Segmentation algorithm is invalid: %s]",
      params_->segmentation_type.c_str());
    return CallbackReturn::FAILURE;
  }

  // Publishers
  segment_pub_ = this->create_publisher<slg_msgs::msg::SegmentArray>(params_->seg_topic, 1);
  segment_viz_points_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_->seg_topic + "/visualization", 10);

  // Subscribers
  auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    params_->scan_topic,
    default_qos,
    std::bind(&LaserSegmentation::scan_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Configured laser segmentation node");

  return CallbackReturn::SUCCESS;
}

CallbackReturn LaserSegmentation::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);
  RCLCPP_INFO(this->get_logger(), "Activating the node...");

  segment_pub_->on_activate();
  segment_viz_points_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LaserSegmentation::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(this->get_logger(), "Deactivating the node...");

  segment_pub_->on_deactivate();
  segment_viz_points_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LaserSegmentation::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning the node...");

  // Release the shared pointers
  segment_pub_.reset();
  segment_viz_points_pub_.reset();
  param_handler_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn LaserSegmentation::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Shutdown the node from state %s.", state.label().c_str());

  // Release the shared pointers
  segment_pub_.reset();
  segment_viz_points_pub_.reset();

  return CallbackReturn::SUCCESS;
}

void LaserSegmentation::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  // Note: Only perform laserscan segmentation if there's any subscriber
  if (segment_pub_->get_subscription_count() == 0 &&
    segment_viz_points_pub_->get_subscription_count() == 0)
  {
    return;
  }
  RCLCPP_INFO_ONCE(
    this->get_logger(), "Subscribed to laser scan topic: [%s]", params_->scan_topic.c_str());

  // Read the laser scan
  std::vector<slg::Point2D> point_list;
  double phi = scan_msg->angle_min;
  double angle_resolution = scan_msg->angle_increment;
  for (const auto r : scan_msg->ranges) {
    if (r >= scan_msg->range_min && r <= scan_msg->range_max) {
      point_list.push_back(slg::Point2D::from_polar_coords(r, phi));
    } else {
      point_list.push_back(slg::Point2D::quiet_NaN());
    }
    phi += angle_resolution;
  }

  // Segment the points
  std::vector<slg::Segment2D> segment_list;
  segmentation_->initialize_segmentation(
    params_->distance_threshold, angle_resolution, params_->noise_reduction,
    params_->method_threshold);
  segmentation_->perform_segmentation(point_list, segment_list);

  // Filter segments
  auto segment_filtered_list = filter_segments(segment_list);

  // Identification of segments and set angular distance
  for (std::vector<slg::Segment2D>::size_type s = 0; s < segment_filtered_list.size(); s++) {
    double angle = std::min(
      scan_msg->angle_max - segment_filtered_list[s].max_angle(),
      segment_filtered_list[s].min_angle() - scan_msg->angle_min);
    segment_filtered_list[s].set_id(s);
    segment_filtered_list[s].set_angular_distance_to_closest_boundary(angle);
  }

  // Publish the segment array
  slg_msgs::msg::SegmentArray segment_array_msg;
  segment_array_msg.header = scan_msg->header;
  for (const auto & segment : segment_filtered_list) {
    segment_array_msg.segments.push_back(segment);
  }
  segment_pub_->publish(segment_array_msg);

  // Publish visualization markers
  segment_viz_points_pub_->publish(
    create_segment_viz_points(scan_msg->header, segment_filtered_list));
}

std::vector<slg::Segment2D> LaserSegmentation::filter_segments(
  const std::vector<slg::Segment2D> & segments)
{
  std::vector<slg::Segment2D> filtered_segments;
  filtered_segments.reserve(segments.size());

  double squared_min_segment_width = params_->min_segment_width * params_->min_segment_width;
  double squared_max_segment_width = params_->max_segment_width * params_->max_segment_width;

  for (const auto & segment : segments) {
    // By number of points
    if (segment.size() < params_->min_points_segment ||
      segment.size() > params_->max_points_segment)
    {
      continue;
    }

    // By distance to sensor
    if (segment.centroid().length() < params_->min_avg_distance_from_sensor ||
      segment.centroid().length() > params_->max_avg_distance_from_sensor)
    {
      continue;
    }

    // By width
    if (segment.width_squared() < squared_min_segment_width ||
      segment.width_squared() > squared_max_segment_width)
    {
      continue;
    }

    filtered_segments.push_back(segment);
  }
  return filtered_segments;
}

visualization_msgs::msg::MarkerArray LaserSegmentation::create_segment_viz_points(
  std_msgs::msg::Header header,
  std::vector<slg::Segment2D> segment_list)
{
  // Create the visualization message
  visualization_msgs::msg::MarkerArray viz_array;

  // Create a deletion marker to clear the previous points
  visualization_msgs::msg::Marker deletion_marker;
  deletion_marker.header = header;
  deletion_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  viz_array.markers.push_back(deletion_marker);

  // Create a marker point
  visualization_msgs::msg::Marker viz_points;
  viz_points.header = header;
  viz_points.lifetime = rclcpp::Duration(0, 10);
  viz_points.ns = "segments";
  viz_points.type = visualization_msgs::msg::Marker::POINTS;
  viz_points.action = visualization_msgs::msg::Marker::ADD;
  viz_points.scale.x = viz_points.scale.y = 0.02;

  // Create a marker centroid
  visualization_msgs::msg::Marker viz_centroids;
  viz_centroids.header = header;
  viz_centroids.lifetime = rclcpp::Duration(0, 10);
  viz_centroids.ns = "centroids";
  viz_centroids.type = visualization_msgs::msg::Marker::CUBE;
  viz_centroids.action = visualization_msgs::msg::Marker::ADD;
  viz_centroids.scale.x = viz_centroids.scale.y = viz_centroids.scale.z = 0.05;

  // Create a marker id text
  visualization_msgs::msg::Marker viz_text;
  viz_text.header = header;
  viz_text.lifetime = rclcpp::Duration(0, 10);
  viz_text.ns = "id";
  viz_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  viz_text.action = visualization_msgs::msg::Marker::ADD;
  viz_text.scale.z = 0.25;
  viz_text.color.r = 1.0;
  viz_text.color.g = 1.0;
  viz_text.color.b = 1.0;
  viz_text.color.a = 1.0;

  // Show the segments and the id
  for (std::vector<slg::Segment2D>::size_type i = 0; i < segment_list.size(); i++) {
    viz_points.id = i;
    viz_text.id = i;
    viz_centroids.id = i;

    // Change the color of the segment
    viz_points.color = get_palette_color(i);
    viz_centroids.color = get_palette_color(i);

    // Iterate over the points of the segment
    slg::Segment2D current_segment = segment_list[i];
    for (const auto & point : current_segment.get_points()) {
      viz_points.points.push_back(point);
    }

    // Get position of the text
    viz_text.text = std::to_string(current_segment.get_id());
    viz_text.pose.position = current_segment.centroid();
    viz_text.pose.position.z = 0.10;

    // Place centroid under text
    viz_centroids.pose.position = current_segment.centroid();
    viz_centroids.pose.position.z = 0.0;

    // Push to arrays
    viz_array.markers.push_back(viz_points);
    viz_array.markers.push_back(viz_centroids);
    viz_array.markers.push_back(viz_text);

    // Clear markers
    viz_points.points.clear();
  }

  return viz_array;
}

std_msgs::msg::ColorRGBA LaserSegmentation::get_parula_color(unsigned int index, unsigned int max)
{
  std_msgs::msg::ColorRGBA color;
  int div = round(256 / max);
  color.r = parula[index * div][0];
  color.g = parula[index * div][1];
  color.b = parula[index * div][2];
  color.a = 1.0;
  return color;
}

std_msgs::msg::ColorRGBA LaserSegmentation::get_palette_color(unsigned int index)
{
  std_msgs::msg::ColorRGBA color;
  switch (index % 8) {
    case 0: color.r = 255; color.g = 051; color.b = 051; break;
    case 2: color.r = 255; color.g = 153; color.b = 051; break;
    case 4: color.r = 255; color.g = 255; color.b = 051; break;
    case 6: color.r = 153; color.g = 051; color.b = 051; break;
    case 1: color.r = 051; color.g = 255; color.b = 051; break;
    case 3: color.r = 051; color.g = 255; color.b = 153; break;
    case 5: color.r = 051; color.g = 153; color.b = 255; break;
    case 7: color.r = 255; color.g = 051; color.b = 255; break;
  }

  color.r /= 255.0; color.g /= 255.0; color.b /= 255.0; color.a = 1.0;
  return color;
}

}  // namespace laser_segmentation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(laser_segmentation::LaserSegmentation)
