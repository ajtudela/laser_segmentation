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

#ifndef LASER_SEGMENTATION__LASERSEGMENTATION_HPP_
#define LASER_SEGMENTATION__LASERSEGMENTATION_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// SIMPLE LASER GEOMETRY
#include "slg_msgs/segment2D.hpp"
#include "slg_msgs/msg/segment_array.hpp"

// LASER SEGMENTATION
#include "laser_segmentation/parula.hpp"
#include "laser_segmentation/segmentation/segmentation.hpp"
#include "laser_segmentation/segmentation/segmentationJumpDistance.hpp"
#include "laser_segmentation/segmentation/segmentationJumpDistanceMerge.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class laserSegmentation : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Construct a new laser Segmentation object
   *
   * @param options Node options
   */
  explicit laserSegmentation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the laser Segmentation object
   *
   */
  ~laserSegmentation() = default;

  /**
   * @brief Configure the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Callback executed when a new scan is received
   *
   * @param scan The received scan
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief Show the visualization of the segments
   *
   * @param header Header of the message
   * @param segment_list List of segments
   */
  void show_visualization(std_msgs::msg::Header header, std::vector<slg::Segment2D> segment_list);

  /**
   * @brief Get the parula color object
   *
   * @param index Index of the color
   * @param max Maximum index
   * @return std_msgs::msg::ColorRGBA
   */
  std_msgs::msg::ColorRGBA get_parula_color(unsigned int index, unsigned int max);

  /**
   * @brief Get the palette color object
   *
   * @param index Index of the color
   * @return std_msgs::msg::ColorRGBA
   */
  std_msgs::msg::ColorRGBA get_palette_color(unsigned int index);

  rclcpp_lifecycle::LifecyclePublisher<slg_msgs::msg::SegmentArray>::SharedPtr segment_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    segment_viz_points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  std::string scan_topic_, seg_topic_, segmentation_type_, method_thres_;
  int min_points_, max_points_;
  double min_avg_distance_from_sensor_, max_avg_distance_from_sensor_, min_segment_width_,
    max_segment_width_, distance_thres_, noise_reduction_;
  bool setup_, restore_;
  std::shared_ptr<Segmentation> segmentation_;
};

#endif  // LASER_SEGMENTATION__LASERSEGMENTATION_HPP_
