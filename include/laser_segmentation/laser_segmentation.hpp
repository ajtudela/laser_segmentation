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

#ifndef LASER_SEGMENTATION__LASER_SEGMENTATION_HPP_
#define LASER_SEGMENTATION__LASER_SEGMENTATION_HPP_

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
#include "laser_segmentation/parameter_handler.hpp"
#include "laser_segmentation/segmentation/segmentation.hpp"
#include "laser_segmentation/segmentation/jump_distance.hpp"
#include "laser_segmentation/segmentation/jump_distance_merge.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_segmentation
{

/**
 * @brief laser_segmentation::LaserSegmentation class is a ROS2 node that subscribes to a laser scan
 * topic and publishes the segments found in the scan.
 *
 */
class LaserSegmentation : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Construct a new laser Segmentation object
   *
   * @param options Node options
   */
  explicit LaserSegmentation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the laser Segmentation object
   *
   */
  ~LaserSegmentation() = default;

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

protected:
  /**
   * @brief Callback executed when a new scan is received
   *
   * @param scan The received scan
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief Filter the segments using the parameters
   *
   * @param segments List of segments
   * @return std::vector<slg::Segment2D> Filtered segments
   */
  std::vector<slg::Segment2D> filter_segments(const std::vector<slg::Segment2D> & segments);

  /**
   * @brief Create the segment array message
   *
   * @param header Header of the message
   * @param segment_list List of segments
   * @return slg_msgs::msg::SegmentArray The segment array message
   */
  visualization_msgs::msg::MarkerArray create_segment_viz_points(
    std_msgs::msg::Header header,
    std::vector<slg::Segment2D> segment_list);

  /**
   * @brief Get the parula color object
   *
   * @param index Index of the color
   * @param max Maximum index
   * @return std_msgs::msg::ColorRGBA The color
   */
  std_msgs::msg::ColorRGBA get_parula_color(unsigned int index, unsigned int max);

  /**
   * @brief Get the palette color object
   *
   * @param index Index of the color
   * @return std_msgs::msg::ColorRGBA The color
   */
  std_msgs::msg::ColorRGBA get_palette_color(unsigned int index);

  rclcpp_lifecycle::LifecyclePublisher<slg_msgs::msg::SegmentArray>::SharedPtr segment_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    segment_viz_points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::unique_ptr<ParameterHandler> param_handler_;

  Parameters * params_;
  std::shared_ptr<Segmentation> segmentation_;
};

}  // namespace laser_segmentation

#endif  // LASER_SEGMENTATION__LASER_SEGMENTATION_HPP_
