// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#ifndef LASER_SEGMENTATION__PARAMETER_HANDLER_HPP_
#define LASER_SEGMENTATION__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

struct Parameters
{
  int min_points_segment;
  int max_points_segment;
  double min_avg_distance_from_sensor;
  double max_avg_distance_from_sensor;
  double min_segment_width;
  double max_segment_width;
  double distance_threshold;
  double noise_reduction;
  std::string scan_topic;
  std::string seg_topic;
  std::string segmentation_type;
  std::string method_threshold;
};

/**
 * @class ParameterHandler
 * @brief Handles parameters and dynamic parameters for laser_segmentation
 */
class ParameterHandler
{
public:
  /**
   * @brief Constructor for ParameterHandler
   */
  ParameterHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, const rclcpp::Logger & logger);

  /**
   * @brief Destructor for ParameterHandler
   */
  ~ParameterHandler() = default;

  std::mutex & getMutex() {return mutex_;}

  Parameters * getParams() {return &params_;}

protected:
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  Parameters params_;
  rclcpp::Logger logger_ {rclcpp::get_logger("laser_segmentation")};
};

#endif  // LASER_SEGMENTATION__PARAMETER_HANDLER_HPP_
