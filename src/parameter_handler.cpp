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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "laser_segmentation/parameter_handler.hpp"

namespace laser_segmentation
{

using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, const rclcpp::Logger & logger)
{
  logger_ = logger;

  declare_parameter_if_not_declared(
    node, "min_points_segment", rclcpp::ParameterValue(3),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Minimium number of points per segment")
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(100)
        .set__step(1)}
  ));

  declare_parameter_if_not_declared(
    node, "max_points_segment", rclcpp::ParameterValue(200),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum number of points per segment")
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(5)
        .set__to_value(500)
        .set__step(1)}
  ));

  declare_parameter_if_not_declared(
    node, "min_avg_distance_from_sensor", rclcpp::ParameterValue(0.0),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Minimium average distance from sensor")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(50.0)
        .set__step(0.01)}
  ));

  declare_parameter_if_not_declared(
    node, "max_avg_distance_from_sensor", rclcpp::ParameterValue(20.0),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum average distance from sensor")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(50.0)
        .set__step(0.01)}
  ));

  declare_parameter_if_not_declared(
    node, "min_segment_width", rclcpp::ParameterValue(0.20),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Minimium width of the segment.")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(5.0)
        .set__step(0.01)}
  ));

  declare_parameter_if_not_declared(
    node, "max_segment_width", rclcpp::ParameterValue(10.0),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum width of the segment.")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(100.0)
        .set__step(0.1)}
  ));

  declare_parameter_if_not_declared(
    node, "distance_threshold", rclcpp::ParameterValue(0.30),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "The jump distance above which a new segment is created "
      "or noise reduction if method is selected.")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(2.0)
        .set__step(0.01)}
  ));

  declare_parameter_if_not_declared(
    node, "noise_reduction", rclcpp::ParameterValue(0.30),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Parameter for noise reduction in 'Santos' and 'Dietmayer' algorithms.")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(2.0)
        .set__step(0.1)}
  ));

  declare_parameter_if_not_declared(
    node, "method_threshold", rclcpp::ParameterValue(""),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "Method to calculate a dynamic jump distance threshold in jump_distance algorithm"));

  declare_parameter_if_not_declared(
    node, "scan_topic", rclcpp::ParameterValue("scan"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Laser topic to read"));

  declare_parameter_if_not_declared(
    node, "segments_topic", rclcpp::ParameterValue("segments"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("published Segements topic"));

  declare_parameter_if_not_declared(
    node, "segmentation_type", rclcpp::ParameterValue("jump_distance"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Type of segmentation"));

  node->get_parameter("min_points_segment", params_.min_points_segment);
  RCLCPP_INFO(
    logger_, "The parameter min_points_segment is set to: [%d]", params_.min_points_segment);

  node->get_parameter("max_points_segment", params_.max_points_segment);
  RCLCPP_INFO(
    logger_, "The parameter max_points_segment is set to: [%d]", params_.max_points_segment);

  node->get_parameter("min_avg_distance_from_sensor", params_.min_avg_distance_from_sensor);
  RCLCPP_INFO(
    logger_, "The parameter min_avg_distance_from_sensor is set to: [%f]",
    params_.min_avg_distance_from_sensor);

  node->get_parameter("max_avg_distance_from_sensor", params_.max_avg_distance_from_sensor);
  RCLCPP_INFO(
    logger_, "The parameter max_avg_distance_from_sensor is set to: [%f]",
    params_.max_avg_distance_from_sensor);

  node->get_parameter("min_segment_width", params_.min_segment_width);
  RCLCPP_INFO(
    logger_, "The parameter min_segment_width is set to: [%f]", params_.min_segment_width);

  node->get_parameter("max_segment_width", params_.max_segment_width);
  RCLCPP_INFO(
    logger_, "The parameter max_segment_width is set to: [%f]", params_.max_segment_width);

  node->get_parameter("distance_threshold", params_.distance_threshold);
  RCLCPP_INFO(
    logger_, "The parameter distance_threshold is set to: [%f]", params_.distance_threshold);

  node->get_parameter("noise_reduction", params_.noise_reduction);
  RCLCPP_INFO(
    logger_, "The parameter noise_reduction is set to: [%f]", params_.noise_reduction);

  node->get_parameter("method_threshold", params_.method_threshold);
  RCLCPP_INFO(
    logger_, "The parameter method_threshold is set to: [%s]", params_.method_threshold.c_str());
  warn_if_unknown_threshold_method(params_.method_threshold);

  node->get_parameter("scan_topic", params_.scan_topic);
  RCLCPP_INFO(logger_, "The parameter scan_topic is set to: [%s]", params_.scan_topic.c_str());

  node->get_parameter("segments_topic", params_.seg_topic);
  RCLCPP_INFO(
    logger_, "The parameter segment_topic is set to: [%s]", params_.seg_topic.c_str());

  node->get_parameter("segmentation_type", params_.segmentation_type);
  RCLCPP_INFO(
    logger_, "The parameter segmentation_type is set to: [%s]", params_.segmentation_type.c_str());

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ParameterHandler::dynamic_parameters_callback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
ParameterHandler::dynamic_parameters_callback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // Work on a candidate copy so that the min/max pairs can be validated against
  // the parameters that are not part of this update, and nothing is applied
  // unless the whole batch is valid.
  Parameters candidate = params_;

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (name == "min_points_segment") {
        candidate.min_points_segment = parameter.as_int();
      } else if (name == "max_points_segment") {
        candidate.max_points_segment = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "min_avg_distance_from_sensor") {
        candidate.min_avg_distance_from_sensor = parameter.as_double();
      } else if (name == "max_avg_distance_from_sensor") {
        candidate.max_avg_distance_from_sensor = parameter.as_double();
      } else if (name == "min_segment_width") {
        candidate.min_segment_width = parameter.as_double();
      } else if (name == "max_segment_width") {
        candidate.max_segment_width = parameter.as_double();
      } else if (name == "distance_threshold") {
        candidate.distance_threshold = parameter.as_double();
      } else if (name == "noise_reduction") {
        candidate.noise_reduction = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "method_threshold") {
        candidate.method_threshold = parameter.as_string();
        warn_if_unknown_threshold_method(candidate.method_threshold);
      } else if (name == "segmentation_type" || name == "scan_topic" || name == "segments_topic") {
        // These parameters are only read on configure; reject runtime changes
        // instead of silently accepting a change that will not take effect.
        result.successful = false;
        result.reason = "The parameter '" + name +
          "' can only be set before the node is configured.";
        RCLCPP_WARN(logger_, "%s", result.reason.c_str());
        return result;
      }
    }
  }

  // Validate the coherence of the min/max pairs.
  if (candidate.min_points_segment > candidate.max_points_segment) {
    result.successful = false;
    result.reason = "'min_points_segment' cannot be greater than 'max_points_segment'.";
  } else if (candidate.min_avg_distance_from_sensor > candidate.max_avg_distance_from_sensor) {
    result.successful = false;
    result.reason =
      "'min_avg_distance_from_sensor' cannot be greater than 'max_avg_distance_from_sensor'.";
  } else if (candidate.min_segment_width > candidate.max_segment_width) {
    result.successful = false;
    result.reason = "'min_segment_width' cannot be greater than 'max_segment_width'.";
  }

  if (!result.successful) {
    RCLCPP_WARN(logger_, "%s", result.reason.c_str());
    return result;
  }

  // All checks passed: commit the new configuration.
  params_ = candidate;
  RCLCPP_INFO(logger_, "Updated laser_segmentation parameters.");

  return result;
}

void ParameterHandler::warn_if_unknown_threshold_method(const std::string & method)
{
  if (method.empty()) {
    // Empty means "use the fixed distance_threshold", which is a valid choice.
    return;
  }
  if (method != "lee" && method != "diet" && method != "santos") {
    RCLCPP_WARN(
      logger_,
      "Unknown method_threshold '%s': falling back to the fixed distance_threshold. "
      "Valid values are 'lee', 'diet', 'santos' or an empty string.",
      method.c_str());
  }
}

}  // namespace laser_segmentation
