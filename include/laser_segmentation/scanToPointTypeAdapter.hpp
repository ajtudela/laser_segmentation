/*
 * SCAN MSG TO POINT2D TYPE ADAPTER
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of laser_segmentation.
 * 
 * All rights reserved.
 *
 */

// C++
#include <vector>

// ROS
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slg_msgs/point2D.hpp"

template<>
struct rclcpp::TypeAdapter<std::vector<slg::Point2D>, sensor_msgs::msg::LaserScan>{
	using is_specialized = std::true_type;
	using custom_type = std::vector<slg::Point2D>;
	using ros_message_type = sensor_msgs::msg::LaserScan;

	static void convert_to_ros_message(const custom_type & source, ros_message_type & destination){
		//destination.data = source;
	}

	static void convert_to_custom(const ros_message_type & source, custom_type & destination){
		double phi = source.angle_min;
		double angle_resolution = source.angle_increment;
		for (const auto r: source.ranges){
			if (r >= source.range_min && r <= source.range_max){
				destination.push_back(slg::Point2D::from_polar_coords(r, phi));
			}else{
				destination.push_back(slg::Point2D::NaN());
			}
			phi += angle_resolution;
		}
	}
};