/*
 * LASER SEGMENTATION ROS NODE
 *
 * Copyright (c) 2017-2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of laser_segmentation.
 * 
 * All rights reserved.
 *
 */

#include "nav2_util/node_utils.hpp"

#include "laser_segmentation/laserSegmentation.hpp"

/* Initialize the subscribers and publishers */
laserSegmentation::laserSegmentation(): Node("laser_segmentation"), setup_(false){
	// Initialize ROS parameters

	// INTEGER PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "min_points_segment", rclcpp::ParameterValue(3), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Minimium number of points per segment")
							.set__integer_range({rcl_interfaces::msg::IntegerRange()
								.set__from_value(0)
								.set__to_value(100)
								.set__step(1)}
								));
	this->get_parameter("min_points_segment", min_points_);
	RCLCPP_INFO(this->get_logger(), "The parameter min_points_segment is set to: %d", min_points_);

	nav2_util::declare_parameter_if_not_declared(this, "max_points_segment", rclcpp::ParameterValue(200), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Maximum number of points per segment")
							.set__integer_range({rcl_interfaces::msg::IntegerRange()
								.set__from_value(5)
								.set__to_value(500)
								.set__step(1)}
								));
	this->get_parameter("max_points_segment", max_points_);
	RCLCPP_INFO(this->get_logger(), "The parameter max_points_segment is set to: %d", max_points_);

	// FLOAT PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "min_avg_distance_from_sensor", rclcpp::ParameterValue(0.0), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Minimium average distance from sensor")
							.set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
								.set__from_value(0.0)
								.set__to_value(1.0)
								.set__step(0.1)}
								));
	this->get_parameter("min_avg_distance_from_sensor", min_avg_distance_from_sensor_);
	RCLCPP_INFO(this->get_logger(), "The parameter min_avg_distance_from_sensor is set to: %f", min_avg_distance_from_sensor_);

	nav2_util::declare_parameter_if_not_declared(this, "max_avg_distance_from_sensor", rclcpp::ParameterValue(20.0), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Maximum average distance from sensor" )
							.set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
								.set__from_value(0.0)
								.set__to_value(50.0)
								.set__step(1.0)}
							));
	this->get_parameter("max_avg_distance_from_sensor", max_avg_distance_from_sensor_);
	RCLCPP_INFO(this->get_logger(), "The parameter max_avg_distance_from_sensor is set to: %f", max_avg_distance_from_sensor_);

	nav2_util::declare_parameter_if_not_declared(this, "min_segment_width", rclcpp::ParameterValue(0.20), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Minimium width of the segment." )
							.set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
								.set__from_value(0.0)
								.set__to_value(5.0)
								.set__step(0.1)}
							));
	this->get_parameter("min_segment_width",            min_segment_width_);
	RCLCPP_INFO(this->get_logger(), "The parameter min_segment_width is set to: %f", min_segment_width_);

	nav2_util::declare_parameter_if_not_declared(this, "max_segment_width", rclcpp::ParameterValue(10.0), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Maximum width of the segment." )
							.set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
								.set__from_value(0.0)
								.set__to_value(100.0)
								.set__step(1.0)}
							));
	this->get_parameter("max_segment_width",            max_segment_width_);
	RCLCPP_INFO(this->get_logger(), "The parameter max_segment_width is set to: %f", max_segment_width_);

	nav2_util::declare_parameter_if_not_declared(this, "distance_threshold", rclcpp::ParameterValue(0.30), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The jump distance above which a new segment is created or noise reduction if method is selected." )
							.set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
								.set__from_value(0.0)
								.set__to_value(2.0)
								.set__step(0.1)}
							));
	this->get_parameter("distance_threshold",           distance_thres_);
	RCLCPP_INFO(this->get_logger(), "The parameter distance_threshold is set to: %f", distance_thres_);

	nav2_util::declare_parameter_if_not_declared(this, "noise_reduction", rclcpp::ParameterValue(0.30), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Parameter for noise reduction in 'Santos' and 'Dietmayer' algorithms." )
							.set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
								.set__from_value(0.0)
								.set__to_value(2.0)
								.set__step(0.1)}
							));
	this->get_parameter("noise_reduction",              noise_reduction_);
	RCLCPP_INFO(this->get_logger(), "The parameter noise_reduction is set to: %f", noise_reduction_);

	// BOOLEAN PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "restore_defaults", rclcpp::ParameterValue(false), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Restore to the original configuration"));
	this->get_parameter("restore_defaults", restore_);
	RCLCPP_INFO(this->get_logger(), "The parameter restore_defaults is set to: %s", restore_ ? "true" : "false");

	// STRING PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "method_threshold", rclcpp::ParameterValue(""), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Method to calculate a dynamic jump distance threshold in jump_distance algorithm"));
	this->get_parameter("method_threshold", method_thres_);
	RCLCPP_INFO(this->get_logger(), "The parameter method_threshold is set to: %s", method_thres_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "scan_topic", rclcpp::ParameterValue("scan"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Laser topic to read"));
	this->get_parameter("scan_topic", scan_topic_);
	RCLCPP_INFO(this->get_logger(), "The parameter scan_topic is set to: %s", scan_topic_.c_str());
	
	nav2_util::declare_parameter_if_not_declared(this, "segments_topic", rclcpp::ParameterValue("segments"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("published Segements topic"));
	this->get_parameter("segments_topic", seg_topic_);
	RCLCPP_INFO(this->get_logger(), "The parameter segment_topic is set to: %s", seg_topic_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "segmentation_type", rclcpp::ParameterValue("jump_distance"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Type of segmentation"));
	this->get_parameter("segmentation_type", segmentation_type_);
	RCLCPP_INFO(this->get_logger(), "The parameter segmentation_type is set to: %s", segmentation_type_.c_str());

	// Callback for monitor changes in parameters
	callback_handle_ = this->add_on_set_parameters_callback(
						std::bind(&laserSegmentation::parameters_callback, this, std::placeholders::_1));

	// Setting for segmentation algorithm
	if (segmentation_type_ == "jump_distance"){
		segmentation_.reset(new JumpDistanceSegmentation);
	}else if(segmentation_type_ == "jump_distance_merge"){
		segmentation_.reset(new JumpDistanceSegmentationMerge);
	}else{
		RCLCPP_FATAL(this->get_logger(), "Segmentation algorithm is invalid: %s", segmentation_type_.c_str());
		return;
	}
	
	// Publishers
	segment_pub_              = this->create_publisher<slg_msgs::msg::SegmentArray>(seg_topic_, 1);
	segment_viz_points_pub_   = this->create_publisher<visualization_msgs::msg::MarkerArray>("segments_viz", 10);
	segment_viz_id_pub_       = this->create_publisher<visualization_msgs::msg::MarkerArray>("segments_id_viz", 10);

	// Subscribers
	auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
						scan_topic_, 
						default_qos, 
						std::bind(&laserSegmentation::scan_callback, this, std::placeholders::_1));
}

laserSegmentation::~laserSegmentation() {
}

rcl_interfaces::msg::SetParametersResult laserSegmentation::parameters_callback(const std::vector<rclcpp::Parameter> &parameters){
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";

	for (const auto &param: parameters){
		// INTEGER PARAMS ..........................................................................
		if (param.get_name() == "min_points_segment" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER){
				min_points_ = param.as_int();
				RCLCPP_INFO(this->get_logger(), "The parameter min_points_segment is set to: %d", min_points_);
		}

		if (param.get_name() == "max_points_segment" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER){
				max_points_ = param.as_int();
				RCLCPP_INFO(this->get_logger(), "The parameter max_points_segment is set to: %d", max_points_);
		}
		// FLOAT PARAMS ..........................................................................
		if (param.get_name() == "min_avg_distance_from_sensor" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
				min_avg_distance_from_sensor_ = param.as_double ();
				RCLCPP_INFO(this->get_logger(), "The parameter min_avg_distance_from_sensor is set to: %3.3f", min_avg_distance_from_sensor_);
		}

		if (param.get_name() == "max_avg_distance_from_sensor" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
				max_avg_distance_from_sensor_ = param.as_double ();
				RCLCPP_INFO(this->get_logger(), "The parameter max_avg_distance_from_sensor is set to: %3.3f", max_avg_distance_from_sensor_);
		}		
		
		if (param.get_name() == "min_segment_width" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
				min_segment_width_ = param.as_double ();
				RCLCPP_INFO(this->get_logger(), "The parameter min_segment_width is set to: %3.3f", min_segment_width_);
		}

		if (param.get_name() == "max_segment_width" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
				max_segment_width_ = param.as_double ();
				RCLCPP_INFO(this->get_logger(), "The parameter max_segment_width is set to: %3.3f", max_segment_width_);
		}

		if (param.get_name() == "distance_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
				distance_thres_ = param.as_double ();
				RCLCPP_INFO(this->get_logger(), "The parameter distance_threshold is set to: %3.3f", distance_thres_);
		}

		if (param.get_name() == "noise_reduction" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
				distance_thres_ = param.as_double ();
				RCLCPP_INFO(this->get_logger(), "The parameter noise_reduction is set to: %3.3f", noise_reduction_);
		}

		// STRING PARAMS ..........................................................................
		this->get_parameter("method_threshold", method_thres_);
		if (param.get_name() == "method_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
				method_thres_ = param.as_string();
				RCLCPP_INFO(this->get_logger(), "The parameter method_threshold is set to: %s", method_thres_.c_str());
		}
	}

	return result;
}

/* Callback function */
void laserSegmentation::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg){
	// Note: Only perform laserscan segmentation if there's any subscriber
	if (segment_pub_->get_subscription_count() == 0 && 
		segment_viz_id_pub_->get_subscription_count() == 0 && 
		segment_viz_points_pub_->get_subscription_count() == 0){
		return;
	}
	RCLCPP_INFO_ONCE(this->get_logger(), "Subscribed to laser scan topic: %s", scan_topic_.c_str());

	// Read the laser scan
	std::vector<slg::Point2D> point_list;
	double phi = scan_msg->angle_min;
	double angle_resolution = scan_msg->angle_increment;
	for (const auto r: scan_msg->ranges){
		if (r >= scan_msg->range_min && r <= scan_msg->range_max){
			point_list.push_back(slg::Point2D::from_polar_coords(r, phi));
		}else{
			point_list.push_back(slg::Point2D::NaN());
		}
		phi += angle_resolution;
	}

	// Segment the points
	std::vector<slg::Segment2D> segment_list;
	segmentation_->initialize_segmentation(distance_thres_, angle_resolution, noise_reduction_, method_thres_);
	segmentation_->perform_segmentation(point_list, segment_list);

	// Filter segments
	double squared_min_segment_width = min_segment_width_ * min_segment_width_;
	double squared_max_segment_width = max_segment_width_ * max_segment_width_;
	std::vector<slg::Segment2D> segment_filtered_list;
	segment_filtered_list.reserve(segment_list.size());

	for (const auto& segment: segment_list){
		// By number of points
		if (segment.size() < min_points_ || segment.size() > max_points_) continue;

		// By distance to sensor
		if (segment.centroid().length() < min_avg_distance_from_sensor_ || segment.centroid().length() > max_avg_distance_from_sensor_) continue;

		// By width
		if (segment.width_squared() < squared_min_segment_width || segment.width_squared() > squared_max_segment_width) continue;

		segment_filtered_list.push_back(segment);
	}

	// Identification of segments and set angular distance
	for (long unsigned int s = 0; s < segment_filtered_list.size(); s++){
		double angle = std::min(scan_msg->angle_max - segment_filtered_list[s].max_angle(), 
								segment_filtered_list[s].min_angle() - scan_msg->angle_min);
		segment_filtered_list[s].set_id(s);
		segment_filtered_list[s].set_angular_distance_to_closest_boundary(angle);
	}

	// Publish the segment array
	slg_msgs::msg::SegmentArray segment_array_msg;
	segment_array_msg.header = scan_msg->header;
	for (const auto& segment: segment_filtered_list){
		segment_array_msg.segments.push_back(segment);
	}
	segment_pub_->publish(segment_array_msg);

	// Shows the segments
	show_visualization(scan_msg->header, segment_filtered_list);
}

/* Show the segments in rviz */
void laserSegmentation::show_visualization(std_msgs::msg::Header header, std::vector<slg::Segment2D> segment_list){
	/* Create a marker point */
	visualization_msgs::msg::MarkerArray viz_point_array;
	visualization_msgs::msg::Marker viz_point;
	viz_point.header = header;
	viz_point.lifetime = rclcpp::Duration(0, 10);
	viz_point.ns = "segments";
	viz_point.type = visualization_msgs::msg::Marker::POINTS;
	viz_point.action = visualization_msgs::msg::Marker::ADD;
	viz_point.scale.x = 0.02;
	viz_point.scale.y = 0.02;

	/* Create a marker id text */
	visualization_msgs::msg::MarkerArray viz_text_array;
	visualization_msgs::msg::Marker viz_text;
	viz_text.header = header;
	viz_text.lifetime = rclcpp::Duration(0, 10);
	viz_text.ns = "segments_names";
	viz_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
	viz_text.action = visualization_msgs::msg::Marker::ADD;
	viz_text.pose.orientation.x = 0.0;
	viz_text.pose.orientation.y = 0.0;
	viz_text.pose.orientation.z = 0.0;
	viz_text.pose.orientation.w = 1.0;
	viz_text.scale.z = 0.25;
	viz_text.color.r = 1.0;
	viz_text.color.g = 1.0;
	viz_text.color.b = 1.0;
	viz_text.color.a = 1.0;

	/* Create a deletion marker */
	visualization_msgs::msg::Marker deletion_marker;
	deletion_marker.header = header;
	deletion_marker.action = visualization_msgs::msg::Marker::DELETEALL;

	// Push the deletion marker
	viz_point_array.markers.push_back(deletion_marker);
	viz_text_array.markers.push_back(deletion_marker);

	/* Show the segments and the id */
	for (std::vector<slg::Segment2D>::size_type i = 0; i < segment_list.size(); i++){
		slg::Segment2D current_segment = segment_list[i];
		viz_point.id = i;
		viz_text.id = i;

		// Change the color of the segment
		viz_point.color = get_palette_color(i);

		// Iterate over the points of the segment
		for (slg::Point2D point: current_segment.get_points()){
			viz_point.points.push_back(point);
		}

		// Get position of the text
		viz_text.text = std::to_string(current_segment.get_id());
		viz_text.pose.position.x = current_segment.centroid().x;
		viz_text.pose.position.y = current_segment.centroid().y;
		viz_text.pose.position.z = 0.05;

		// Push to arrays
		viz_point_array.markers.push_back(viz_point);
		viz_text_array.markers.push_back(viz_text);

		// Clear markers
		viz_point.points.clear();
	}
	// Publish visualization
	segment_viz_points_pub_->publish(viz_point_array);
	segment_viz_id_pub_->publish(viz_text_array);
}

/* Get Parula color of the class */
std_msgs::msg::ColorRGBA laserSegmentation::get_parula_color(unsigned int index, unsigned int max){
	std_msgs::msg::ColorRGBA color;
	int div = round(256 / max);
	color.r = parula[index * div][0];
	color.g = parula[index * div][1];
	color.b = parula[index * div][2];
	color.a = 1.0;
	return color;
}

/* Get palette color of the class */
std_msgs::msg::ColorRGBA laserSegmentation::get_palette_color(unsigned int index){
	std_msgs::msg::ColorRGBA color;
	switch(index % 8) {
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

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<laserSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
