/*
 * JUMP DISTANCE SEGMENTATION CLASS
 *
 * Copyright (c) 2017-2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of laser_segmentation.
 * 
 * All rights reserved.
 *
 */

#include "laser_segmentation/segmentation/segmentationJumpDistance.hpp"

/* Constructor */
void JumpDistanceSegmentation::initialize_segmentation(double jump_distance, double angle_resolution, double noise_reduction, std::string method){
	jump_distance_ = jump_distance;
	angle_resolution_ = angle_resolution;
	noise_reduction_ = noise_reduction;
	threshold_method_ = method;
}

/* Segment the list of points into a list of segments using Jump Distance Clustering */
void JumpDistanceSegmentation::perform_segmentation(const std::vector<slg::Point2D> points, std::vector<slg::Segment2D>& segments){
	int count = -1;

	/* Create the points and segment */
	slg::Point2D prev_point = slg::Point2D::NaN();
	slg::Point2D current_point = slg::Point2D::NaN();
	slg::Point2D next_point = slg::Point2D::NaN();
	slg::Segment2D current_segment;

	/* Iterate over the n points to create the segments */
	for (std::vector<slg::Point2D>::size_type p = 0; p < points.size(); p++){
		// Create current point
		current_point = points[p];
		// Check if point is valid
		if (!is_valid(current_point)) continue;
		// Create the next point. First, we check if it's the last loop
		if (p != points.size()-1){
			next_point = points[p+1];
		}else{
			next_point = slg::Point2D::NaN();
		}
		// Check if the points belong to the same segment or we have to create one
		if (!is_jump_between(prev_point, current_point)){
			// Add the point to the end of the actual segment
			current_segment.add_point(current_point);
			current_segment.set_next_segment(next_point);
		}else{
			// Add the actual segment to the list
			if(!current_segment.empty()) segments.push_back(current_segment);
			// And create a new segment
			current_segment = slg::Segment2D(++count, prev_point, current_point, next_point);
		}
		// Prepare next iteration
		prev_point = current_point;
	}

	/* Add the last segment to the list */
	segments.push_back(current_segment);

	/* Check if last and first segments belongs to the same segment */
	slg::Segment2D first_segment = segments.front();
	slg::Segment2D last_segment = segments.back();
	// Check if the point belong to the same segment or we have to discard it.
	// Remember that the points are radially sorted.
	if (!is_jump_between(last_segment, first_segment)){
		last_segment.merge(first_segment);
		// Insert the last segment as the first one
		segments[0] = last_segment;
		// And remove the last one
		segments.pop_back(); 
	}else{
		// Fix the prior segment of the first segment
		segments.front().set_prior_segment(last_segment.last_point());
		// Fix the next segment of the last segment
		segments.back().set_next_segment(first_segment.first_point());
	}
}

/* Checks if two adjacent points are close to each other. */
bool JumpDistanceSegmentation::is_jump_between(const slg::Point2D point1, const slg::Point2D point2){
	// Check if one or both points are NaN
	if (point1.is_NaN() || point2.is_NaN()) return true;
	// Calculate the distance between the two points
	double distance = (point2 - point1).length();
	// Select the method to calculate a dynamic jump distance threshold
	double new_jump_distance;
	if (threshold_method_ == "lee"){
		new_jump_distance = calculate_lee_threshold(point1, point2);
	}else if (threshold_method_ == "diet"){
		new_jump_distance = calculate_diet_threshold(point1, point2);
	}else if (threshold_method_ == "santos"){
		new_jump_distance = calculate_santos_threshold(point1, point2);
	}else{
		new_jump_distance = jump_distance_;
	}
	// Calculate distance threshold
	return distance > new_jump_distance;
}

/* Checks if two adjacent segments are close to each other. The order is important. */
bool JumpDistanceSegmentation::is_jump_between(const slg::Segment2D segment1, const slg::Segment2D segment2){
	return is_jump_between(segment1.last_point(), segment2.first_point());
}

/* Calculate jump distance using Lee method (Lee, 2001). */
double JumpDistanceSegmentation::calculate_lee_threshold(const slg::Point2D point1, const slg::Point2D point2){
	return fabs((point1.length() - point2.length()) / (point1.length() + point2.length()));
}

/* Calculate threshold condition using Dietmayer method (Dietmayer, et al., 2001). */
double JumpDistanceSegmentation::calculate_diet_threshold(const slg::Point2D point1, const slg::Point2D point2){
	double minRange = std::min(point1.length(), point2.length());
	double c0 = noise_reduction_;
	double c1 = sqrt(2.0 * (1.0 - cos(angle_resolution_)) );

	return c0 + c1 * minRange;
}

/* Calculate threshold condition using Santos method (Santos, et al., 2003). */
double JumpDistanceSegmentation::calculate_santos_threshold(const slg::Point2D point1, const slg::Point2D point2){
	double minRange = std::min(point1.length(), point2.length());
	double c0 = noise_reduction_;
	double c1 = sqrt(2.0 * (1.0 - cos(angle_resolution_)) );

	double beta = atan2(fabs(point1.y - point2.y), fabs(point1.x - point2.x));
	return c0 + (c1 * minRange * tan(beta)) / (cos(angle_resolution_ / 2) - sin(angle_resolution_ / 2));
}
