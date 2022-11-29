/*
 * JUMP DISTANCE MERGE SEGMENTATION CLASS
 *
 * Copyright (c) 2017-2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of laser_segmentation.
 * 
 * All rights reserved.
 *
 */

#ifndef LASER_SEGMENTATION__SEGMENTATION_JUMP_DISTANCE_MERGE_HPP
#define LASER_SEGMENTATION__SEGMENTATION_JUMP_DISTANCE_MERGE_HPP

#include "laser_segmentation/segmentation/segmentationJumpDistance.hpp"

/* Jump distance segmentation which merge segments by checking against the last point of preceding segments */
class JumpDistanceSegmentationMerge: public JumpDistanceSegmentation{
	public:
		virtual void initialize_segmentation(double distance, double angle_resolution, double noise_reduction, std::string method = "");
		virtual void perform_segmentation(const std::vector<slg::Point2D> points, std::vector<slg::Segment2D>& segments);

		typedef std::shared_ptr<JumpDistanceSegmentationMerge>SharedPtr;
};

#endif  // LASER_SEGMENTATION__SEGMENTATION_JUMP_DISTANCE_MERGE_HPP
