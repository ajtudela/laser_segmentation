/*
 * SEGMENTATION ALGORITHM CLASS
 *
 * Copyright (c) 2017-2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of laser_segmentation.
 * 
 * All rights reserved.
 *
 */

#ifndef LASER_SEGMENTATION__SEGMENTATION_HPP_
#define LASER_SEGMENTATION__SEGMENTATION_HPP_

// C++
#include <memory>
#include <string>

// SIMPLE LASER GEOMETRY
#include "slg_msgs/point2D.hpp"
#include "slg_msgs/segment2D.hpp"

/* Abstract class for a generic segmentation algorithm. */
class Segmentation{
	public:
		// Destructor.
		virtual ~Segmentation(){};

		// Initialize segmentation algorithm.
		virtual void initialize_segmentation(double distance, double angle_resolution, double noise_reduction, std::string method = "") = 0;

		// Segment the given list of points. Consecutive points are assumed to be adjacent.
		virtual void perform_segmentation(const std::vector<slg::Point2D> points, std::vector<slg::Segment2D>& segments) = 0;

		// Typedefs for easier readability.
		typedef std::shared_ptr<Segmentation>SharedPtr;

	protected:
		// Check if the point is valid (i.e. not out-of-range)
		inline bool is_valid(const slg::Point2D point){
			return !point.is_NaN();
		}
};

#endif  // LASER_SEGMENTATION__SEGMENTATION_HPP_
