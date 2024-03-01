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

#ifndef LASER_SEGMENTATION__SEGMENTATION__SEGMENTATIONJUMPDISTANCE_HPP_
#define LASER_SEGMENTATION__SEGMENTATION__SEGMENTATIONJUMPDISTANCE_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

#include "laser_segmentation/segmentation/segmentation.hpp"

/* Classic jump distance segmentation algorithm. */
class JumpDistanceSegmentation : public Segmentation
{
public:
  virtual void initialize_segmentation(
    double distance, double angle_resolution,
    double noise_reduction, std::string method = "");
  virtual void perform_segmentation(
    const std::vector<slg::Point2D> points,
    std::vector<slg::Segment2D> & segments);

  typedef std::shared_ptr<JumpDistanceSegmentation> SharedPtr;

protected:
  // Checks if two adjacent points are close to each other.
  bool is_jump_between(const slg::Point2D point1, const slg::Point2D point2);

  // Checks if two adjacent segments are close to each other. The order is important.
  bool is_jump_between(const slg::Segment2D segment1, const slg::Segment2D segment2);

  // Calculate jump distance using Lee method (Lee, 2001).
  double calculate_lee_threshold(const slg::Point2D point1, const slg::Point2D point2);

  // Calculate jump distance using Dietmayer method (Dietmayer, et al., 2001).
  double calculate_diet_threshold(const slg::Point2D point1, const slg::Point2D point2);

  // Calculate jump distance using Santos method (Santos, et al., 2003).
  double calculate_santos_threshold(const slg::Point2D point1, const slg::Point2D point2);

  // The jump distance above which a new segment is created.
  double jump_distance_;

  // The angle resolution of the lidar.
  double angle_resolution_;

  // Parameter for noise reduction.
  double noise_reduction_;

  // Method to calculate a dynamic jump distance threshold
  std::string threshold_method_;
};

#endif  // LASER_SEGMENTATION__SEGMENTATION__SEGMENTATIONJUMPDISTANCE_HPP_
