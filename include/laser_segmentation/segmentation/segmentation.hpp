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

#ifndef LASER_SEGMENTATION__SEGMENTATION__SEGMENTATION_HPP_
#define LASER_SEGMENTATION__SEGMENTATION__SEGMENTATION_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

// SIMPLE LASER GEOMETRY
#include "slg_msgs/point2D.hpp"
#include "slg_msgs/segment2D.hpp"

namespace laser_segmentation
{

/**
 * @brief Abstract class for a generic segmentation algorithm.
 *
 */
class Segmentation
{
public:
  /**
   * @brief Destroy the Segmentation object
   *
   */
  virtual ~Segmentation() {}

  /**
   * @brief Initialize the segmentation algorithm.
   *
   * @param distance The maximum distance between two consecutive points
   * to be considered part of the same segment.
   * @param angle_resolution The minimum angle between two consecutive points.
   * @param noise_reduction Parameter for noise reduction (if applicable).
   * @param method The method to be used for segmentation.
   */
  virtual void initialize_segmentation(
    double distance, double angle_resolution,
    double noise_reduction, std::string method = "") = 0;

  /**
   * @brief Perform the segmentation of the given list of points.
   *
   * @param points The list of points to be segmented.
   * @param segments The resulting list of segments.
   */
  virtual void perform_segmentation(
    const std::vector<slg::Point2D> points,
    std::vector<slg::Segment2D> & segments) = 0;

protected:
  /**
   * @brief Check if the point is valid (i.e. not out-of-range)
   *
   * @param point The point to be checked.
   * @return true If the point is valid.
   * @return false If the point is not valid.
   */
  inline bool is_valid(const slg::Point2D point)
  {
    return !point.isnan();
  }
};

}  // namespace laser_segmentation

#endif  // LASER_SEGMENTATION__SEGMENTATION__SEGMENTATION_HPP_
