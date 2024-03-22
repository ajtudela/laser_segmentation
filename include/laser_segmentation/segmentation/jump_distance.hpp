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

#ifndef LASER_SEGMENTATION__SEGMENTATION__JUMP_DISTANCE_HPP_
#define LASER_SEGMENTATION__SEGMENTATION__JUMP_DISTANCE_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

#include "laser_segmentation/segmentation/segmentation.hpp"

namespace laser_segmentation
{

/**
 * @brief Classic jump distance segmentation algorithm.
 *
 */
class JumpDistanceSegmentation : public Segmentation
{
public:
/**
 * @brief Construct a new Jump Distance Segmentation object
 *
 */
  JumpDistanceSegmentation() = default;

/**
 * @brief Destroy the Jump Distance Segmentation object
 *
 */
  ~JumpDistanceSegmentation() override = default;

  /**
 * @brief Initialize the segmentation algorithm.
 *
 * @param distance The maximum distance between two consecutive points
 * to be considered part of the same segment.
 * @param angle_resolution The minimum angle between two consecutive points.
 * @param noise_reduction Parameter for noise reduction (if applicable).
 * @param method The method to be used for segmentation.
 */
  void initialize_segmentation(
    double distance, double angle_resolution,
    double noise_reduction, std::string method = "") override;

  /**
 * @brief Perform the segmentation of the given list of points
 * into a list of segments using Jump Distance Clustering.
 *
 * @param points The list of points to be segmented.
 * @param segments The resulting list of segments.
 */
  void perform_segmentation(
    const std::vector<slg::Point2D> points,
    std::vector<slg::Segment2D> & segments) override;

protected:
  /**
   * @brief Checks if two adjacent points are close to each other.
   *
   * @param point1 First point
   * @param point2 Second point
   * @return true If the points are close to each other
   * @return false If the points are not close to each other
   */
  bool is_jump_between(const slg::Point2D point1, const slg::Point2D point2);

  /**
   * @brief Checks if two adjacent segments are close to each other. The order is important.
   *
   * @param segment1 First segment
   * @param segment2 Second segment
   * @return true If the segments are close to each other
   * @return false If the segments are not close to each other
   */
  bool is_jump_between(const slg::Segment2D segment1, const slg::Segment2D segment2);

  /**
   * @brief Calculate jump distance using Lee method (Lee, 2001).
   *
   * @param point1 First point
   * @param point2 Second point
   * @return double The jump distance
   */
  double calculate_lee_threshold(const slg::Point2D point1, const slg::Point2D point2);

  /**
   * @brief Calculate jump distance using Dietmayer method (Dietmayer, et al., 2001).
   *
   * @param point1 First point
   * @param point2 Second point
   * @return double The jump distance
   */
  double calculate_diet_threshold(const slg::Point2D point1, const slg::Point2D point2);

  /**
   * @brief Calculate jump distance using Santos method (Santos, et al., 2003).
   *
   * @param point1 First point
   * @param point2 Second point
   * @return double The jump distance
   */
  double calculate_santos_threshold(const slg::Point2D point1, const slg::Point2D point2);

  /**
   * @brief The jump distance above which a new segment is created.
   *
   */
  double jump_distance_;

  /**
   * @brief The angle resolution of the lidar.
   *
   */
  double angle_resolution_;

  /**
   * @brief Parameter for noise reduction.
   *
   */
  double noise_reduction_;

  /**
   * @brief Method to calculate a dynamic jump distance threshold.
   *
   */
  std::string threshold_method_;
};

}  // namespace laser_segmentation

#endif  // LASER_SEGMENTATION__SEGMENTATION__JUMP_DISTANCE_HPP_
