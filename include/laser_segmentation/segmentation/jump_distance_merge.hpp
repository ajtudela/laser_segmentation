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

#ifndef LASER_SEGMENTATION__SEGMENTATION__JUMP_DISTANCE_MERGE_HPP_
#define LASER_SEGMENTATION__SEGMENTATION__JUMP_DISTANCE_MERGE_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

#include "laser_segmentation/segmentation/jump_distance.hpp"

namespace laser_segmentation
{

/**
 * @brief Jump distance segmentation algorithm which merge segments
 * by checking against the last point of preceding segments.
 *
 */
class JumpDistanceSegmentationMerge : public JumpDistanceSegmentation
{
public:
  /**
   * @brief Construct a new Jump Distance Segmentation Merge object
   *
   */
  JumpDistanceSegmentationMerge() = default;

  /**
   * @brief Destroy the Jump Distance Segmentation Merge object
   *
   */
  ~JumpDistanceSegmentationMerge() override = default;

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

  typedef std::shared_ptr<JumpDistanceSegmentationMerge> SharedPtr;
};

}  // namespace laser_segmentation

#endif  // LASER_SEGMENTATION__SEGMENTATION__JUMP_DISTANCE_MERGE_HPP_
