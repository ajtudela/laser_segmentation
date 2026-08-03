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

protected:
  /**
   * @brief Merge the newly created segment into an earlier predecessor when it is
   * close enough, instead of always starting a fresh segment after a jump. Checks
   * the pre-pre-predecessor first (second-order comparison), falling back to the
   * pre-predecessor (first-order comparison).
   *
   * @param segments The segments collected so far (not including current_segment).
   * @param current_segment The segment that was just created; cleared if merged.
   */
  void on_segment_created(
    std::vector<slg::Segment2D> & segments,
    slg::Segment2D & current_segment) override;
};

}  // namespace laser_segmentation

#endif  // LASER_SEGMENTATION__SEGMENTATION__JUMP_DISTANCE_MERGE_HPP_
