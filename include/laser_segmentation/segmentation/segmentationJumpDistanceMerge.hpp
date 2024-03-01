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

#ifndef LASER_SEGMENTATION__SEGMENTATION__SEGMENTATIONJUMPDISTANCEMERGE_HPP_
#define LASER_SEGMENTATION__SEGMENTATION__SEGMENTATIONJUMPDISTANCEMERGE_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

#include "laser_segmentation/segmentation/segmentationJumpDistance.hpp"

/**
 * @brief Jump distance segmentation algorithm which merge segments
 * by checking against the last point of preceding segments.
 *
 */
class JumpDistanceSegmentationMerge : public JumpDistanceSegmentation
{
public:
  virtual void initialize_segmentation(
    double distance, double angle_resolution,
    double noise_reduction, std::string method = "");
  virtual void perform_segmentation(
    const std::vector<slg::Point2D> points,
    std::vector<slg::Segment2D> & segments);

  typedef std::shared_ptr<JumpDistanceSegmentationMerge> SharedPtr;
};

#endif  // LASER_SEGMENTATION__SEGMENTATION__SEGMENTATIONJUMPDISTANCEMERGE_HPP_
