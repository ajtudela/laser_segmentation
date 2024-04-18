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

#include "laser_segmentation/segmentation/jump_distance_merge.hpp"

namespace laser_segmentation
{

void JumpDistanceSegmentationMerge::initialize_segmentation(
  double jump_distance,
  double angle_resolution,
  double noise_reduction,
  std::string method)
{
  jump_distance_ = jump_distance;
  angle_resolution_ = angle_resolution;
  noise_reduction_ = noise_reduction;
  threshold_method_ = method;
}

void JumpDistanceSegmentationMerge::perform_segmentation(
  const std::vector<slg::Point2D> points,
  std::vector<slg::Segment2D> & segments)
{
  int count = -1;

  // Create the points and segment
  slg::Point2D prev_point = slg::Point2D::quiet_NaN();
  slg::Point2D current_point = slg::Point2D::quiet_NaN();
  slg::Point2D next_point = slg::Point2D::quiet_NaN();
  slg::Segment2D current_segment;

  // Iterate over the n points to create the segments
  for (std::vector<slg::Point2D>::size_type p = 0; p < points.size(); p++) {
    // Create current point
    current_point = points[p];
    // Skip invalid points
    if (!is_valid(current_point)) {
      continue;
    }
    // Create the next point.
    // First, we check if it's the last loop
    if (p != points.size() - 1) {
      next_point = points[p + 1];
    } else {
      next_point = slg::Point2D::quiet_NaN();
    }
    // Check if the points belong to the same segment or we have to create one
    if (!is_jump_between(prev_point, current_point)) {
      // Add the point to the end of the actual segment
      current_segment.add_point(current_point);
      current_segment.set_next_segment(next_point);
    } else {
      // Add the actual segment to the list
      if (!current_segment.empty()) {
        segments.push_back(current_segment);
      }
      // And create a new segment
      current_segment = slg::Segment2D(++count, prev_point, current_point, next_point);
      // Check predecessors segments
      if (segments.size() > 2) {
        // Second-order comparison:
        // Check pre-pre-predecessor segment if it is close to the current point
        slg::Segment2D prev_segment = segments[segments.size() - 3];
        if (!is_jump_between(prev_segment, current_segment)) {
          // Merge both segments
          prev_segment.merge(current_segment);
          // Push back to the list
          segments[segments.size() - 3] = prev_segment;
          // Empty the current segment
          current_segment = slg::Segment2D();
        }
      } else if (segments.size() > 1) {
        // First-order comparison:
        // Check pre-predecessor segment to see if it is close to the current point
        slg::Segment2D prev_segment = segments[segments.size() - 2];
        if (!is_jump_between(prev_segment, current_segment)) {
          // Merge both segments
          prev_segment.merge(current_segment);
          // Push back to the list
          segments[segments.size() - 2] = prev_segment;
          // Empty the current segment
          current_segment = slg::Segment2D();
        }
      }
    }
    // Prepare next iteration
    prev_point = current_point;
  }

  // Add the last segment to the list
  if (!current_segment.empty()) {
    segments.push_back(current_segment);
  }

  // Check if last and first segments belongs to the same segment
  if (segments.size() > 1) {
    slg::Segment2D first_segment = segments.front();
    slg::Segment2D last_segment = segments.back();
    // Check if the point belong to the same segment or we have to discard it.
    // Remember that the points are radially sorted.
    if (!is_jump_between(last_segment, first_segment)) {
      last_segment.merge(first_segment);
      // Insert the last segment as the first one
      segments[0] = last_segment;
      // And remove the last one
      segments.pop_back();
    } else {
      // Fix the prior segment of the first segment
      segments.front().set_prior_segment(last_segment.last_point());
      // Fix the next segment of the last segment
      segments.back().set_next_segment(first_segment.first_point());
    }
  }
}

}  // namespace laser_segmentation
