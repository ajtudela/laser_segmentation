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

void JumpDistanceSegmentationMerge::on_segment_created(
  std::vector<slg::Segment2D> & segments, slg::Segment2D & current_segment)
{
  // Check predecessors segments
  if (segments.size() > 2) {
    // Second-order comparison:
    // Check pre-pre-predecessor segment if it is close to the current point
    slg::Segment2D & prev_segment = segments[segments.size() - 3];
    if (!is_jump_between(prev_segment, current_segment)) {
      // Merge the current segment into the predecessor in place
      prev_segment.merge(current_segment);
      // Empty the current segment
      current_segment = slg::Segment2D();
    }
  } else if (segments.size() > 1) {
    // First-order comparison:
    // Check pre-predecessor segment to see if it is close to the current point
    slg::Segment2D & prev_segment = segments[segments.size() - 2];
    if (!is_jump_between(prev_segment, current_segment)) {
      // Merge the current segment into the predecessor in place
      prev_segment.merge(current_segment);
      // Empty the current segment
      current_segment = slg::Segment2D();
    }
  }
}

}  // namespace laser_segmentation
