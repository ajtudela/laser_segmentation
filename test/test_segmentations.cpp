// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include "gtest/gtest.h"
#include "slg_msgs/point2D.hpp"
#include "slg_msgs/segment2D.hpp"
#include "laser_segmentation/segmentation/jump_distance.hpp"
#include "laser_segmentation/segmentation/jump_distance_merge.hpp"

class JDistanceFixture : public laser_segmentation::JumpDistanceSegmentation
{
public:
  JDistanceFixture()
  : JumpDistanceSegmentation() {}

  bool is_jump_between(slg::Point2D prev_point, slg::Point2D current_point)
  {
    return JumpDistanceSegmentation::is_jump_between(prev_point, current_point);
  }

  bool is_jump_between(const slg::Segment2D segment1, const slg::Segment2D segment2)
  {
    return JumpDistanceSegmentation::is_jump_between(segment1, segment2);
  }

  double calculate_lee_threshold(const slg::Point2D point1, const slg::Point2D point2)
  {
    return JumpDistanceSegmentation::calculate_lee_threshold(point1, point2);
  }

  double calculate_diet_threshold(const slg::Point2D point1, const slg::Point2D point2)
  {
    return JumpDistanceSegmentation::calculate_diet_threshold(point1, point2);
  }

  double calculate_santos_threshold(const slg::Point2D point1, const slg::Point2D point2)
  {
    return JumpDistanceSegmentation::calculate_santos_threshold(point1, point2);
  }

  double get_jump_distance()
  {
    return jump_distance_;
  }

  double get_angle_resolution()
  {
    return angle_resolution_;
  }

  double get_noise_reduction()
  {
    return noise_reduction_;
  }

  std::string get_threshold_method()
  {
    return threshold_method_;
  }
};

class JDistanceMergeFixture : public laser_segmentation::JumpDistanceSegmentationMerge
{
public:
  JDistanceMergeFixture()
  : JumpDistanceSegmentationMerge() {}

  double get_jump_distance()
  {
    return jump_distance_;
  }

  double get_angle_resolution()
  {
    return angle_resolution_;
  }

  double get_noise_reduction()
  {
    return noise_reduction_;
  }

  std::string get_threshold_method()
  {
    return threshold_method_;
  }
};

// Create a vector of points with a distance
std::vector<slg::Point2D> create_points(int num_points, double distance)
{
  std::vector<slg::Point2D> points;
  for (int i = 0; i < num_points; i++) {
    points.push_back(slg::Point2D(i * distance, i * distance, slg::BACKGROUND));
  }
  return points;
}

TEST(JumpDistanceTest, configure) {
  // Configure the segmentation
  JDistanceFixture segmentation;
  segmentation.initialize_segmentation(0.1, 0.2, 0.3, "jump_distance");

  // Check the parameters
  EXPECT_EQ(segmentation.get_jump_distance(), 0.1);
  EXPECT_EQ(segmentation.get_angle_resolution(), 0.2);
  EXPECT_EQ(segmentation.get_noise_reduction(), 0.3);
  EXPECT_EQ(segmentation.get_threshold_method(), "jump_distance");
}

TEST(JumpDistanceTest, distanceThresholds) {
  // Calculate the jump distance using Lee method
  JDistanceFixture segmentation;
  segmentation.initialize_segmentation(0.1, 0.2, 0.3, "test");
  slg::Point2D point1(0.0, 0.0);
  slg::Point2D point2(0.5, 0.35);
  // Check distance|point1 - point2|
  EXPECT_DOUBLE_EQ((point2 - point1).length(), 0.61032778078668515);

  double jump_distance = segmentation.calculate_lee_threshold(point1, point2);
  // Check the jump distance
  EXPECT_DOUBLE_EQ(jump_distance, 1.0);

  // Calculate the jump distance using Dietmayer method
  jump_distance = segmentation.calculate_diet_threshold(point1, point2);
  // Check the jump distance
  EXPECT_DOUBLE_EQ(jump_distance, 0.29999999999999999);

  // Calculate the jump distance using Santos method
  jump_distance = segmentation.calculate_santos_threshold(point1, point2);
  // Check the jump distance
  EXPECT_DOUBLE_EQ(jump_distance, 0.29999999999999999);
}

TEST(JumpDistanceTest, isJumpBetween) {
  JDistanceFixture segmentation;
  // Set the points
  slg::Point2D point1(0.0, 0.0);
  slg::Point2D point2(0.5, 0.35);

  // Calculate the jump distance when one of the points is NaN
  bool is_jump = segmentation.is_jump_between(point1, slg::Point2D::quiet_NaN());
  EXPECT_TRUE(is_jump);
  is_jump = segmentation.is_jump_between(slg::Point2D::quiet_NaN(), point2);
  EXPECT_TRUE(is_jump);
  is_jump = segmentation.is_jump_between(slg::Point2D::quiet_NaN(), slg::Point2D::quiet_NaN());
  EXPECT_TRUE(is_jump);

  // Calculate the jump distance using Lee method
  segmentation.initialize_segmentation(0.1, 0.2, 0.3, "lee");
  is_jump = segmentation.is_jump_between(point1, point2);
  // Check the result: 0.61 > 1.0 is false
  EXPECT_FALSE(is_jump);

  // Calculate the jump distance using Dietmayer method
  segmentation.initialize_segmentation(0.1, 0.2, 0.3, "diet");
  is_jump = segmentation.is_jump_between(point1, point2);
  // Check the result: 0.61 > 0.3 is true
  EXPECT_TRUE(is_jump);

  // Calculate the jump distance using Santos method
  segmentation.initialize_segmentation(0.1, 0.2, 0.3, "santos");
  is_jump = segmentation.is_jump_between(point1, point2);
  // Check the result: 0.61 > 0.3 is true
  EXPECT_TRUE(is_jump);

  // Calculate the jump distance using default method
  segmentation.initialize_segmentation(0.1, 0.2, 0.3, "test");
  is_jump = segmentation.is_jump_between(point1, point2);
  // Check the result: 0.61 > 0.1 is true
  EXPECT_TRUE(is_jump);

  // Set the segments
  slg::Segment2D segment1;
  segment1.add_point(slg::Point2D(1.0, 2.0, slg::BACKGROUND));
  segment1.add_point(slg::Point2D(3.0, 4.0, slg::BACKGROUND));
  segment1.add_point(slg::Point2D(5.0, 6.0, slg::BACKGROUND));
  slg::Segment2D segment2;
  segment2.add_point(slg::Point2D(7.3, 8.0, slg::BACKGROUND));
  segment2.add_point(slg::Point2D(7.5, 8.1, slg::BACKGROUND));
  segment2.add_point(slg::Point2D(8.0, 9.5, slg::BACKGROUND));
  segment2.add_point(slg::Point2D(10.0, 11.0, slg::BACKGROUND));
  // Check distance between segments
  point1 = segment1.last_point();
  point2 = segment2.first_point();
  EXPECT_DOUBLE_EQ((point2 - point1).length(), 3.047950130825634);
  // Calculate the jump distance using default method
  is_jump = segmentation.is_jump_between(segment1, segment2);
  // Check the result: 3.0 > 0.1 is true
  EXPECT_TRUE(is_jump);
}

TEST(JumpDistanceTest, performSegmentation) {
  JDistanceFixture segmentation;
  // Set the jump distance to 0.1
  segmentation.initialize_segmentation(0.1, 0.2, 0.3, "test");

  // Set points with a distance of 0.05
  std::vector<slg::Point2D> points = create_points(5, 0.05);
  points.push_back(slg::Point2D::quiet_NaN());
  // Perform the segmentation
  std::vector<slg::Segment2D> segments;
  segmentation.perform_segmentation(points, segments);
  // Check the number of segments
  // Should be 1 segment as the jump distance is 0.1
  // and the distance between points is 0.05
  EXPECT_EQ(segments.size(), 1);
  // Check the number of points in the segment
  // Should be 5 points in the segment
  // as the last point is a NaN point
  EXPECT_EQ(segments[0].size(), points.size() - 1);

  // Now we set the points with a distance of 1.0
  points = create_points(5, 1.0);
  // Perform the segmentation
  segments.clear();
  segmentation.perform_segmentation(points, segments);
  // Check the number of segments
  // Should be 1 segment per point as the jump distance is 0.1
  // and the distance between points is 1.0
  EXPECT_EQ(segments.size(), points.size());
  // Check the number of points in the segment
  for (auto segment : segments) {
    EXPECT_EQ(segment.size(), 1);
  }

  // Now we set the points with a distance of 1.0
  // and the last point is near the first point
  points = create_points(3, 1.0);
  points.push_back(slg::Point2D(0.05, 0.05, slg::BACKGROUND));
  // Perform the segmentation
  segments.clear();
  segmentation.perform_segmentation(points, segments);
  // Check the number of segments
  // Should be 3 segments as the jump distance is 0.1
  // and the distance between points is 1.0
  // but the last point is near the first point
  EXPECT_EQ(segments.size(), 3);
  // Check the number of points in the segment
  EXPECT_EQ(segments[0].size(), 2);
  EXPECT_EQ(segments[1].size(), 1);
  EXPECT_EQ(segments[2].size(), 1);
}

TEST(JumpDistanceMergeTest, configure) {
  // Configure the segmentation
  JDistanceMergeFixture segmentation;
  segmentation.initialize_segmentation(0.1, 0.2, 0.3, "jump_distance");

  // Check the parameters
  EXPECT_EQ(segmentation.get_jump_distance(), 0.1);
  EXPECT_EQ(segmentation.get_angle_resolution(), 0.2);
  EXPECT_EQ(segmentation.get_noise_reduction(), 0.3);
  EXPECT_EQ(segmentation.get_threshold_method(), "jump_distance");
}

TEST(JumpDistanceMergeTest, performSegmentation) {
  JDistanceMergeFixture segmentation;
  // Set the jump distance to 0.1
  segmentation.initialize_segmentation(0.1, 0.2, 0.3, "test");

  // Set points with a distance of 0.05
  std::vector<slg::Point2D> points = create_points(3, 0.05);
  points.push_back(slg::Point2D::quiet_NaN());
  // Perform the segmentation
  std::vector<slg::Segment2D> segments;
  segmentation.perform_segmentation(points, segments);
  // Check the number of segments
  // Should be 1 segment as the jump distance is 0.1
  // and the distance between points is 0.05
  EXPECT_EQ(segments.size(), 1);
  // Check the number of points in the segment
  // Should be 3 points in the segment
  // as the last point is a NaN point
  EXPECT_EQ(segments[0].size(), 3);

  // Now we set the points with a distance of 1.0
  points = create_points(3, 1.0);
  // Perform the segmentation
  segments.clear();
  segmentation.perform_segmentation(points, segments);
  // Check the number of segments
  // Should be 1 segment per point as the jump distance is 0.1
  // and the distance between points is 1.0
  EXPECT_EQ(segments.size(), points.size());
  // Check the number of points in the segment
  for (auto segment : segments) {
    EXPECT_EQ(segment.size(), 1);
  }

  // Now we set the points with a distance of 1.0
  // and the last point is near the first point
  points = create_points(3, 1.0);
  points.push_back(slg::Point2D(0.05, 0.05, slg::BACKGROUND));
  // Perform the segmentation
  segments.clear();
  segmentation.perform_segmentation(points, segments);
  // Check the number of segments
  // Should be 3 segments as the jump distance is 0.1
  // and the distance between points is 1.0
  // but the last point is near the first point
  EXPECT_EQ(segments.size(), 3);
  // Check the number of points in the segment
  EXPECT_EQ(segments[0].size(), 2);
  EXPECT_EQ(segments[1].size(), 1);
  EXPECT_EQ(segments[2].size(), 1);

  // Now we set the points with a distance of 0.05
  // and two of the points are spurious points
  points.clear();
  points.push_back(slg::Point2D(0.0, 0.0, slg::BACKGROUND));
  points.push_back(slg::Point2D(1.0, 1.0, slg::BACKGROUND));
  points.push_back(slg::Point2D(0.05, 0.05, slg::BACKGROUND));
  points.push_back(slg::Point2D(0.06, 0.06, slg::BACKGROUND));
  points.push_back(slg::Point2D(3.0, 3.0, slg::BACKGROUND));
  points.push_back(slg::Point2D(4.0, 4.0, slg::BACKGROUND));
  points.push_back(slg::Point2D(0.065, 0.065, slg::BACKGROUND));
  points.push_back(slg::Point2D(0.05, 0.05, slg::BACKGROUND));
  // Perform the segmentation
  segments.clear();
  segmentation.perform_segmentation(points, segments);
  // Check the number of segments
  // Should be 5 segments as the jump distance is 0.1
  // and the distance between points is 0.05
  // but the last point is near the first point
  EXPECT_EQ(segments.size(), 5);
  // Check the number of points in the segment
  EXPECT_EQ(segments[0].size(), 3);
  EXPECT_EQ(segments[1].size(), 1);
  EXPECT_EQ(segments[2].size(), 2);
  EXPECT_EQ(segments[3].size(), 1);
  EXPECT_EQ(segments[4].size(), 1);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
