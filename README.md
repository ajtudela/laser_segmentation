# laser_segmentation
![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/license-MIT-green)

## Overview

Implementation of differents algorithms for segmentation of laserscans, splitting them into subsets of beams, with a ROS2 interface. The currently implemented algorithm are:

* **`Jump distance clustering:`** widely used method for 2D laser range data in mobile robotics. It's a simple and fast method to segment the scans: if the Euclidean distance between two adjacent beams exceeds a given threshold distance, a new segment is generated.
* **`Jump distance clustering and merge:`** Similar algorithm as above but checks if pre-predecessor segments are close to each other. This deals with over-segmented data with many small cluster in outdoor environment. It uses the same threshold condition twice. 

In the two implementation mentioned above the jump distance threshold is fixed. However, a dynamic jump distance threshold can be calculated using methods of Lee (Lee, 2001), Dietmayer (Dietmayer, et al., 2001) and Santos (Santos, et al., 2003). See the parameters list below to change between them.

Includes a dynamic reconfigure server parameter to change online the configuration of the algorithms and the filtering.

**Keywords:** ROS2, laser, segmentation, clustering

**Author: Alberto Tudela<br />**

The laser_segmentation package has been tested under [ROS2] Humble on [Ubuntu] 22.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/humble/) (middleware for robotics),
- [slg_msgs](https://github.com/ajtudela/slg_msgs) (Library and messages to interact with laser related geometry - use Humble branch),

#### Building

To build from source, clone the latest version from the main repository into your catkin workspace and compile the package using

	cd colcon_workspace/src
	git clone https://github.com/ajtudela/laser_segmentation.git -b humble
	cd ../
	rosdep install -i --from-path src --rosdistro humble -y
	colcon build --symlink-install

## Usage

With some scan source running, run the laser_segmentation node with:
	ros2 launch laser_segmentation segmentation.launch.py

## Nodes

### laser_segmentation

Segmentation of the laserscans.

#### Subscribed Topics

* **`scan`** ([sensor_msgs/LaserScan])

	Laser scan topic where segmentation will be performed.

#### Published Topics

* **`segments`** ([slg_msgs/SegmentArray])

	Splitted segments resulting of the segmentation.

* **`segments_viz`** ([visualization_msgs/MarkerArray])

	It comprises three namespaces:

	- "segments": 3d markers of the segments for showing the segments in [Rviz2].

	- "segments_names": 3d markers with the id of the segments.

	- "centroids": 3d markers of the segment centroids in [Rviz2].

#### Parameters

* **`scan_topic`** (string, default: "scan")

	Topic of the laserscan.

* **`segment_topic`** (string, default: "segments")

	Topic of the segmented laser scan.

* **`segmentation_type`** (string, default: "jump_distance")

	Choose between several segmentation algorithms. Jump distance clustering (`jump_distance`) and jump distance and merge (`jump_distance_merge`).

#### Parameters for filtering

* **`min_points_segment`** (int, default: 3)

	Minimium number of points per segment.

* **`max_points_segment`** (int, default: 200)

	Maximum number of points per segment.

* **`min_avg_distance_from_sensor`** (double, default: 0.0)

	Minimium average distance from sensor.

* **`max_avg_distance_from_sensor`** (double, default: 20.0)

	Maximum average distance from sensor.

* **`min_segment_width`** (double, default: 0.2)

	Minimium width of the segment.

* **`max_segment_width`** (double, default: 10.0)

	Maximum width of the segment.

#### Parameters for segmentation algorithms

* **`method_threshold`** (string, default: "")

	Method to calculate a dynamic jump distance threshold in jump_distance or jump_distance_merge algorithms. This value and methods are based on `lee` (Lee, 2001), `diet` (Dietmayer, et al., 2001) or `santos` (Santos, et al., 2003).

* **`distance_threshold`** (double, default: 0.3)

	If none of the algorithms mentioned above is selected, this parameters will be use as the jump distance above which a new segment is created.

* **`noise_reduction`** (double, default: 0.3)

	Parameter for noise reduction in "Santos" and "Dietmayer" algorithms.

## Future work
- [ ] Add Delaunay triangulation method based on "Human Detection using Multimodal and Multidimensional Features" (Spinello).
- [ ] Add Agglomerative Hierarchical Clustering.
- [x] Change the decay of the markers because sometimes Rviz doesn't erase them. Maybe delete all at the end?
- [x] Update distance_threshold / method_threshold parameters. Too confusion.
- [ ] Restore default parameters in runtime.

[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/humble/
[Rviz2]: https://github.com/ros2/rviz
[sensor_msgs/LaserScan]: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
[slg_msgs/SegmentArray]: https://github.com/ajtudela/slg_msgs/blob/-/msg/SegmentArray.msg
[visualization_msgs/MarkerArray]: http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html
