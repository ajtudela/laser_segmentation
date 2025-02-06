^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_segmentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.3 (06-02-2025)
------------------
* First jazzy release.

3.0.2 (31-07-2024)
------------------
* Update to use modern CMake idioms.

3.0.1 (12-06-2024)
------------------
* Remove nav2_util dependency.

3.0.0 (08-03-2024)
------------------
* Improve formatting.
* Improve documentation of source.
* Converted to Lifecycle node.
* Converted to component.
* Rename header and source files.
* Move parameters to new class: parameterHandler.
* Rename show_visualization to create_segment_viz_points.
* Fix jump distance merge.
* Add unit testing.

2.1.2 (23-06-2023)
------------------
* Rename visualization topic.

2.1.1 (29-11-2022)
------------------
* Add hierarchy between segmentationJumpDistance and segmentationJumpDistanceMerge to avoid code redundancy.
* Rise up centroids visualization.

2.1.0 (28-11-2022)
------------------
* Added include to find slg_msgs library.
* Added launch with some non-default param values.
* Minor changes to configuration info display.
* Added params folder with two yaml config files.
* Launch file now relies on config files, it is its only argument.
* Minor changes to range values in laser_segmentation paramas to accept smaller granularity.
* Added rviz CUBE marker with segment centroids.
* Reunited all markers into a single topic, using three namespaces.
* Added license.

2.0.0 (15-10-2022)
------------------
* First ROS2 (Humble) version.
* Contributors: Manuel Fernandez-Carmona

1.2.1 (20-10-2022)
------------------
* Rename segment_topic to segments_topic.

1.2.0 (03-05-2022)
------------------
* Added noise reduction parameter.
* Push a delete marker before the others. I hope this fix the RViz problem.

1.1.3 (13-01-2022)
------------------
* Refactoring jump distance and jump distance merge.
* Added jump between points with NaN values.
* Fix merge of first and last segments.

1.1.2 (12-01-2022)
------------------
* Update simple_laser_geometry to 3.2.0.
* Checking of NaN points.
* Change lifetime of markers.

1.1.1 (23-12-2021)
------------------
* Check publisher before doing any calculations.

1.1.0 (15-12-2021)
------------------
* Change segment marker to markerArray.
* Publish id of the segment as markerArray.
* Change the color palette.
* Fix jump distance merge.

1.0.1 (15-12-2021)
------------------
* Update simple_laser_geometry to 3.0.0.

1.0.0 (14-12-2021)
------------------
* Changes to new simple laser geometry library.
* Add max width filter.
* Remove readDataPoints.
* Added option to select threshold method.
* Update dynamic jump distance methods.
* Improve jump distance merge.

0.0.0 (-)
------------------
* Initial release.
* Create README.md.
* Added laserSegmentation class (.h and .cpp files).
* Added segmentationJumpDistance class (.h and .cpp files).
* Added segmentationJumpDistanceMerge class (.h and .cpp files).
* Added SegmentationParameters.cfg configuration files.
* Added launch files.
* Contributors: Alberto Tudela
