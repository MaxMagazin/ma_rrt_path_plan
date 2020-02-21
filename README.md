# RRT with multiple remote goals.

This repository contains the ROS package with code implementation of RRT-based path planning algorithm suitable for exploration of a trackdrive circuit according to Formula Student Driverless competition.

![Basic animation of the approach](https://github.com/egnitionHamburg/ma_rrt_path_plan/blob/master/anim/drive.gif "RRT with multiple remote goals")

An  example of algorithm usage for exploration of FSG'18 trackdrive can be found [here](https://www.youtube.com/watch?v=kjssdifs0DQ). A brief introduction to the algorithm is given in my [Master's thesis presentation](https://www.youtube.com/watch?v=eOevF5jFSoc). A bit more detailed overview of algorithm steps can be found in [my thesis](https://drive.google.com/file/d/1vniFoNd71E5ITufIgLfM-vc1K-1cDCUu/view).

__NOTE__: The algorithm does not use the colors of cones, but instead a simple logic, which penalizes the branches having cones only from one side, and rewards the branches with cones from both sides (see findBestBranch(...) method). With cone classification info a better (and probably a more stable) rating system can be implemented.

## Inputs
- rospy.Subscriber("/map", Map, ...)
- rospy.Subscriber("/odometry", Odometry, ...)

## Outputs
- rospy.Publisher("/waypoints", WaypointsArray, ...)

## Outputs (Visuals)
- rospy.Publisher("/visual/tree_marker_array", MarkerArray, ...)
- rospy.Publisher("/visual/best_tree_branch", Marker, ...)
- rospy.Publisher("/visual/filtered_tree_branch", Marker, ...)
- rospy.Publisher("/visual/delaunay_lines", Marker, ...)
- rospy.Publisher("/visual/waypoints", MarkerArray, ...)

## Parameters to tune (main)
- planDistance = 12 m - Maximum length of tree branch
- expandDistance = 1 m - Length of tree node/step
- expandAngle = 20 deg - constraining angle for next tree nodes
- coneObstacleSize = 1.1 m - size of obstacles derived from cone position
- coneTargetsDistRatio = 0.5 - ratio to frontConesDist for deriving remote cone goals

## Licence

#### MIT
- Use me and my code in any way you want
- Keep the names and the same licence
