# RRT with multiple remote goals.

This repository contains the [ROS](https://www.ros.org/) package with code implementation of RRT-based path planning algorithm suitable for exploration of a trackdrive circuit according to the rules of Formula Student Driverless [competition](https://www.formulastudent.de/fsg/) in Germany. This algorithm is based on my contribution to [E-gnition](https://www.egnition.hamburg/), a FS Team of Technical University Hamburg.

![Basic animation of the approach](https://github.com/egnitionHamburg/ma_rrt_path_plan/blob/master/anim/drive.gif "RRT with multiple remote goals")

A brief introduction to the main steps of the proposed algorithm is given in my Master's thesis presentation [(direct timestamp)](https://youtu.be/eOevF5jFSoc?t=475).

[<img src="https://img.youtube.com/vi/eOevF5jFSoc/hqdefault.jpg" width="50%">](https://youtu.be/eOevF5jFSoc)

### Notes
- The algorithm does not utilize the cones' color information, but instead a simple logic is used, which rewards the branches with cones from both sides (see findBestBranch(...) method), and penalizes the branches having cones only from one side. With cone classification information a better rating system can be implemented and applied.
- Unfortunately I wasn't able to test and see this algorithm working on real hardware, a FS Driverless car, so I am excited if you can bring it the reality on any mobile robot and share some videos with me (see section **Usage**)

### Usage
- Exploration of [FSG'18 trackdrive circuit](https://www.youtube.com/watch?v=kjssdifs0DQ) in Gazebo.
- Exploration of [FSG'17 trackdrive circuit](https://www.youtube.com/watch?v=jJAjrCig3yE) in Gazebo.
- *Your video (feel free to pull-request a link with it here).*

#### Vehicle Messages
Some custom messages are included inside the `ma_rrt_path_plan` package folder. Because the Git repository was setup before the creation of the messages (via a PR from [ekampourakis](https://github.com/ekampourakis/)), the `vehicle_msgs` package folder needs to be moved to the parent folder of the `ma_rrt_path_plan` in order to act as a seperate package and possibly included in the Makefile too.

## Inputs, outputs, params

#### Inputs
- rospy.Subscriber("/map", Track, ...)
- rospy.Subscriber("/odometry", Odometry, ...)

#### Outputs
- rospy.Publisher("/waypoints", WaypointsArray, ...)

#### Outputs (Visuals)
- rospy.Publisher("/visual/tree_marker_array", MarkerArray, ...)
- rospy.Publisher("/visual/best_tree_branch", Marker, ...)
- rospy.Publisher("/visual/filtered_tree_branch", Marker, ...)
- rospy.Publisher("/visual/delaunay_lines", Marker, ...)
- rospy.Publisher("/visual/waypoints", MarkerArray, ...)

#### Parameters to tune (main)
- `odom_topic` = `/odometry` - The topic to get the odometry information from
- `world_frame` = `world` - The world frame to use for the visualizations
- planDistance = 12 m - Maximum length of tree branch
- expandDistance = 1 m - Length of tree node/step
- expandAngle = 20 deg - constraining angle for next tree nodes
- coneObstacleSize = 1.1 m - size of obstacles derived from cone position
- coneTargetsDistRatio = 0.5 - ratio to frontConesDist for deriving remote cone goals

## Licence

### MIT
- Use me and my code in any way you want
- Keep the names and the same licence
