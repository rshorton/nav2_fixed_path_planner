# Fixed Path Planner

WIP planner for learning about planners.

Based on the Nav2 tutorial nav2_straightline_planner created by Shivang Patel. 

Functionality:
* Takes as input a pre-defined set of map points via the 'waypoints' parameter of the planner.
* Determines the first point to approach based on the closest waypoint in front of the robot.
* Creates a path by straightline interpolation between each pair of points.
* Supports dynamic parameter updates to allow the waypoint list to be edited dynamically.
