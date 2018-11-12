# Introduction
CMPT-742 (Visual Computing) Course Assignment: ICP SLAM

========================================

## How to run the repo
`catkin_make`
`roscore`
`rosrun stage_ros stageros `rospack find safe_teleop`/worlds/willow-erratic.world`
for simple world, has base_scan_0 and base_scan_1
`rostopic echo /base_scan_0 --noarr`
`rosrun stage_ros stageros `rospack find safe_teleop`/worlds/simple.world base_scan_1:=/base_scan`
`rosrun icp_slam icp_slam_node` or `rosrun icp_slam icp_slam_node _odom_frame="odom1`
`rosrun safe_teleop safe_teleop_node`

## How to use new 
`catkin_make --force-cmake`

## How to debug the repo
run `catkin_make -DCMAKE_BUILD_TYPE=Debug`
run `rosrun --prefix "gdb -ex run --args" safe_teleop safe_teleop_node`

----------------------------------------

### in laserScanToPointMat, creating cv matrix - m * x, where m is the number of points - point cloud
for r and theta
x = r * cos(theta)
y = r * sin(theta)
For closestPoints method, laser scan to openCV matrix

### in transformToMatrix
R is 2 * 2, t is 2 * 1, transform to
R00, R01, t0
R10, R11, t1
0,   0,   1

C++: T.rowRange(0, 2).colRange(0, 2)