## How to run the repo
`catkin_make`
`roscore`
`rosrun stage_ros stageros `rospack find safe_teleop`/worlds/willow-erratic.world`
for simple world, has base_scan_0 and base_scan_1
`rostopic echo /base_scan_0 --noarr`
`rosrun stage_ros stageros `rospack find safe_teleop`/worlds/simple.world base_scan_1:=/base_scan`
`rosrun icp_slam icp_slam_node` or `rosrun icp_slam icp_slam_node _odom_frame="odom1`
`rosrun safe_teleop safe_teleop_node`
