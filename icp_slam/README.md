## How to run the repo
`catkin_make`
`roscore`
`rosrun stage_ros stageros `rospack find safe_teleop`/worlds/willow-erratic.world`
`rosrun stage_ros stageros `rospack find safe_teleop`/worlds/simple.world base_scan_1:=/base_scan`
`rosrun icp_slam icp_slam_node`
`rosrun safe_teleop safe_teleop_node`
