## How to run the repo
To compile, run `catkin_make`
run `roscore`
run `rosrun stage_ros stageros `rospack find safe_teleop`/worlds/willow-erratic.world`
Or `rosrun stage_ros stageros `rospack find safe_teleop`/worlds/simple.world base_scan_1:=/base_scan`
run `rosrun icp_slam icp_slam_node`
run `rosrun safe_teleop safe_teleop_node`
