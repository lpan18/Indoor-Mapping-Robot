## Dependencies
* ROS
* ncurses
```
sudo apt-get install libncurses5 libncurses5-dev
```

run `roscore`
run `rosrun safe_teleop safe_teleop_node`
run `rosrun stage_ros stageros `rospack find safe_teleop`/worlds/willow-erratic.world base_scan:=/scan`

To compile, run `catkin_make`


remote PC
run `roscore`
turtlebot
run `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
remote PC
run `rosrun safe_teleop safe_teleop_node`

debugging
run `catkin_make -DCMAKE_BUILD_TYPE=Debug`
run `rosrun --prefix "gdb -ex run --args" safe_teleop safe_teleop_node`
