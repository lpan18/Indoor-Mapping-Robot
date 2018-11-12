/**
 * @file safe_teleop.cpp
 * @brief Safe teleoperation library implementation
 * Created by rakesh on 28/09/18.
 */
#include <limits>
#include <safe_teleop/safe_teleop.h>

namespace safe_teleop
{
SafeTeleop::SafeTeleop() : is_shutdown_(false),
                           max_cmd_vel_age_(1.0),
                           max_linear_vel_(1.0),
                           max_angular_vel_(1.0),
                           linear_vel_increment_(0.05),
                           angular_vel_increment_(0.05),
                           laser_safety_check_angle_(0.25),
                           min_safety_impact_time_(0.5),
                           min_safety_distance_(0.5),
                           linear_vel_(0.0),
                           linear_speed_(0.0),
                           angular_vel_(0.0),
                           angular_speed_(0.0),
                           last_command_timestamp_(0.0)
{
  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  // The subscriber callback is set to the laserScanCallback method of the instantiated object of this class
  laser_scan_sub_ = global_nh.subscribe("scan", 5, &SafeTeleop::laserScanCallback, this);

  run_thread_ = boost::thread(&SafeTeleop::run, this);
  displayCurrentSpeeds();
}

SafeTeleop::~SafeTeleop()
{
  shutdown();
  // wait for the run thread to terminate
  run_thread_.join();

  geometry_msgs::Twist zero_cmd_vel;
  zero_cmd_vel.linear.x = 0;
  zero_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_cmd_vel);
}

void SafeTeleop::run()
{
  ros::Rate r(10);
  while (ros::ok() && !is_shutdown_)
  {
    auto current_timestamp = ros::Time::now().toSec();

    auto last_cmd_vel_age = current_timestamp - last_command_timestamp_;
    geometry_msgs::Twist cmd_vel;
    if (last_cmd_vel_age > max_cmd_vel_age_)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      // ROS_WARN_THROTTLE(1.0, "Time out\r");
    }
    else
    {
      auto is_safe = checkSafety(static_cast<double>(linear_vel_));
      if(is_safe){
        cmd_vel.linear.x = linear_vel_;
      }else{
        cmd_vel.linear.x = 0;
      }
      cmd_vel.angular.z = angular_vel_;
    }
    cmd_vel_pub_.publish(cmd_vel);
    r.sleep();
  }
}

void SafeTeleop::moveForward()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  linear_vel_ = (double)linear_speed_;
  angular_vel_ = 0;
}

void SafeTeleop::moveBackward()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  linear_vel_ = -(double)linear_speed_;
  angular_vel_ = 0;
}

void SafeTeleop::rotateClockwise()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  angular_vel_ = -(double)angular_speed_;
  linear_vel_ = 0;
}

void SafeTeleop::rotateCounterClockwise()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  angular_vel_ = (double)angular_speed_;
  linear_vel_ = 0;
}

void SafeTeleop::stop()
{
  linear_vel_ = 0;
  angular_vel_ = 0;
}

void SafeTeleop::increaseLinearSpeed()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  if (linear_speed_ + 0.05 <= 1)
  {
    linear_speed_ = linear_speed_ + 0.05;
  }
  else
  {
    linear_speed_ = 1;
  }
  linear_vel_ = 0;
  angular_vel_ = 0;
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseLinearSpeed()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  if (linear_speed_ - 0.05 >= 0)
  {
    linear_speed_ = linear_speed_ - 0.05;
  }
  else
  {
    linear_speed_ = 0;
  }
  linear_vel_ = 0;
  angular_vel_ = 0;
  displayCurrentSpeeds();
}

void SafeTeleop::increaseAngularSpeed()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  if (angular_speed_ + 0.05 <= 1)
  {
    angular_speed_ = angular_speed_ + 0.05;
  }
  else
  {
    angular_speed_ = 1;
  }
  linear_vel_ = 0;
  angular_vel_ = 0;
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseAngularSpeed()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  if (angular_speed_ - 0.05 >= 0)
  {
    angular_speed_ = angular_speed_ - 0.05;
  }
  else
  {
    angular_speed_ = 0;
  }
  linear_vel_ = 0;
  angular_vel_ = 0;
  displayCurrentSpeeds();
}

bool SafeTeleop::checkSafety(double linear_vel)
{
  auto laser_scan = getLaserScan();
  // -3.141593, 3.141593, 0.049474 2.83degree - ranges.size() =128 data
  // 0, 6.28, 360data
  // ROS_INFO_THROTTLE(1.0, "print laser - %f, %f, %f", laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment);
  int delta = ceil((15.0 / 180 * M_PI) / laser_scan.angle_increment);
  int laser_num = laser_scan.ranges.size();
  bool flag;
  if(laser_scan.angle_min<0){   // use stage simulation
    flag=(linear_vel >= 0);
  }else{                        // use real turtlebot
    flag=(linear_vel <= 0);
  }
  if (flag)
  {
    for (int i = laser_num / 2 - delta; i < laser_num / 2 + delta; i++)
    {
      if (laser_scan.ranges[i] <= min_safety_distance_ && laser_scan.ranges[i]>0)
      {
        ROS_WARN_THROTTLE(1.0, "Attention! Found obstacle within %f m\r",laser_scan.ranges[i]);
        return false;
      }
    }
  }
  else
  {
    for (int i = 0; i <= delta; i++)
    {
      if (laser_scan.ranges[i] <= min_safety_distance_ && laser_scan.ranges[i]>0)
      {
        ROS_WARN_THROTTLE(1.0, "Attention! Found obstacle within %f m\r",laser_scan.ranges[i]);
        return false;
      };
    }
    for (int i = laser_num - delta - 1; i < laser_num; i++)
    {
      if (laser_scan.ranges[i] <= min_safety_distance_ && laser_scan.ranges[i]>0)
      {
        ROS_WARN_THROTTLE(1.0, "Attention! Found obstacle within %f m\r",laser_scan.ranges[i]);
        return false;
      }
    }
  }
  return true;
}

} // namespace safe_teleop
