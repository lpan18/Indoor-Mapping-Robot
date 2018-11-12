//
// Created by rakesh on 17/08/18.
//

#include <icp_slam/utils.h>

namespace icp_slam
{
namespace utils
{

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan)
{
  // TODO
  // angle_min--3.141593,angle_max-3.141593,angle_increment-0.049474, range_min-0.000000,range_max-30.000000
  // ROS_INFO("angle_min-%f,angle_max-%f,angle_increment-%f,range_min-%f,range_max-%f",scan->angle_min,scan->angle_max,scan->angle_increment, scan->range_min,scan->range_max);
  auto laser_theta_min = scan->angle_min;
  auto laser_theta_max = scan->angle_max;
  auto laser_increment = scan->angle_increment;
  auto laser_r_min = scan->range_min;
  auto laser_r_max = scan->range_max;
  auto laser_ranges = scan->ranges;
  auto num_of_scans = laser_ranges.size();

  cv::Mat point_mat_homogeneous(num_of_scans, 2, CV_32F, cv::Scalar(1.0f));
  float r;
  float theta;
  float x;
  float y;

  for (int i = 0; i < num_of_scans; i++)
  {
    r = laser_ranges[i];
    theta = laser_theta_min + laser_increment * i;
    x = r * std::cos(theta);
    y = r * std::sin(theta);
    point_mat_homogeneous.at<float>(i, 0) = x;
    point_mat_homogeneous.at<float>(i, 1) = y;
  }

  return point_mat_homogeneous;
}

cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  cv::Mat transformed_point_mat = T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

} // namespace utils
} // namespace icp_slam