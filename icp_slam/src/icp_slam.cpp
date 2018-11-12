//
// Created by rakesh on 13/08/18.
//
#include <cmath>
#include <map>
#include <numeric>
#include <chrono>
#include <string>

#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>
#include <icp_slam/config.h>

#define TIME_DIFF(tic, toc) ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

ICPSlam::ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time)
    : max_keyframes_distance_(max_keyframes_distance),
      max_keyframes_angle_(max_keyframes_angle),
      max_keyframes_time_(max_keyframes_time),
      last_kf_laser_scan_(new sensor_msgs::LaserScan()),
      is_tracker_running_(false)
{
  last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
}

bool ICPSlam::track(const sensor_msgs::LaserScanConstPtr &laser_scan,
                    const tf::StampedTransform &current_frame_tf_odom_laser,
                    tf::StampedTransform &tf_map_laser)
{

  if (is_tracker_running_)
  {
    ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running");
    return false;
  }

  // TODO: find the pose of laser in map frame
  // if a new keyframe is created, run ICP
  // if not a keyframe, obtain the laser pose in map frame based on odometry update
  is_tracker_running_ = true;

  if (last_kf_tf_odom_laser_.frame_id_ == "")
  {
    tf::Transform origin_tf;
    origin_tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    origin_tf.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_map_laser.setData(origin_tf);

    *last_kf_laser_scan_ = *laser_scan;
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    last_kf_tf_map_laser_ = tf_map_laser;

    is_tracker_running_ = false;
    return true;
  }

  // TODO: find the pose of laser in map frame
  bool whetherCreate = isCreateKeyframe(current_frame_tf_odom_laser, last_kf_tf_odom_laser_);
  if (whetherCreate) // if a new keyframe is created, run ICP
  {
    tf::Transform T_2_1 = last_kf_tf_odom_laser_.inverse() * current_frame_tf_odom_laser; // 1 to 2 || last to current
    tf::Transform T_scan2_scan1 = icpRegistration(laser_scan, last_kf_laser_scan_, T_2_1);
    tf_map_laser.setData(last_kf_tf_map_laser_ * T_scan2_scan1);

    // TODO: update last keyframe
    *last_kf_laser_scan_ = *laser_scan;
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    last_kf_tf_map_laser_ = tf_map_laser;
  }
  else // if not a keyframe, obtain the laser pose in map frame based on odometry update
  {
    tf::Transform T_2_1 = last_kf_tf_odom_laser_.inverse() * current_frame_tf_odom_laser; // 1 to 2 || last to current
    tf_map_laser.setData(last_kf_tf_map_laser_ * T_2_1);
  }

  is_tracker_running_ = false;

  return whetherCreate;
}

bool ICPSlam::isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const
{
  assert(current_frame_tf.frame_id_ == last_kf_tf.frame_id_);
  assert(current_frame_tf.child_frame_id_ == last_kf_tf.child_frame_id_);

  // TODO: check whether you want to create keyframe (based on max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_)
  auto time_diff = current_frame_tf.stamp_.toSec() - last_kf_tf.stamp_.toSec();
  bool checkTime = time_diff >= max_keyframes_time_;
  // ROS_INFO("In isCreateKeyframe - time diff - (%f, %f, %f, %f, %d)", time_diff, max_keyframes_time_, checkTime);

  auto angle_diff = abs(tf::getYaw(current_frame_tf.getRotation()) - tf::getYaw(last_kf_tf.getRotation()));
  bool checkAngle = angle_diff >= max_keyframes_angle_;
  // ROS_INFO("In isCreateKeyframe - angle diff - (%f, %f, %d)", angle_diff, max_keyframes_angle_, checkAngle);

  auto x_diff = current_frame_tf.getOrigin().getX() - last_kf_tf.getOrigin().getX();
  auto y_diff = current_frame_tf.getOrigin().getY() - last_kf_tf.getOrigin().getY();
  auto dist_diff = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
  bool checkDist = dist_diff >= max_keyframes_distance_;
  // ROS_INFO("In isCreateKeyframe - dist diff - (%f, %f, %d)", dist_diff, max_keyframes_distance_, checkDist);

  return checkTime || checkAngle || checkDist;
}

/**
 * TODO:
 * @param laser_scan1
 * @param laser_scan2
 * @param T_2_1 estimated transform from scan1 to scan2 (T_scan2_scan1)
 * @return refined T_scan2_scan1
 */
tf::Transform ICPSlam::icpRegistration(const sensor_msgs::LaserScanConstPtr &laser_scan1,
                                       const sensor_msgs::LaserScanConstPtr &laser_scan2,
                                       const tf::Transform &T_2_1)
{ // all three are in size of 128 * 2
  cv::Mat point_mat1 = utils::laserScanToPointMat(laser_scan1);
  cv::Mat point_mat2 = utils::laserScanToPointMat(laser_scan2);

  float lastDist = std::numeric_limits<float>::max();
  std::vector<int> closest_indices = std::vector<int>(point_mat1.rows, -1);
  std::vector<float> closest_distances_2 = std::vector<float>(point_mat1.rows, -1);

  tf::Transform T_scan2_scan1 = T_2_1;

  for (size_t itr = 0; itr < 100; itr++) // use maximum iteration = 100
  {
    // find correspondance
    closest_indices.clear();
    closest_distances_2.clear();

    cv::Mat trans_point_mat1 = utils::transformPointMat(T_scan2_scan1, point_mat1);

    closestPoints(trans_point_mat1, point_mat2, closest_indices, closest_distances_2);

    float dist = cv::mean(closest_distances_2)[0];

    // ROS_INFO("lastDist: %f, dist: %f", lastDist, dist);

    if (dist == 0) {
      return T_scan2_scan1;
    }

    if (lastDist - dist < 0.00001)
    {
      // std::cout << "=====break at itr==" << itr << std::endl;
      break;
    }

    lastDist = dist;

    // Calculate dist_threshold
    float mean;
    float std_dev;

    utils::meanAndStdDev(closest_distances_2, mean, std_dev);

    float dist_threshold = mean + 2 * std_dev;
    // count how many over threshold
    int count_saved = 0;
    for (int i = 0; i < closest_distances_2.size(); i++)
    {
      if (closest_distances_2[i] < dist_threshold)
      {
        count_saved++;
      }
    }
    // Redefine points
    cv::Mat point_mat1_match(count_saved, 2, CV_32F, cv::Scalar(1.0f));
    cv::Mat point_mat2_match(count_saved, 2, CV_32F, cv::Scalar(1.0f));

    int count_saving = 0;
    for (int i = 0; i < closest_distances_2.size(); i++)
    {
      if (closest_distances_2[i] < dist_threshold)
      {
        point_mat1_match.at<float>(count_saving, 0) = point_mat1.at<float>(i, 0);
        point_mat1_match.at<float>(count_saving, 1) = point_mat1.at<float>(i, 1);
        point_mat2_match.at<float>(count_saving, 0) = point_mat2.at<float>(closest_indices[i], 0);
        point_mat2_match.at<float>(count_saving, 1) = point_mat2.at<float>(closest_indices[i], 1);
        // according to indices, match 1 and 2
        count_saving++;
      }
    }

    T_scan2_scan1 = icpIteration(point_mat1_match, point_mat2_match);
    // vizClosestPoints(point_mat1_match, point_mat2_match, T_scan2_scan1, std::to_string(itr));
  }
  return T_scan2_scan1;
}

/**
 * TODO:
 * @param point_mat1 nx2 matrix with points of laser scan 1 with a match in laser scan 2
 * @param point_mat2 nx2 matrix with points of laser scan 2 with a match in laser scan 1
 * @return estimated transform from scan1 to scan2 (T_scan2_scan1) that minimizes least square alignment error
 */
tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1,
                                    cv::Mat &point_mat2)
{
  point_mat1 = point_mat1.reshape(2);
  point_mat2 = point_mat2.reshape(2);

  cv::Scalar point_mat1_bar = mean(point_mat1);
  cv::Scalar point_mat2_bar = mean(point_mat2);

  cv::Mat point_mat1_mod = point_mat1 - point_mat1_bar;
  cv::Mat point_mat2_mod = point_mat2 - point_mat2_bar;

  point_mat1_mod = point_mat1_mod.reshape(1);
  point_mat2_mod = point_mat2_mod.reshape(1);

  cv::Mat W(2, 2, CV_32FC1);
  W = point_mat2_mod.t() * point_mat1_mod;

  cv::SVD svd(W);
  cv::Mat R = svd.vt * svd.u;
  float *_R = R.ptr<float>(0);

  cv::Scalar T;
  T[0] = point_mat2_bar[0] - (_R[0] * point_mat1_bar[0] + _R[1] * point_mat1_bar[1]);
  T[1] = point_mat2_bar[1] - (_R[2] * point_mat1_bar[0] + _R[3] * point_mat1_bar[1]);

  tf::Transform T_scan2_scan1;
  tf::Matrix3x3 rotation_matrix_3d(R.at<float>(0, 0), R.at<float>(0, 1), 0,
                                   R.at<float>(1, 0), R.at<float>(1, 1), 0,
                                   0, 0, 1);
  T_scan2_scan1.setOrigin(tf::Vector3(T[0], T[1], 0.0));
  T_scan2_scan1.setBasis(rotation_matrix_3d);
  
  point_mat1 = point_mat1.reshape(1);
  point_mat2 = point_mat2.reshape(1);

  return T_scan2_scan1;
}

void ICPSlam::closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2)
{
  // uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
  closest_indices = std::vector<int>(point_mat1.rows, -1);
  closest_distances_2 = std::vector<float>(point_mat1.rows, -1);

  cv::Mat multi_channeled_mat1;
  cv::Mat multi_channeled_mat2;

  point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
  point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

  cv::flann::Index flann_index(multi_channeled_mat2, cv::flann::KDTreeIndexParams(2)); // using 2 randomized kdtrees

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
  cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
  flann_index.knnSearch(multi_channeled_mat1, mat_indices, mat_dists, 1, cv::flann::SearchParams(64));

  int *indices_ptr = mat_indices.ptr<int>(0);
  //float* dists_ptr = mat_dists.ptr<float>(0);
  for (int i = 0; i < mat_indices.rows; ++i)
  {
    closest_indices[i] = indices_ptr[i];
  }

  closest_distances_2.resize(mat_dists.rows); // added
  mat_dists.copyTo(cv::Mat(closest_distances_2));

  // ---------------------------- naive version ---------------------------- //
  // max allowed distance between corresponding points
  //  const float max_distance = 0.5;
  //
  //  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  //  {
  //    int closest_point_idx = -1;
  //    float closest_distance_2 = std::pow(max_distance, 2.0f);
  //
  //    for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
  //    {
  //      auto distance2 =
  //        std::pow(point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f)
  //        + std::pow(point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
  //
  //      if (distance2 < closest_distance_2)
  //      {
  //        closest_distance_2 = distance2;
  //        closest_point_idx = (int)j;
  //      }
  //    }
  //
  //    if (closest_point_idx >= 0)
  //    {
  //      closest_indices[i] = closest_point_idx;
  //      closest_distances_2[i] = closest_distance_2;
  //    }
  //  }
}

void ICPSlam::vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1,
                               std::string count)
{
  assert(point_mat1.size == point_mat2.size);

  const float resolution = 0.005;

  float *float_array = (float *)(point_mat1.data);
  float size_m = std::accumulate(
      float_array, float_array + point_mat1.total(), std::numeric_limits<float>::min(),
      [](float max, float current) {
        return current > max ? current : max;
      });
  // add some slack
  size_m += 0.5;

  int size_pix = (int)(size_m / resolution);

  cv::Mat img(
      size_pix,
      size_pix,
      CV_8UC3,
      cv::Scalar(0, 0, 0));

  auto meters_to_pix = [&size_pix, resolution](float meters) {
    int pix = (int)(meters / resolution + size_pix / 2.0f);
    pix = std::max(0, pix);
    pix = std::min(size_pix - 1, pix);
    return pix;
  };

  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1.inverse(), point_mat2);

  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  {
    float x1 = point_mat1.at<float>(i, 0);
    float y1 = point_mat1.at<float>(i, 1);
    float x2 = point_mat2.at<float>(i, 0);
    float y2 = point_mat2.at<float>(i, 1);
    float x3 = transformed_point_mat2.at<float>(i, 0);
    float y3 = transformed_point_mat2.at<float>(i, 1);

    auto pix_x1 = meters_to_pix(x1);
    auto pix_y1 = meters_to_pix(y1);
    auto pix_x2 = meters_to_pix(x2);
    auto pix_y2 = meters_to_pix(y2);
    auto pix_x3 = meters_to_pix(x3);
    auto pix_y3 = meters_to_pix(y3);

    cv::Point point1(pix_x1, pix_y1);
    cv::Point point2(pix_x2, pix_y2);
    cv::Point point3(pix_x3, pix_y3);

    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1); // red point could 1
    cv::circle(img, point2, 5, cv::Scalar(0, 255, 0), -1); // green point cloud 2 brfore transform
    cv::circle(img, point3, 5, cv::Scalar(255, 0, 0), -1); //blue  point cloud 2 after transform

    cv::line(img, point1, point3, cv::Scalar(255, 255, 255), 2);
  }

  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  cv::imwrite("/home/jayleenz/cmpt742-a3/tmp/icp_laser_" + count + ".png", img);
}

} // namespace icp_slam
