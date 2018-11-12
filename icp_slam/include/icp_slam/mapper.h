/** @file mapper.h */
//
// Created by rakesh on 27/08/18.
//

#ifndef ICP_SLAM_MAPPER_H
#define ICP_SLAM_MAPPER_H

// stdlib includes
#include <map>

// misc includes
#include <boost/thread/recursive_mutex.hpp>
#include <boost/atomic.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include <omp.h>
#include <atomic>

namespace icp_slam
{

// cost values for different occupancy tyes
static const unsigned char NO_INFORMATION = -1;
static const unsigned char FREE_SPACE = 0;
static const unsigned char LETHAL_OBSTACLE = 100;

class Mapper
{
public:
  typedef struct
  {
    double x; // world coordinate
    double y;
    double a; // in radian
  } robot_pose_t;

  typedef boost::recursive_mutex mutex_t;

  Mapper();

  cv::Mat getMapCopy();

  int getWidth() { return width_; }

  int getHeight() { return height_; }

  bool isFirstInput() { return is_first_input_; }

  mutex_t &getMutex() { return mutex_; }

  /**
   *
   * @param width width of map in meters
   * @param height height of map in meters
   * @param resolution meters/pixel in map
   * @param unknown_cost_value value to initialize the map with (the map is unknown at the beginning)
   */
  void initMap(int width, int height, float resolution,
               double origin_x_meters, double origin_y_meters,
               uint8_t *pointer = nullptr, unsigned char unknown_cost_value = NO_INFORMATION);

  /**
   * 
   * Update the Grid Map
   * @return: 1 - success, 0 - failure
   */
  int updateGridMap();

  /**
   * 
   * Update the Relative Map
   * @return: 1 - success, 0 - failure
   */
  int updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                const tf::StampedTransform &pose);

  // @return: 1 - success, 0 - failure
  int updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan, robot_pose_t robot_pose);

  // @return: 1 - success, 0 - failure
  int drawScanLine(int x1, int y1, int x2, int y2, bool whetherHasObstable);

  robot_pose_t getRobotPose() { return robot_pose_; }

  // utilities
  /**
   * @brief convert coords from continuous world coordinate to discrete image coord
   */
  void convertToGridCoords(double x, double y, int &grid_x, int &grid_y);

  /**
   * @brief convert coords from continuous world coordinate to discrete image coord
   */
  void convertToWorldCoords(int grid_x, int grid_y, double &x, double &y);

  static robot_pose_t poseFromGeometryPoseMsg(const geometry_msgs::Pose &pose_msg);
  static robot_pose_t poseFromTf(const tf::StampedTransform &tf_pose);

  /**
   * @brief method to print a mat
   */
  inline void printMat(cv::Mat map)
  {
    std::cout << "====Map===" << std::endl;
    std::cout << map << std::endl;
    std::cout << "==========" << std::endl;
  }

protected:
  mutex_t mutex_;
  bool is_first_input_;

  cv::Mat map_;
  cv::Mat relative_map_;

  int width_;
  int height_;
  float resolution_; ///< @brief meters/pixels

  double origin_x_; ///< origin of the map in meters
  double origin_y_; ///< origin of the map in meters

  robot_pose_t robot_pose_;

  double prev_a;
};
} // namespace icp_slam

#endif //ICP_SLAM_MAPPER_H
