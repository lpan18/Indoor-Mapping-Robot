//
// Created by rakesh on 27/08/18.
//

#include <icp_slam/mapper.h>
#include <icp_slam/utils.h>

namespace icp_slam
{

Mapper::Mapper() {}

/**
   *
   * @param width width of map in meters
   * @param height height of map in meters
   * @param resolution meters/pixel in map
   * @param unknown_cost_value value to initialize the map with (the map is unknown at the beginning)
   */
void Mapper::initMap(int width, int height, float resolution,
                     double origin_x_meters, double origin_y_meters,
                     uint8_t *pointer, unsigned char unknown_cost_value)
{
    std::cout << "Map initialized" << std::endl;
    cv::Mat map(width, height, CV_8SC1, unknown_cost_value);
    cv::Mat relative_map(width, height, CV_32S, -1);
    map_ = map;
    relative_map_ = relative_map;

    width_ = width; // passed in from node as pixel, 600
    height_ = height;
    resolution_ = resolution; ///< @brief meters/pixelsï¼Œ 0.05

    origin_x_ = origin_x_meters; ///< origin of the map in meters, double
    origin_y_ = origin_y_meters; ///< origin of the map in meters

    robot_pose_.x = 0;
    robot_pose_.y = 0;
    robot_pose_.a = 0;

    prev_a = 0;

    is_first_input_ = true;
}

cv::Mat Mapper::getMapCopy()
{
    int hasUpdateGrid = updateGridMap();
    return map_;
}

// Update the Grid Map @return: 1 - success, 0 - failure
int Mapper::updateGridMap()
{
    boost::unique_lock<boost::recursive_mutex> scoped_lock(mutex_);

    for (size_t j = 0; j < map_.cols; j++)
    {
        for (size_t i = 0; i < map_.rows; i++)
        {
            int vote = relative_map_.at<int>(i, j);
            if (vote == -1)
            {
                map_.at<int8_t>(i, j) = NO_INFORMATION;
            }
            else if (vote < 100)
            {
                map_.at<int8_t>(i, j) = FREE_SPACE;
            }
            else
            {
                map_.at<int8_t>(i, j) = LETHAL_OBSTACLE;
            }
        }
    }
    return true;
}

// Update the Relative Map  @return: 1 - success, 0 - failure
int Mapper::updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan, // the scan data
                      const tf::StampedTransform &pose)                 // where the robot is now
{
    boost::unique_lock<boost::recursive_mutex> scoped_lock(mutex_);

    robot_pose_t new_pose = poseFromTf(pose);

    if (isFirstInput())
    {
        is_first_input_ = false;
        robot_pose_.x = new_pose.x;
        robot_pose_.y = new_pose.y;
        robot_pose_.a = new_pose.a;
        std::cout << "First time, expecting 0 for x, y, (" << new_pose.x << " " << new_pose.y << " " << new_pose.a << ")\n";
    }

    int hasUpdateLaserScan = updateLaserScan(laser_scan, new_pose);

    robot_pose_.x = new_pose.x;
    robot_pose_.y = new_pose.y;
    robot_pose_.a = new_pose.a;

    // printMat(relative_map_);
    return true;
}

// @return: 1 - success, 0 - failure
int Mapper::updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan, Mapper::robot_pose_t robot_pose)
{
    int grid_x;
    int grid_y;
    convertToGridCoords(robot_pose.x, robot_pose.y, grid_x, grid_y);

    // Update occupancy.
    auto laser_theta_min = laser_scan->angle_min;
    auto laser_theta_max = laser_scan->angle_max;
    auto laser_increment = laser_scan->angle_increment;
    auto laser_r_min = laser_scan->range_min;
    auto laser_r_max = laser_scan->range_max;
    auto laser_ranges = laser_scan->ranges;
    auto num_of_scans = laser_ranges.size();

    for (size_t i = 0; i < num_of_scans; ++i)
    {
        const double theta = robot_pose.a + laser_theta_min + i * laser_increment;

        bool whetherHasObstable;
        float endpoint_x;
        float endpoint_y;
        auto curr_range = laser_ranges[i];
        // Get end point loc in the world
        if (curr_range > laser_r_min && curr_range <= laser_r_max)
        {
            whetherHasObstable = true;
            utils::polarToCartesian(curr_range, theta, endpoint_x, endpoint_y);
        }
        else
        {
            whetherHasObstable = false;
            utils::polarToCartesian(laser_r_max, theta, endpoint_x, endpoint_y);
        }
        // end loc from world to grid
        int grid_x_end;
        int grid_y_end;
        convertToGridCoords(robot_pose.x + endpoint_x, robot_pose.y + endpoint_y, grid_x_end, grid_y_end);
        int hasDrawn = drawScanLine(grid_x, grid_y, grid_x_end, grid_y_end, whetherHasObstable);
    }

    return true;
}

// @return: 1 - success, 0 - failure
int Mapper::drawScanLine(int x1, int y1, int x2, int y2, bool whetherHasObstable)
{
    boost::unique_lock<boost::recursive_mutex> scoped_lock(mutex_);

    cv::LineIterator it(map_,
                        cv::Point(x1, y1),
                        cv::Point(x2, y2));

    for (int i = 0; i < it.count; i++, ++it)
    {
        cv::Point point = it.pos(); // (point.x, point.y)
        if (whetherHasObstable && i == it.count - 1)
        {
            if (relative_map_.at<int>(point.y, point.x) == -1)
            {
                relative_map_.at<int>(point.y, point.x) = 10;
            }
            else
            {
                relative_map_.at<int>(point.y, point.x) = relative_map_.at<int>(point.y, point.x) + 10;
            }
        }
        else
        {
            if (relative_map_.at<int>(point.y, point.x) == -1)
            {
                relative_map_.at<int>(point.y, point.x) = 0;
            }
            else
            {
                relative_map_.at<int>(point.y, point.x) = relative_map_.at<int>(point.y, point.x) + 0;
            }
        }
    }
    return it.count > 0;
}

void Mapper::convertToGridCoords(double x, double y, int &grid_x, int &grid_y)
{
    boost::unique_lock<boost::recursive_mutex> scoped_lock(mutex_);

    double x_min = 0 - width_ * resolution_ / 2.0; // -15
    double y_max = height_ * resolution_ / 2.0;    // 15
    grid_x = (int)((x - x_min) / resolution_);
    grid_y = (int)((y_max - y) / resolution_);
}

void Mapper::convertToWorldCoords(int grid_x, int grid_y, double &x, double &y)
{
    boost::unique_lock<boost::recursive_mutex> scoped_lock(mutex_);

    double x_min = 0 - width_ * resolution_ / 2.0; // -15
    double y_max = height_ * resolution_ / 2.0;    // 15
    x = (double)(grid_x * resolution_ + x_min);
    y = (double)(y_max - grid_y * resolution_);
}

Mapper::robot_pose_t Mapper::poseFromGeometryPoseMsg(const geometry_msgs::Pose &pose_msg)
{
}

Mapper::robot_pose_t Mapper::poseFromTf(const tf::StampedTransform &tf_pose)
{
    Mapper::robot_pose_t pose_delta = {
        tf_pose.getOrigin().getX(), // world coordinate
        tf_pose.getOrigin().getY(),
        tf::getYaw(tf_pose.getRotation()), // in radian
    };
    return pose_delta;
}

} // namespace icp_slam
