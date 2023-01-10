#ifndef DWA_ROS2
#define DWA_ROS

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace dwa_ros2
{
class DWA : public rclcpp::Node
{
public:
  DWA();
  
private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
};

} // namespace dwa_ros2

#endif
