#include "rclcpp/rclcpp.hpp"
#include "dwa_ros2.hpp"
using std::placeholders::_1;

namespace dwa_ros2
{

DWA::DWA(const rclcpp::NodeOptions & options) : Node("dwa_ros2", options)
{
    occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10, std::bind(&DWA::obstacleCallback, this, _1));
    //odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(10), callback);
}

void DWA::obstacleCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr _msg)
{

}



} // namespace dwa_ros2

// Components
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dwa_ros2::DWA)