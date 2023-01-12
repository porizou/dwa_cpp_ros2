#include "rclcpp/rclcpp.hpp"
#include "dwa_ros2.hpp"
using std::placeholders::_1;

namespace dwa_ros2
{

DWA::DWA(const rclcpp::NodeOptions & options) : Node("dwa_ros2", options)
{
    occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10, std::bind(&DWA::obstacleCallback, this, _1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&DWA::odomCallback, this, _1));
}

void DWA::obstacleCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr _msg)
{

}

void DWA::odomCallback(const nav_msgs::msg::Odometry::SharedPtr _msg)
{

}

void DWA::dwaInitialize(void) 
{
}

std::vector<State> DWA::predictTrajectory(double v, double omega)
{
}

double DWA::calcGoalCost(std::vector<State> trajectory)
{
}  

double DWA::calcObstacleCost(std::vector<State> trajectory)
{
}

double DWA::calcSpeedCost(std::vector<State> trajectory)
{
}

void DWA::publishTwist(double v, double omega)
{
}

void DWA::dwaControl(void)
{
}

std::vector<double> DWA::calcDynamicWindow(void)
{
}

void DWA::robotControl(double v, double omega)
{
}

} // namespace dwa_ros2

// Components
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dwa_ros2::DWA)