#include "rclcpp/rclcpp.hpp"
#include "dwa_ros2.hpp"

namespace dwa_ros2
{

DWA::DWA() : Node("dwa_ros2")
{
    // OccupancyGridトピックをSubsucribe
    auto callback =
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void
      {
        // OccupancyGridメッセージを処理するコードをここに書く
      };
    occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("occupancy_grid", rclcpp::QoS(10), callback);

    // OdometryトピックをSubsucribe
    auto callback =
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void
      {
        // Odometryメッセージを処理するコードをここに書く
      };
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(10), callback);
}

} // namespace dwa_ros2

// Components
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dwa_ros2::DWA)