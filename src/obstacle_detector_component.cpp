#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

namespace dwa_ros2
{

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector(const rclcpp::NodeOptions & options);
  ~ObstacleDetector();
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
  laser_geometry::LaserProjection projector_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

};

ObstacleDetector::ObstacleDetector(const rclcpp::NodeOptions & options)
: Node("obstacle_detector_node", options)
{
  using std::placeholders::_1;
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ObstacleDetector::laserCallback, this, _1));
  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_points", 10);
}

ObstacleDetector::~ObstacleDetector()
{
}

void ObstacleDetector::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0], _msg->ranges[100]);
    sensor_msgs::msg::PointCloud2 cloud;
    projector_.projectLaser(*_msg, cloud);
    point_cloud_pub_->publish(std::move(cloud));
}

}  // namespace dwa_ros2

// Components
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dwa_ros2::ObstacleDetector)