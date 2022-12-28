#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

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
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  double resolution_;
  double offset_x_;
  double offset_y_;
};

ObstacleDetector::ObstacleDetector(const rclcpp::NodeOptions & options)
: Node("obstacle_detector_node", options)
{
  using std::placeholders::_1;
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ObstacleDetector::laserCallback, this, _1));
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
  // parameter
  resolution_ = this->declare_parameter("resolution", 0.05);
  offset_x_ = this->declare_parameter("offset_x", 0.0);
  offset_y_ = this->declare_parameter("offset_y", 0.0);
}

ObstacleDetector::~ObstacleDetector()
{
}

void ObstacleDetector::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0], _msg->ranges[100]);
    // LaserScanデータから点群を作成する
    std::vector<std::pair<double, double>> points;
    double angle = _msg->angle_min;
    for (const double range : _msg->ranges)
    {
      if (std::isfinite(range))
      {
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);
        points.emplace_back(x, y);
      }
      angle += _msg->angle_increment;
    }
    // 点群を回転させ、offsetを加える
    tf2::Quaternion quat;
    quat.setRPY(0, 0, _msg->angle_min);
    tf2::Transform transform(quat, tf2::Vector3(offset_x_, offset_y_, 0));
    for (std::pair<double, double>& point : points)
    {
      tf2::Vector3 vec(point.first, point.second, 0);
      vec = transform * vec;
      point.first = vec.x();
      point.second = vec.y();
    }

    // 点群を使用して、グリッドセルを占有するかどうかを判定する
    double x_min = std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
    for (const std::pair<double, double>& point : points)
    {
      x_min = std::min(x_min, point.first);
      y_min = std::min(y_min, point.second);
      x_max = std::max(x_max, point.first);
      y_max = std::max(y_max, point.second);
    }
    int n_x = std::ceil((x_max - x_min) / resolution_);
    int n_y = std::ceil((y_max - y_min) / resolution_);
    std::vector<int8_t> data(n_x * n_y, -1);
    for (const std::pair<double, double>& point : points)
    {
      int i = std::floor((point.second - y_min) / resolution_);
      int j = std::floor((point.first - x_min) / resolution_);
      if (i >= 0 && i < n_y && j >= 0 && j < n_x)
      {
        data[i * n_x + j] = 100;
      }
    }
}

}  // namespace dwa_ros2

// Components
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dwa_ros2::ObstacleDetector)