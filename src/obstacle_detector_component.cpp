#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

ObstacleDetector::ObstacleDetector(const rclcpp::NodeOptions & options)
: Node("obstacle_detector_node", options)
{
  using std::placeholders::_1;
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SystemDefaultsQoS()), std::bind(&ObstacleDetector::laserCallback, this, _1));
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
  // parameter
  resolution_ = this->declare_parameter("resolution", 0.05);
  offset_x_ = this->declare_parameter("offset_x", 0.0);
  offset_y_ = this->declare_parameter("offset_y", 0.0);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

ObstacleDetector::~ObstacleDetector()
{
}

void ObstacleDetector::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0], _msg->ranges[100]);

    // Look up the transform from the laser frame to the map frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_->lookupTransform("odom", _msg->header.frame_id,  tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
      return;
    }


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

    // Transform the points into the map frame
    tf2::Transform tf_transform;
    tf2::convert(transform_stamped.transform, tf_transform);
    for (std::pair<double, double>& point : points)
    {
      tf2::Vector3 vec(point.first, point.second, 0);
      vec = tf_transform * vec;
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

    // OccupancyGridをPublishする
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = _msg->header.stamp;
    occupancy_grid.header.frame_id = "odom";
    occupancy_grid.info.resolution = resolution_;
    occupancy_grid.info.width = n_x;
    occupancy_grid.info.height = n_y;
    occupancy_grid.info.origin.position.x = x_min;
    occupancy_grid.info.origin.position.y = y_min;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.data = data;

    // OccupancyGridをpublishする
    occupancy_grid_pub_->publish(occupancy_grid);
}

}  // namespace dwa_ros2

// Components
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dwa_ros2::ObstacleDetector)
