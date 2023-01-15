#ifndef DWA_ROS2
#define DWA_ROS

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "data_struct.hpp"


namespace dwa_ros2
{
class DWA : public rclcpp::Node
{
public:
  DWA(const rclcpp::NodeOptions & options);
  void obstacleCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
  //制御パラメータ
  Parameter param_;
  //現在状態変数
  State current_state_;
  //ゴール位置
  std::pair<double, double> goal_;
  //障害物位置
  std::vector<std::pair<double, double>> obstacles_;
  std::string frame_id_;
  //パラメータ設定
  void initParameter(void);
  //パラメータ更新
  void updateParameter(void);

  /*軌道予測関数
    引数：v : x軸速度[m/s] omega : 回転速度[rad/s]
    戻り値 : 軌道
  */
  std::vector<State> predictTrajectory(double v, double omega);

  /*ゴールコスト計算
    引数：trajectry : 予測軌跡
    戻り値 : コスト
  */
  double calcGoalCost(std::vector<State> trajectory);

  /*障害物コスト計算
    引数：trajectry : 予測軌跡
    戻り値 : コスト
  */
  double calcObstacleCost(std::vector<State> trajectory);

  /*速度コスト計算
    引数：trajectry : 予測軌跡
    戻り値 : コスト
  */
  double calcSpeedCost(std::vector<State> trajectory);

  //制御出力
  void publishTwist(double v, double omega);

  void publishCurrentPose(void);

  void publishPath(std::vector<State>& trajectory);

  bool isArrivedAtGoal(void);

  //DWA実行
  void dwaControl(void);
  //Dynamic Window計算
  std::vector<double> calcDynamicWindow(void);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
};

} // namespace dwa_ros2

#endif
