#ifndef DWA_ROS2
#define DWA_ROS

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "data_struct.hpp"


namespace dwa_ros2
{
class DWA : public rclcpp::Node
{
public:
  DWA(const rclcpp::NodeOptions & options);
  void obstacleCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  //制御パラメータ
  Parameter param_;
  //現在状態変数
  State current_state_;
  //ゴール位置
  std::pair<double, double> goal_;
  //障害物位置
  std::vector<std::pair<double, double>> obstacles_;

  //パラメータ設定
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

  //DWA実行
  void dwaControl(void);
  //Dynamic Window計算
  std::vector<double> calcDynamicWindow(void);

    /*ロボット制御関数
    引数：v : x軸速度[m/s] omega : 回転速度[rad/s] 
    戻り値 : なし
  */
  void robotControl(double v, double omega);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

} // namespace dwa_ros2

#endif
