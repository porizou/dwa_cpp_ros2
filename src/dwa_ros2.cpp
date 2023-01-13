#include "rclcpp/rclcpp.hpp"
#include "dwa_ros2.hpp"
using std::placeholders::_1;

namespace dwa_ros2
{

DWA::DWA(const rclcpp::NodeOptions & options) : Node("dwa_ros2", options)
{
    occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10, std::bind(&DWA::obstacleCallback, this, _1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&DWA::odomCallback, this, _1));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Initialize params
    updateParameter();
    // Initialize current state
    current_state_ = {0.0, 0.0, 0.0, 0.0, 0.0};

    goal_ = std::make_pair(3.0, 5.0);
}

void DWA::obstacleCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Clear the previous obstacles
    obstacles_.clear();

    // Get the resolution and the origin of the map
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;    // Iterate through the map data
    for (int i = 0; i < msg->data.size(); i++)
    {
        // Get the cell value
        int cell_value = msg->data[i];

        // Check if the cell is occupied
        if (cell_value == 100)
        {
            // Calculate the x and y coordinates of the cell
            int x = i % msg->info.width;
            int y = i / msg->info.width;
            double world_x = (x * resolution) + origin_x;
            double world_y = (y * resolution) + origin_y;

            // Add the obstacle to the list
            obstacles_.push_back(std::make_pair(world_x, world_y));
        }
    }
}

void DWA::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Update position
    current_state_.x = msg->pose.pose.position.x;
    current_state_.y = msg->pose.pose.position.y;

    // Update velocity
    current_state_.v = msg->twist.twist.linear.x;
    current_state_.omega = msg->twist.twist.angular.z;

    // Convert quaternion to euler
    geometry_msgs::msg::Quaternion quat = msg->pose.pose.orientation;
    double roll, pitch, yaw = 0.0;
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    current_state_.theta = yaw;
}

void DWA::updateParameter(void) 
{
    // Declare max speed parameter
    param_.max_speed = this->declare_parameter("max_speed", 0.0);

    // Declare min speed parameter
    param_.min_speed = this->declare_parameter("min_speed", 0.0);

    // Declare max omega parameter
    param_.max_omega = this->declare_parameter("max_omega", 0.0);

    // Declare max accel parameter
    param_.max_accel = this->declare_parameter("max_accel", 0.0);

    // Declare max accel omega parameter
    param_.max_accel_omega = this->declare_parameter("max_accel_omega", 0.0);

    // Declare v resolution parameter
    param_.v_resolution = this->declare_parameter("v_resolution", 0.0);

    // Declare omega resolution parameter
    param_.omega_resolution = this->declare_parameter("omega_resolution", 0.0);

    // Declare predict time parameter
    param_.predict_time = this->declare_parameter("predict_time", 0.0);

    // Declare dt parameter
    param_.dt = this->declare_parameter("dt", 0.0);

    // Declare goal cost gain parameter
    param_.goal_cost_gain = this->declare_parameter("goal_cost_gain", 0.0);

    // Declare speed cost gain parameter
    param_.speed_cost_gain = this->declare_parameter("speed_cost_gain", 0.0);

    // Declare obstacle cost gain parameter
    param_.obstacle_cost_gain = this->declare_parameter("obstacle_cost_gain", 0.0);

    // Declare robot radius parameter
    param_.robot_radius = this->declare_parameter("robot_radius", 0.0);
}

std::vector<State> DWA::predictTrajectory(double v, double omega)
{
    // Create an empty list to store the predicted states
    std::vector<State> predicted_states;

    // Get the current state
    State current_state = current_state_;

    // Set the initial state
    predicted_states.push_back(current_state);

    // Get the predict time and the time step
    double predict_time = param_.predict_time;
    double dt = param_.dt;

    // Set the time step
    double time = dt;

    // Iterate through the predict time
    while (time < predict_time)
    {
        // Update the state
        current_state.x += current_state.v * cos(current_state.theta) * dt;
        current_state.y += current_state.v * sin(current_state.theta) * dt;
        current_state.theta += current_state.omega * dt;
        current_state.v = v;
        current_state.omega = omega;

        // Add the state to the list
        predicted_states.push_back(current_state);

        // Update the time
        time += dt;
    }

    // Return the list of predicted states
    return predicted_states;
}

double DWA::calcGoalCost(std::vector<State> trajectory)
{
}  

double DWA::calcObstacleCost(std::vector<State> trajectory)
{
    double cost = 0.0;
    double min_distance = std::numeric_limits<double>::max();

    for (auto &state : trajectory)
    {
        for (auto &obstacle : obstacles_)
        {
            // Calculate the distance between the state and the obstacle
            double distance = std::sqrt(std::pow(state.x - obstacle.first, 2) + std::pow(state.y - obstacle.second, 2));

            // Check if the distance is lower than the minimum distance
            if (distance < min_distance)
            {
                min_distance = distance;
            }
        }
    }
    // Calculate the obstacle cost
    if (min_distance <= param_.robot_radius)
    {
        cost = 1.0 - min_distance / param_.robot_radius;
    }
    else
    {
        cost = 0.0;
    }
    return cost;
}

double DWA::calcSpeedCost(std::vector<State> trajectory)
{
}

void DWA::publishTwist(double v, double omega)
{
    auto twist = std::make_unique<geometry_msgs::msg::Twist>();
    twist->linear.x = v;
    twist->angular.z = omega;
    twist_pub_->publish(std::move(twist));
}

void DWA::dwaControl(void)
{
    // Get the dynamic window
    std::vector<double> dw = calcDynamicWindow();

    // Set the initial cost and best trajectory
    double best_cost = std::numeric_limits<double>::lowest();
    std::vector<State> best_trajectory;

    // Iterate through the possible linear and angular velocities
    for (double v = dw[0]; v <= dw[1]; v += param_.v_resolution)
    {
        for (double omega = dw[2]; omega <= dw[3]; omega += param_.omega_resolution)
        {
            // Predict the trajectory
            std::vector<State> trajectory = predictTrajectory(v, omega);

            //Calculate the cost of the trajectory
            double goal_cost = calcGoalCost(trajectory);
            double speed_cost = calcSpeedCost(trajectory);
            double obstacle_cost = calcObstacleCost(trajectory);

            double cost = param_.goal_cost_gain * goal_cost + param_.speed_cost_gain * speed_cost + param_.obstacle_cost_gain * obstacle_cost;

            // Check if the cost is lower than the current best cost
            if (cost > best_cost)
            {
                // Update the best cost and trajectory
                best_cost = cost;
                best_trajectory = trajectory;
            }
        }
    }

    // Get the final linear and angular velocities
    double v = best_trajectory.back().v;
    double omega = best_trajectory.back().omega;

    // Publish the twist message
    publishTwist(v, omega);
}

std::vector<double> DWA::calcDynamicWindow(void)
{
    // Create a vector to store the dynamic window
    std::vector<double> dw;

    // Get the current speed and omega
    double v = current_state_.v;
    double omega = current_state_.omega;

    // Get the max and min speeds
    double max_v = param_.max_speed;
    double min_v = param_.min_speed;

    // Get the max and min omegas
    double max_omega = param_.max_omega;
    double min_omega = -1 * param_.max_omega;

    // Get the max accel and max accel omega
    double max_a = param_.max_accel;
    double max_a_omega = param_.max_accel_omega;

    // Calculate the dynamic window
    double v_min = std::max(min_v, v - max_a * param_.dt);
    double v_max = std::min(max_v, v + max_a * param_.dt);
    double omega_min = std::max(min_omega, omega - max_a_omega * param_.dt);
    double omega_max = std::min(max_omega, omega + max_a_omega * param_.dt);

    // Add the values to the dynamic window
    dw.push_back(v_min);
    dw.push_back(v_max);
    dw.push_back(omega_min);
    dw.push_back(omega_max);

    // Return the dynamic window
    return dw;
}
} // namespace dwa_ros2

// Components
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dwa_ros2::DWA)