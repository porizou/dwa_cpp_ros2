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
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
    predicted_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("predicted_path", 10);
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(&DWA::goalCallback, this, _1));
    // Initialize params
    initParameter();
    // Initialize current state
    current_state_ = {0.0, 0.0, 0.0, 0.0, 0.0};

    goal_ = std::make_pair(0.0, -0.5);
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
    transformObstacles();
    dwaControl();
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

void DWA::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Update goal position
    goal_.first = msg->pose.position.x;
    goal_.second = msg->pose.position.y;
    RCLCPP_INFO(this->get_logger(), "Updated goal position: x = %f, y = %f", goal_.first, goal_.second);
}

void DWA::initParameter(void) 
{
    // Declare max speed parameter
    param_.max_speed = this->declare_parameter("max_speed", 0.4);

    // Declare min speed parameter
    param_.min_speed = this->declare_parameter("min_speed", -0.0);

    // Declare max omega parameter
    param_.max_omega = this->declare_parameter("max_omega", 40.0 * M_PI / 180.0);

    // Declare max accel parameter
    param_.max_accel = this->declare_parameter("max_accel", 1.0);

    // Declare max accel omega parameter
    param_.max_accel_omega = this->declare_parameter("max_accel_omega", 40.0 * M_PI / 180.0);

    // Declare v resolution parameter
    param_.v_resolution = this->declare_parameter("v_resolution", 0.01);

    // Declare omega resolution parameter
    param_.omega_resolution = this->declare_parameter("omega_resolution", 0.1 * M_PI / 180.0);

    // Declare predict time parameter
    param_.predict_time = this->declare_parameter("predict_time", 1.5);

    // Declare dt parameter
    param_.dt = this->declare_parameter("dt", 0.1);

    // Declare goal cost gain parameter
    param_.goal_cost_gain = this->declare_parameter("goal_cost_gain", 0.5);

    // Declare speed cost gain parameter
    param_.speed_cost_gain = this->declare_parameter("speed_cost_gain", 0.5);

    // Declare obstacle cost gain parameter
    param_.obstacle_cost_gain = this->declare_parameter("obstacle_cost_gain", 0.6);

    // Declare robot radius parameter
    param_.robot_radius = this->declare_parameter("robot_radius", 0.1);

    param_.goal_tolerance = this->declare_parameter("goal_tolerance", 0.2);

    frame_id_ = this->declare_parameter("frame_id_", "odom");
}


void DWA::updateParameter(void)
{
    auto update_param = [&](const std::string& name) {
        return this->get_parameter(name).get_value<double>();
    };

    auto update_param_log = [&](const std::string& name, double& variable) {
        variable = update_param(name);
        //RCLCPP_INFO(this->get_logger(), "Updated %s: %f", name.c_str(), variable);
    };

    update_param_log("max_speed", param_.max_speed);
    update_param_log("min_speed", param_.min_speed);
    update_param_log("max_omega", param_.max_omega);
    update_param_log("max_accel", param_.max_accel);
    update_param_log("max_accel_omega", param_.max_accel_omega);
    update_param_log("v_resolution", param_.v_resolution);
    update_param_log("omega_resolution", param_.omega_resolution);
    update_param_log("dt", param_.dt);    
    update_param_log("goal_cost_gain", param_.goal_cost_gain);
    update_param_log("speed_cost_gain", param_.speed_cost_gain);
    update_param_log("obstacle_cost_gain", param_.obstacle_cost_gain);
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
    double cost = 0.0;

    // Get the final state
    State final_state = trajectory[trajectory.size() - 1];

    // Calculate the distance to the goal
    double distance = std::sqrt(std::pow(final_state.x - goal_.first, 2) + std::pow(final_state.y - goal_.second, 2));

    // Calculate the goal cost
    cost = std::exp(-distance);

    return cost;
}  

double DWA::calcObstacleCost(std::vector<State> trajectory)
{
    double cost = 0.0;
    double min_distance = std::numeric_limits<double>::max();
    if(obstacles_.empty()){
        return 1.0;
    }
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
        // If the robot hits an obstacle, the cost is 0
        cost = 0.0; 
    }
    else
    {
        cost = 1.0 - min_distance / param_.robot_radius;
    }
    return cost;
}

double DWA::calcSpeedCost(std::vector<State> trajectory)
{
    double cost = 0.0;

    // Get the final state
    State final_state = trajectory[trajectory.size() - 1];

    // Calculate the speed cost
    cost = std::exp(-(std::pow(final_state.v - param_.max_speed, 2) / (2 * std::pow(param_.v_resolution, 2))));

    return cost;
}

void DWA::publishTwist(double v, double omega)
{
    // If you reach the goal, stop.
    if(isArrivedAtGoal())
    {
        v = 0.0; omega = 0.0;
    }
    //RCLCPP_INFO(this->get_logger(), "linear.x: %lf, angular.z: %lf", v, omega);
    auto twist = std::make_unique<geometry_msgs::msg::Twist>();
    twist->linear.x = v;
    twist->angular.z = omega;
    twist_pub_->publish(std::move(twist));
}

void DWA::publishCurrentPose(void)
{
    auto current_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
    current_pose->header.stamp = this->now();
    current_pose->header.frame_id = frame_id_;
    current_pose->pose.position.x = current_state_.x;
    current_pose->pose.position.y = current_state_.y;
    tf2::Quaternion q; 
    q.setRPY(0, 0, current_state_.theta); 
    current_pose->pose.orientation = tf2::toMsg(q);
    current_pose_pub_->publish(std::move(current_pose));
}

void DWA::publishPath(std::vector<State>& trajectory)
{
    auto path = std::make_unique<nav_msgs::msg::Path>();

    for (const auto& state : trajectory)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        tf2::Quaternion q; 
        q.setRPY(0, 0, state.theta);
        pose.pose.orientation = tf2::toMsg(q);
        pose.header.stamp = this->now();
        pose.header.frame_id = frame_id_;
        path->poses.push_back(pose);
    }

    path->header.frame_id = frame_id_;
    path->header.stamp = this->now();
    predicted_path_pub_->publish(std::move(path));
}

bool DWA::isArrivedAtGoal(void)
{
    double distance = std::sqrt(std::pow(current_state_.x - goal_.first, 2) + std::pow(current_state_.y - goal_.second, 2));
    return distance <= param_.goal_tolerance ? true : false;
}

void DWA::transformObstacles(void)
{
    if (obstacles_.empty()) 
    {
        RCLCPP_ERROR(this->get_logger(), "No obstacles to transform");
        return;
    }

    // Create a tf2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a transform message to hold the result
    geometry_msgs::msg::TransformStamped transform_msg;

    try {
        // Get the transform from base_link to odom
        transform_msg = tf_buffer_->lookupTransform(frame_id_, "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Error looking up transform: %s", ex.what());
        return;
    }

    // Create a tf2 transform from the transform message
    tf2::Transform transform;
    tf2::fromMsg(transform_msg.transform, transform);

    for (size_t i = 0; i < obstacles_.size(); i++) {
        // Create a point in base_link frame
        tf2::Vector3 point_base_link;
        point_base_link.setX(obstacles_[i].first);
        point_base_link.setY(obstacles_[i].second);

        // Transform the point to the odom frame
        tf2::Vector3 point_odom = transform * point_base_link;

        // Update the obstacle position in the odom frame
        obstacles_[i].first = point_odom.getX();
        obstacles_[i].second = point_odom.getY();
    }
}

void DWA::dwaControl(void)
{
    // Update Parameter
    updateParameter();
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
    // Publish the current robot pose
    publishCurrentPose();
    // Publish the current best trajectory 
    publishPath(best_trajectory);
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