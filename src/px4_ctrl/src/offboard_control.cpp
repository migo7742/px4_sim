/**
 * @file offboard_control.cpp
 * @brief C++ implementation of the offboard control node for PX4
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

// 假设 quadrotor_msgs/PositionCommand 结构包含 position, velocity, yaw 等字段
#include "quadrotor_msgs/msg/position_command.hpp" 
#include "std_msgs/msg/string.hpp"

#include <Eigen/Dense>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("ego_pos_command_publisher") {
        // QoS Profiles
        rmw_qos_profile_t qos_profile_pub = rmw_qos_profile_default;
        qos_profile_pub.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        qos_profile_pub.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        qos_profile_pub.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_profile_pub.depth = 1;

        rmw_qos_profile_t qos_profile_sub = rmw_qos_profile_default;
        qos_profile_sub.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        qos_profile_sub.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        qos_profile_sub.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_profile_sub.depth = 1;

        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(1), qos_profile_pub);
        auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(1), qos_profile_sub);

        // Subscribers
        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "fmu/out/vehicle_status", qos_sub,
            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_sub,
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));

        vehicle_visual_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", qos_sub,
            std::bind(&OffboardControl::vehicle_visual_odom_callback, this, std::placeholders::_1));

        planning_pos_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "/drone_0_planning/pos_cmd", qos_sub,
            std::bind(&OffboardControl::planning_pos_cmd_callback, this, std::placeholders::_1));

        mode_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mode_key", qos_sub,
            std::bind(&OffboardControl::mode_cmd_callback, this, std::placeholders::_1));

        // Publishers
        publisher_offboard_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "fmu/in/offboard_control_mode", qos_pub);
        
        publisher_trajectory_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "fmu/in/trajectory_setpoint", qos_pub);
        
        publisher_vehicle_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos_pub);

        // Transformation Matrix (ROS ENU <-> PX4 NED/FRD adapter depending on setup)
        // Python: [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        ros_to_fmu_ << 0, 1, 0,
                       1, 0, 0,
                       0, 0, -1;
        
        // Timer (10Hz for stability, Python was 1Hz which is dangerous for offboard)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&OffboardControl::cmdloop_callback, this));
        
        // Initialize variables
        control_mode_ = "m";
        nav_state_ = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MAX;
        arming_state_ = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    }

private:
    // Variables
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_visual_odom_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr planning_pos_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_cmd_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr publisher_offboard_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_trajectory_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_vehicle_command_;

    std::string control_mode_;
    px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
    px4_msgs::msg::VehicleOdometry vehicle_visual_odom_;
    px4_msgs::msg::VehicleStatus vehicle_status_;
    quadrotor_msgs::msg::PositionCommand latest_planning_msg_;

    uint8_t nav_state_;
    uint8_t arming_state_;
    
    bool vehicle_local_position_received_ = false;
    bool vehicle_visual_odom_received_ = false;
    bool planning_pos_command_received_ = false;
    bool takeoff_hover_des_set_ = false;
    bool offboard_hover_des_set_ = false;
    bool in_position_hold_ = false;

    px4_msgs::msg::TrajectorySetpoint hover_setpoint_;
    Eigen::Matrix3d ros_to_fmu_;

    // Helper: Quaternion to Yaw
    double quaternion_to_yaw(double w, double x, double y, double z) {
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // Callbacks
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        nav_state_ = msg->nav_state;
        arming_state_ = msg->arming_state;
        vehicle_status_ = *msg;
    }

    void vehicle_visual_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        vehicle_visual_odom_ = *msg;
        vehicle_visual_odom_received_ = true;
    }

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        vehicle_local_position_ = *msg;
        vehicle_local_position_received_ = true;
    }

    void planning_pos_cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
        latest_planning_msg_ = *msg;
        planning_pos_command_received_ = true;
    }

    void mode_cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
        control_mode_ = msg->data;
    }

    // Command Helpers
    void publish_offboard_control_heartbeat_signal() {
        px4_msgs::msg::OffboardControlMode msg;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        publisher_offboard_mode_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, double p1 = 0.0, double p2 = 0.0, 
                                 double p3 = 0.0, double p4 = 0.0, double p5 = 0.0, 
                                 double p6 = 0.0, double p7 = 0.0) {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = command;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.param3 = p3;
        msg.param4 = p4;
        msg.param5 = p5;
        msg.param6 = p6;
        msg.param7 = p7;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        publisher_vehicle_command_->publish(msg);
    }

    void engage_offboard_mode() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        RCLCPP_INFO(this->get_logger(), "Switching to offboard mode");
    }

    void enter_position_mode() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0);
        RCLCPP_INFO(this->get_logger(), "Exiting OFFBOARD mode -> Switching to POSITION mode");
    }

    void arm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    void disarm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent");
    }

    void takeoff() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0);
        RCLCPP_INFO(this->get_logger(), "Taking off to 1.0 meters");
    }

    void land() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(this->get_logger(), "Switching to land mode");
    }

    // Publishers logic
    void position_msg_pub() {
        px4_msgs::msg::TrajectorySetpoint msg;
        
        if (vehicle_local_position_received_ && !vehicle_visual_odom_received_) {
            if (!takeoff_hover_des_set_) {
                hover_setpoint_.position[0] = vehicle_local_position_.x;
                hover_setpoint_.position[1] = vehicle_local_position_.y;
                hover_setpoint_.position[2] = -2.5;
                hover_setpoint_.yaw = vehicle_local_position_.heading;
                takeoff_hover_des_set_ = true;
            }
        } else if (vehicle_visual_odom_received_) {
            if (!takeoff_hover_des_set_) {
                hover_setpoint_.position[0] = vehicle_visual_odom_.position[0];
                hover_setpoint_.position[1] = vehicle_visual_odom_.position[1];
                hover_setpoint_.position[2] = -2.5;
                auto q = vehicle_visual_odom_.q;
                hover_setpoint_.yaw = quaternion_to_yaw(q[0], q[1], q[2], q[3]);
                takeoff_hover_des_set_ = true;
            }
        }

        if (takeoff_hover_des_set_) {
            msg.position = hover_setpoint_.position;
            msg.yaw = hover_setpoint_.yaw;
        } else {
            // Default initialization if data not received yet
             msg.position[0] = std::numeric_limits<float>::quiet_NaN();
             msg.position[1] = std::numeric_limits<float>::quiet_NaN();
             msg.position[2] = std::numeric_limits<float>::quiet_NaN();
             msg.yaw = std::numeric_limits<float>::quiet_NaN();
        }

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        publisher_trajectory_->publish(msg);
    }

    void hover_cmd_pub() {
        px4_msgs::msg::TrajectorySetpoint msg;
        
        if (vehicle_local_position_received_ && !vehicle_visual_odom_received_) {
            if (!offboard_hover_des_set_) {
                hover_setpoint_.position[0] = vehicle_local_position_.x;
                hover_setpoint_.position[1] = vehicle_local_position_.y;
                hover_setpoint_.position[2] = vehicle_local_position_.z;
                hover_setpoint_.yaw = vehicle_local_position_.heading;
                offboard_hover_des_set_ = true;
            }
        } else if (vehicle_visual_odom_received_) {
            if (!offboard_hover_des_set_) {
                hover_setpoint_.position[0] = vehicle_visual_odom_.position[0];
                hover_setpoint_.position[1] = vehicle_visual_odom_.position[1];
                hover_setpoint_.position[2] = vehicle_visual_odom_.position[2];
                auto q = vehicle_visual_odom_.q;
                hover_setpoint_.yaw = quaternion_to_yaw(q[0], q[1], q[2], q[3]);
                offboard_hover_des_set_ = true;
            }
        }

        if (offboard_hover_des_set_) {
            msg.position = hover_setpoint_.position;
            msg.yaw = hover_setpoint_.yaw;
        } else {
             msg.position[0] = std::numeric_limits<float>::quiet_NaN();
             msg.position[1] = std::numeric_limits<float>::quiet_NaN();
             msg.position[2] = std::numeric_limits<float>::quiet_NaN();
             msg.yaw = std::numeric_limits<float>::quiet_NaN();
        }

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        publisher_trajectory_->publish(msg);
    }

    void ego_cmd_pub() {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        Eigen::Vector3d planning_position(latest_planning_msg_.position.x,
                                          latest_planning_msg_.position.y,
                                          latest_planning_msg_.position.z);
        
        Eigen::Vector3d planning_velocity(latest_planning_msg_.velocity.x,
                                          latest_planning_msg_.velocity.y,
                                          latest_planning_msg_.velocity.z);

        // Inverse of rotation matrix (Transpose for rotation matrices)
        Eigen::Matrix3d inv_ros_to_fmu = ros_to_fmu_.inverse();

        Eigen::Vector3d transformed_pos = inv_ros_to_fmu * planning_position;
        Eigen::Vector3d transformed_vel = inv_ros_to_fmu * planning_velocity;

        msg.position[0] = transformed_pos.x();
        msg.position[1] = transformed_pos.y();
        msg.position[2] = transformed_pos.z();
        
        msg.velocity[0] = transformed_vel.x();
        msg.velocity[1] = transformed_vel.y();
        msg.velocity[2] = transformed_vel.z();

        double yaw_enu = latest_planning_msg_.yaw;
        msg.yaw = std::atan2(std::cos(yaw_enu), std::sin(yaw_enu));
        
        publisher_trajectory_->publish(msg);
    }

    // Main Loop
    void cmdloop_callback() {
        publish_offboard_control_heartbeat_signal();

        if (control_mode_ == "m") {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "manual control");
            return;
        }

        if (control_mode_ == "t") {
            if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                engage_offboard_mode();
            }
            if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                arm();
            }
            offboard_hover_des_set_ = false;
            position_msg_pub();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "takeoff");
            return;
        }

        if (control_mode_ == "p") {
            takeoff_hover_des_set_ = false;
            hover_cmd_pub();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "position mode");
            return;
        }

        if (control_mode_ == "o" && !planning_pos_command_received_) {
            in_position_hold_ = true;
            takeoff_hover_des_set_ = false;
            hover_cmd_pub();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No command in offboard, return to position mode");
            return;
        }

        if (control_mode_ == "o" && planning_pos_command_received_) {
            if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                engage_offboard_mode();
            }
            
            if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                offboard_hover_des_set_ = false;
                takeoff_hover_des_set_ = false;
                ego_cmd_pub();
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "offboard velocity");
                return;
            } else {
                 // Fallback if not switched yet
                 in_position_hold_ = true;
                 hover_cmd_pub();
                 RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for offboard switch...");
                 return;
            }
        }

        if (control_mode_ == "l") {
            land();
            return;
        }

        if (control_mode_ == "d") {
            disarm();
            return;
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
