#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>
#include <string>

using namespace gz;
using namespace sim;
using namespace systems;

class ImuToRos2 : public System, public ISystemConfigure, public ISystemPostUpdate
{
public:
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override
    {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        
        std::string node_name = "mid360_imu_bridge";
        this->ros_node_ = std::make_shared<rclcpp::Node>(node_name);

        this->ros_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
            "/livox/imu", 10); // 确认这是算法订阅的IMU话题

        this->gz_node_ = std::make_shared<transport::Node>();
        // 确认这里是 gz topic -l 看到的名字
        std::string gz_topic = "/mid360/imu"; 
        
        this->gz_node_->Subscribe(gz_topic, &ImuToRos2::OnImuMsg, this);
        RCLCPP_INFO(this->ros_node_->get_logger(), "IMU Bridge Listening to %s", gz_topic.c_str());
    }

    void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override {
        if (rclcpp::ok()) rclcpp::spin_some(this->ros_node_);
    }

    void OnImuMsg(const msgs::IMU &_msg)
    {
        auto ros_msg = sensor_msgs::msg::Imu();

        // ==========================================
        // 【绝对核心】使用 Gazebo 物理时间
        // ==========================================
        // 必须和 LiDAR 插件的逻辑一模一样
        if (_msg.has_header() && _msg.header().has_stamp()) {
            ros_msg.header.stamp = rclcpp::Time(
                _msg.header().stamp().sec(),
                _msg.header().stamp().nsec()
            );
        } else {
            ros_msg.header.stamp = this->ros_node_->get_clock()->now();
        }
        
        ros_msg.header.frame_id = "mid360_link"; 

        // 姿态
        if (_msg.has_orientation()) {
            ros_msg.orientation.x = _msg.orientation().x();
            ros_msg.orientation.y = _msg.orientation().y();
            ros_msg.orientation.z = _msg.orientation().z();
            ros_msg.orientation.w = _msg.orientation().w();
        } else {
            ros_msg.orientation.w = 1.0;
        }

        // 角速度
        if (_msg.has_angular_velocity()) {
            ros_msg.angular_velocity.x = _msg.angular_velocity().x();
            ros_msg.angular_velocity.y = _msg.angular_velocity().y();
            ros_msg.angular_velocity.z = _msg.angular_velocity().z();
        }

        // 线加速度
        if (_msg.has_linear_acceleration()) {
            ros_msg.linear_acceleration.x = _msg.linear_acceleration().x();
            ros_msg.linear_acceleration.y = _msg.linear_acceleration().y();
            ros_msg.linear_acceleration.z = _msg.linear_acceleration().z();
        }
        
        ros_msg.orientation_covariance[0] = -1;
        ros_msg.angular_velocity_covariance[0] = -1;
        ros_msg.linear_acceleration_covariance[0] = -1;

        this->ros_pub_->publish(ros_msg);
    }

private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros_pub_;
    std::shared_ptr<transport::Node> gz_node_;
};

GZ_ADD_PLUGIN(ImuToRos2, gz::sim::System, ImuToRos2::ISystemConfigure, ImuToRos2::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(ImuToRos2, "imu_to_ros2")
