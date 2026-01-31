#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

// 万能头文件，包含 Image 等所有消息定义
#include <gz/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

using namespace gz;
using namespace sim;
using namespace systems;

class OakDToRos2 : public System, public ISystemConfigure, public ISystemPostUpdate
{
public:
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override
    {
        // 1. 初始化 ROS 2
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        std::string node_name = "oakd_bridge_plugin";
        this->ros_node_ = std::make_shared<rclcpp::Node>(node_name);

        // ---------------------------------------------------------
        // 配置 RGB 相机
        // ---------------------------------------------------------
        // 使用 Reliable (10) 以兼容 RViz
        this->rgb_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
            "/oakd/rgb/image_raw", 10);
        
        // 注意：这里的 topic 必须匹配 gz topic -l 显示的名字
        // 如果 SDF 里写的是 <topic>camera</topic>，通常 Gazebo 会加上 /camera
        std::string rgb_gz_topic = "/camera"; 
        
        this->gz_node_ = std::make_shared<transport::Node>();
        this->gz_node_->Subscribe(rgb_gz_topic, &OakDToRos2::OnRgbImage, this);
        RCLCPP_INFO(this->ros_node_->get_logger(), "Subscribed to Gazebo RGB: %s", rgb_gz_topic.c_str());

        // ---------------------------------------------------------
        // 配置 深度(Depth) 相机
        // ---------------------------------------------------------
        this->depth_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
            "/oakd/depth/image_raw", 10);

        std::string depth_gz_topic = "/depth_camera";
        
        this->gz_node_->Subscribe(depth_gz_topic, &OakDToRos2::OnDepthImage, this);
        RCLCPP_INFO(this->ros_node_->get_logger(), "Subscribed to Gazebo Depth: %s", depth_gz_topic.c_str());
    }

    void PostUpdate(const UpdateInfo &_info,
                    const EntityComponentManager &_ecm) override
    {
        if (rclcpp::ok()) {
            rclcpp::spin_some(this->ros_node_);
        }
    }

    // --- RGB 回调 ---
    void OnRgbImage(const msgs::Image &_msg)
    {
        auto ros_msg = sensor_msgs::msg::Image();
        
        ros_msg.header.stamp = this->ros_node_->get_clock()->now();
        ros_msg.header.frame_id = "camera_link"; // RViz Fixed Frame 要用这个

        ros_msg.height = _msg.height();
        ros_msg.width = _msg.width();
        
        // Gazebo 通常输出 rgb8 (R8G8B8)
        ros_msg.encoding = "rgb8";
        ros_msg.is_bigendian = 0;
        
        // 步长 = 宽度 * 3 (因为是RGB 3通道)
        ros_msg.step = _msg.width() * 3; 

        // 拷贝数据
        ros_msg.data.resize(_msg.data().size());
        memcpy(ros_msg.data.data(), _msg.data().c_str(), _msg.data().size());

        this->rgb_pub_->publish(ros_msg);
    }

    // --- Depth 回调 ---
    void OnDepthImage(const msgs::Image &_msg)
    {
        auto ros_msg = sensor_msgs::msg::Image();

        ros_msg.header.stamp = this->ros_node_->get_clock()->now();
        ros_msg.header.frame_id = "camera_link";

        ros_msg.height = _msg.height();
        ros_msg.width = _msg.width();

        // SDF 里定义的是 R_FLOAT32，对应 ROS 的 32FC1 (32位浮点单通道)
        ros_msg.encoding = "32FC1"; 
        ros_msg.is_bigendian = 0;

        // 步长 = 宽度 * 4 (因为 float 是 4 字节)
        ros_msg.step = _msg.width() * 4;

        ros_msg.data.resize(_msg.data().size());
        memcpy(ros_msg.data.data(), _msg.data().c_str(), _msg.data().size());

        this->depth_pub_->publish(ros_msg);
    }

private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    std::shared_ptr<transport::Node> gz_node_;
};

// 注册插件
GZ_ADD_PLUGIN(OakDToRos2,
              gz::sim::System,
              OakDToRos2::ISystemConfigure,
              OakDToRos2::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(OakDToRos2, "oakd_to_ros2")
