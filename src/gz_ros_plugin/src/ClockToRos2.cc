#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <memory>
#include <string>

using namespace gz;
using namespace sim;
using namespace systems;

class ClockToRos2 : public System, public ISystemConfigure, public ISystemPostUpdate
{
public:
    // 1. 配置阶段：初始化 ROS 2 节点和 Publisher
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override
    {
        // 确保 ROS 2 环境初始化
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        // 创建 ROS 2 节点
        std::string node_name = "gazebo_clock_bridge";
        this->ros_node_ = std::make_shared<rclcpp::Node>(node_name);

        // 创建 /clock 发布者
        // QoS 设为 BestEffort 以保证即时性，或者 Reliable 也可以，/clock 通常比较耐造
        this->clock_pub_ = this->ros_node_->create_publisher<rosgraph_msgs::msg::Clock>(
            "/clock", 10);
            
        RCLCPP_INFO(this->ros_node_->get_logger(), "ClockToRos2 plugin initialized. Publishing to /clock");
    }

    // 2. 每次仿真更新后调用：获取时间并发布
    void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override
    {
        // 如果 ROS 挂了，就不跑了
        if (!rclcpp::ok()) return;

        // 获取 Gazebo 当前仿真时间 (simTime)
        auto sim_time = _info.simTime; // std::chrono::steady_clock::duration

        // 转换为秒和纳秒
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(sim_time).count();
        auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time).count() % 1000000000;

        // 构建 ROS 2 Clock 消息
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock.sec = static_cast<int32_t>(sec);
        clock_msg.clock.nanosec = static_cast<uint32_t>(nanosec);

        // 发布
        this->clock_pub_->publish(clock_msg);
        
        // 处理 ROS 回调 (虽然这里只发不收，但加上 spin 保持活性是好习惯)
        rclcpp::spin_some(this->ros_node_);
    }

private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
};

// 注册插件
GZ_ADD_PLUGIN(ClockToRos2, gz::sim::System, ClockToRos2::ISystemConfigure, ClockToRos2::ISystemPostUpdate)
// 注册别名，方便 SDF 调用
GZ_ADD_PLUGIN_ALIAS(ClockToRos2, "clock_to_ros2")
