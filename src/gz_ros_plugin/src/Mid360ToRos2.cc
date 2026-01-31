// 文件路径: src/Mid360ToRos2.cc

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>

#include <rclcpp/rclcpp.hpp>
// 引入 Livox 自定义消息头文件
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

using namespace gz;
using namespace sim;
using namespace systems;

class Mid360ToRos2 : public System, public ISystemConfigure, public ISystemPostUpdate
{
public:
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override
    {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        
        std::string node_name = "mid360_bridge_node";
        this->ros_node_ = std::make_shared<rclcpp::Node>(node_name);

        // 发布 Livox CustomMsg
        this->ros_pub_ = this->ros_node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10);

        this->gz_node_ = std::make_shared<transport::Node>();
        
        // 这里的 topic 需要和你 SDF 文件中 Lidar 插件输出的 topic 一致
        std::string gz_topic = "/mid360/points"; 
        if (_sdf->HasElement("topic")) {
            gz_topic = _sdf->Get<std::string>("topic");
        }

        this->gz_node_->Subscribe(gz_topic, &Mid360ToRos2::OnPointCloud, this);
        RCLCPP_INFO(this->ros_node_->get_logger(), "Mid360ToRos2 Listening to Gazebo topic: %s", gz_topic.c_str());
    }

    void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override {
        if (rclcpp::ok()) rclcpp::spin_some(this->ros_node_);
    }

    void OnPointCloud(const msgs::PointCloudPacked &_msg)
    {
        livox_ros_driver2::msg::CustomMsg ros_msg;

        // 1. 处理时间戳 (Timebase)
        uint64_t timestamp_ns = 0;
        if (_msg.has_header() && _msg.header().has_stamp()) {
            timestamp_ns = static_cast<uint64_t>(_msg.header().stamp().sec()) * 1000000000 + 
                           static_cast<uint64_t>(_msg.header().stamp().nsec());
        } else {
            timestamp_ns = this->ros_node_->get_clock()->now().nanoseconds();
        }

        ros_msg.header.stamp = rclcpp::Time(timestamp_ns);
        ros_msg.header.frame_id = "mid360_link"; // 请根据你的 URDF 修改这个 Frame ID
        ros_msg.timebase = timestamp_ns;
        ros_msg.lidar_id = 192; 
        ros_msg.rsvd = {0, 0, 0};

        // 2. 解析 Gazebo 数据字段偏移量
        int offset_x = -1, offset_y = -1, offset_z = -1;
        int offset_intensity = -1;
        
        for (int i = 0; i < _msg.field_size(); ++i) {
            const auto &field = _msg.field(i);
            if (field.name() == "x") offset_x = field.offset();
            else if (field.name() == "y") offset_y = field.offset();
            else if (field.name() == "z") offset_z = field.offset();
            else if (field.name() == "intensity") offset_intensity = field.offset();
        }

        if (offset_x == -1 || offset_y == -1 || offset_z == -1) return;

        uint32_t point_step = _msg.point_step();
        uint32_t num_points = _msg.width() * _msg.height();
        const char* data_ptr = _msg.data().c_str();

        ros_msg.point_num = num_points;
        ros_msg.points.reserve(num_points);

        // 3. 填充点云数据
        for (uint32_t i = 0; i < num_points; ++i) {
            livox_ros_driver2::msg::CustomPoint pt;

            pt.x = *reinterpret_cast<const float*>(data_ptr + offset_x);
            pt.y = *reinterpret_cast<const float*>(data_ptr + offset_y);
            pt.z = *reinterpret_cast<const float*>(data_ptr + offset_z);

            if (offset_intensity != -1) {
                float intensity = *reinterpret_cast<const float*>(data_ptr + offset_intensity);
                // 简单的强度映射
                if (intensity <= 1.0f) pt.reflectivity = static_cast<uint8_t>(intensity * 255.0f);
                else pt.reflectivity = static_cast<uint8_t>(std::min(intensity, 255.0f));
            } else {
                pt.reflectivity = 100;
            }

            pt.tag = 0;
            pt.line = 0; // Gazebo 默认很难模拟出真实的线束ID，暂时置0，不影响 Fast-LIO 运行
            pt.offset_time = 0; // 模拟瞬时扫描

            ros_msg.points.push_back(pt);
            data_ptr += point_step;
        }

        this->ros_pub_->publish(ros_msg);
    }

private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr ros_pub_;
    std::shared_ptr<transport::Node> gz_node_;
};

// 注册插件
GZ_ADD_PLUGIN(Mid360ToRos2, gz::sim::System, Mid360ToRos2::ISystemConfigure, Mid360ToRos2::ISystemPostUpdate)
// 设置别名，你在 SDF 里用 <plugin filename="Mid360ToRos2" name="mid360_to_ros2"> 调用它
GZ_ADD_PLUGIN_ALIAS(Mid360ToRos2, "mid360_to_ros2")
