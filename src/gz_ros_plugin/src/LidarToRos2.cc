#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

using namespace gz;
using namespace sim;
using namespace systems;

class LidarToRos2 : public System, public ISystemConfigure, public ISystemPostUpdate
{
public:
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override
    {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        
        std::string node_name = "lidar_bridge_plugin";
        this->ros_node_ = std::make_shared<rclcpp::Node>(node_name);

        this->ros_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10); 

        this->gz_node_ = std::make_shared<transport::Node>();
        // 确认这里是 gz topic -l 看到的名字
        std::string gz_topic = "/mid360/points"; 
        
        this->gz_node_->Subscribe(gz_topic, &LidarToRos2::OnPointCloud, this);
        RCLCPP_INFO(this->ros_node_->get_logger(), "LidarToRos2 Listening to %s", gz_topic.c_str());
    }

    void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override {
        if (rclcpp::ok()) rclcpp::spin_some(this->ros_node_);
    }

    void OnPointCloud(const msgs::PointCloudPacked &_msg)
    {
        auto ros_msg = sensor_msgs::msg::PointCloud2();

        // ==========================================
        // 1. 时间戳处理
        // ==========================================
        // 适配器逻辑: new_point.timestamp = *out_timestamp * 1e-9;
        // 因此我们需要在这个字段里填入【总纳秒数】，而不是秒。
        double timestamp_ns_float = 0.0; 

        if (_msg.has_header() && _msg.header().has_stamp()) {
            int64_t sec = _msg.header().stamp().sec();
            int64_t nsec = _msg.header().stamp().nsec();
            
            ros_msg.header.stamp = rclcpp::Time(sec, nsec);
            
            // 将秒和纳秒转换为 double 类型的总纳秒数
            timestamp_ns_float = static_cast<double>(sec) * 1e9 + static_cast<double>(nsec);
        } else {
            auto now = this->ros_node_->get_clock()->now();
            ros_msg.header.stamp = now;
            timestamp_ns_float = static_cast<double>(now.nanoseconds());
        }

        ros_msg.header.frame_id = "mid360_link"; 
        ros_msg.height = _msg.height();
        ros_msg.width = _msg.width();
        ros_msg.is_bigendian = _msg.is_bigendian();
        ros_msg.is_dense = _msg.is_dense();

        // ==========================================
        // 2. 字段定义 (Fields)
        // ==========================================
        // 我们需要保留原有的 x, y, z, intensity，并追加 tag 和 timestamp
        
        uint32_t original_step = _msg.point_step();
        
        // 计算新的步长：原步长 + tag(1字节) + timestamp(8字节)
        uint32_t tag_size = sizeof(uint8_t);
        uint32_t time_size = sizeof(double);
        
        ros_msg.point_step = original_step + tag_size + time_size;
        ros_msg.row_step = ros_msg.width * ros_msg.point_step;

        // 复制原有字段定义 (x, y, z, intensity...)
        for (int i = 0; i < _msg.field_size(); ++i) {
            sensor_msgs::msg::PointField pf;
            pf.name = _msg.field(i).name();
            pf.offset = _msg.field(i).offset();
            pf.count = _msg.field(i).count();
            if (pf.name == "x" || pf.name == "y" || pf.name == "z" || pf.name == "intensity") {
                pf.datatype = sensor_msgs::msg::PointField::FLOAT32; 
            } else {
                pf.datatype = _msg.field(i).datatype();
            }
            ros_msg.fields.push_back(pf);
        }

        // 添加 "tag" 字段 (uint8)
        sensor_msgs::msg::PointField tag_field;
        tag_field.name = "tag";
        tag_field.offset = original_step; // 紧接在原有数据之后
        tag_field.datatype = sensor_msgs::msg::PointField::UINT8;
        tag_field.count = 1;
        ros_msg.fields.push_back(tag_field);

        // 添加 "timestamp" 字段 (double)
        sensor_msgs::msg::PointField ts_field;
        ts_field.name = "timestamp";
        ts_field.offset = original_step + tag_size; // 紧接在 tag 之后
        ts_field.datatype = sensor_msgs::msg::PointField::FLOAT64;
        ts_field.count = 1;
        ros_msg.fields.push_back(ts_field);

        // ==========================================
        // 3. 数据填充 (Data Filling)
        // ==========================================
        ros_msg.data.resize(ros_msg.row_step * ros_msg.height);
        
        const char* src_ptr = _msg.data().c_str();
        uint8_t* dest_ptr = ros_msg.data.data();
        int num_points = _msg.width() * _msg.height();

        // 定义要填入的 tag 值。
        // 适配器要求: (*out_tag & 0b00111111) == 0
        // 设置为 0 即可满足条件。
        uint8_t tag_value = 0;

        for (int i = 0; i < num_points; ++i) {
            // 1. 拷贝原始数据 (x, y, z, intensity)
            memcpy(dest_ptr, src_ptr, original_step);
            
            // 2. 追加 tag
            memcpy(dest_ptr + original_step, &tag_value, tag_size);
            
            // 3. 追加 timestamp (总纳秒数)
            memcpy(dest_ptr + original_step + tag_size, &timestamp_ns_float, time_size);

            // 指针移动
            src_ptr += original_step;
            dest_ptr += ros_msg.point_step;
        }

        this->ros_pub_->publish(ros_msg);
    }

private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ros_pub_;
    std::shared_ptr<transport::Node> gz_node_;
};

GZ_ADD_PLUGIN(LidarToRos2, gz::sim::System, LidarToRos2::ISystemConfigure, LidarToRos2::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(LidarToRos2, "lidar_to_ros2")
