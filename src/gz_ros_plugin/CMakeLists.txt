cmake_minimum_required(VERSION 3.8)
project(gz_ros_plugin)

set(CMAKE_CXX_STANDARD 17)

# --------------------------------------------------------
# 1. 寻找依赖 (适配 Gazebo Harmonic / Sim 8)
# --------------------------------------------------------
find_package(ament_cmake REQUIRED)

# Harmonic 版本对应的库后缀：sim8, transport13, msgs10
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# ROS 2 依赖 (保持不变)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# --------------------------------------------------------
# 2. 定义库: LidarToRos2
# --------------------------------------------------------
add_library(LidarToRos2 SHARED src/LidarToRos2.cc)
ament_target_dependencies(LidarToRos2 rclcpp sensor_msgs)
target_link_libraries(LidarToRos2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# --------------------------------------------------------
# 3. 定义库: Mid360ToRos2 (Livox)
# --------------------------------------------------------
add_library(Mid360ToRos2 SHARED src/Mid360ToRos2.cc)
ament_target_dependencies(Mid360ToRos2 
  rclcpp 
  sensor_msgs 
  livox_ros_driver2
)
target_link_libraries(Mid360ToRos2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# --------------------------------------------------------
# 4. 定义库: ImuToRos2
# --------------------------------------------------------
add_library(ImuToRos2 SHARED src/ImuToRos2.cc)
ament_target_dependencies(ImuToRos2 rclcpp sensor_msgs)
target_link_libraries(ImuToRos2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# --------------------------------------------------------
# 5. 定义库: OakDToRos2
# --------------------------------------------------------
add_library(OakDToRos2 SHARED src/OakDToRos2.cc)
ament_target_dependencies(OakDToRos2 rclcpp sensor_msgs)
target_link_libraries(OakDToRos2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# --------------------------------------------------------
# 6. 定义库: ClockToRos2
# --------------------------------------------------------
add_library(ClockToRos2 SHARED src/ClockToRos2.cc)
ament_target_dependencies(ClockToRos2 rclcpp rosgraph_msgs)
target_link_libraries(ClockToRos2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

# --------------------------------------------------------
# 7. 安装规则
# --------------------------------------------------------
install(TARGETS 
  LidarToRos2
  ClockToRos2 
  Mid360ToRos2
  ImuToRos2
  OakDToRos2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
