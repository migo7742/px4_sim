cmake_minimum_required(VERSION 3.8)
project(px4_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(offboard_control_node src/offboard_control.cpp)
ament_target_dependencies(offboard_control_node 
  rclcpp 
  px4_msgs 
  quadrotor_msgs
  std_msgs
)

# 链接 Eigen
target_link_libraries(offboard_control_node Eigen3::Eigen)

install(TARGETS
  offboard_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
