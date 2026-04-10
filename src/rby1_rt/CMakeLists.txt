cmake_minimum_required(VERSION 3.18)
project(rby1_rt)

set(CMAKE_CXX_STANDARD 17)

# ─── Find ROS2 packages ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(rby1_core_msgs REQUIRED)

# ─── Find rby1-sdk ───────────────────────────────────────────────────────────
# Build & install the SDK first:
#   cd <sdk-root>  && cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local
#   cmake --build build && cmake --install build
# Or point CMAKE_PREFIX_PATH to the SDK build directory:
#   colcon build --cmake-args -DCMAKE_PREFIX_PATH=/path/to/rby1-sdk-main/build
find_package(rby1-sdk REQUIRED)
find_package(Threads REQUIRED)

# ─── Build targets ───────────────────────────────────────────────────────────
# robot_a (differential 2-wheel, default)
add_executable(rby1_rt_node rby1_rt_node.cpp)

target_link_libraries(rby1_rt_node
  rby1-sdk::rby1-sdk
  Threads::Threads
)

ament_target_dependencies(rby1_rt_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  interbotix_xs_msgs
  rby1_core_msgs
)

# robot_m (mecanum 4-wheel)
add_executable(rby1_rt_node_m rby1_rt_node.cpp)

target_compile_definitions(rby1_rt_node_m PRIVATE USE_MODEL_M=1)

target_link_libraries(rby1_rt_node_m
  rby1-sdk::rby1-sdk
  Threads::Threads
)

ament_target_dependencies(rby1_rt_node_m
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  interbotix_xs_msgs
  rby1_core_msgs
)

# ─── Install ─────────────────────────────────────────────────────────────────
install(TARGETS rby1_rt_node rby1_rt_node_m
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
