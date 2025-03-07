cmake_minimum_required(VERSION 3.8)
project(jolie_utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Tambahkan executable untuk node twist_converter_node
add_executable(twist_converter_node src/twist_converter_node.cpp)
ament_target_dependencies(twist_converter_node rclcpp geometry_msgs)

# Pasang (install) executable ke dalam direktori yang sesuai
install(TARGETS
  twist_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
