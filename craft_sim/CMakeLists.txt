cmake_minimum_required(VERSION 3.8)
project(craft_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Custom messages ──────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleParams.msg"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── C++ nodes ─────────────────────────────────────────────────────────────────
foreach(node simulator_node rate_controller_node attitude_controller_node)
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PRIVATE include)
  ament_target_dependencies(${node} rclcpp geometry_msgs)
  target_link_libraries(${node} ${cpp_typesupport_target})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

# ── Python package ────────────────────────────────────────────────────────────
ament_python_install_package(craft_sim_viz)

install(PROGRAMS
  craft_sim_viz/visualizer_node.py
  craft_sim_viz/flight_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch files ──────────────────────────────────────────────────────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
