cmake_minimum_required(VERSION 3.8)
project(rmw_zenoh_examples_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# odom_subscription
add_library(odom_subscription_component SHARED
  src/odom_subscription.cpp
)

target_link_libraries(odom_subscription_component
  rclcpp::rclcpp
  rclcpp_components::component
  ${nav_msgs_TARGETS}
)
rclcpp_components_register_node(odom_subscription_component
  PLUGIN "rmw_zenoh_examples::OdomSubscription"
  EXECUTABLE odom_subscription_node
  EXECUTOR EventsExecutor
)

install(
  TARGETS odom_subscription_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
