cmake_minimum_required(VERSION 3.8)
project(thermal_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
  "srv/Cooling.srv"
  "msg/ATCS.msg"

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs
)



include_directories(include)

add_executable(coldplate_system src/temp_publish_node.cpp)
ament_target_dependencies(coldplate_system std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(coldplate_system ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(cooling_process src/cooling_system_server.cpp)
ament_target_dependencies(cooling_process std_srvs rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(cooling_process ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  coldplate_system
  cooling_process
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()