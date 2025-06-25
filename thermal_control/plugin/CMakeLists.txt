cmake_minimum_required(VERSION 3.10)
project(thermal_plugin)

# Required packages
find_package(Protobuf REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Define proto files
set(PROTO_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/thermal_control/msgs/ThermalNodeState.proto
)

# Generate custom messages
gz_msgs_generate_messages(
  TARGET thermal_plugin-msgs
  PROTO_PACKAGE "thermal_control.msgs"
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  MSGS_PROTOS ${PROTO_FILES}
  DEPENDENCIES gz-msgs10::gz-msgs10
)

# Plugin source
add_library(ThermalPlugin SHARED
  thermal_plugin.cc
)

target_include_directories(ThermalPlugin PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/thermal_plugin-thermal_plugin-msgs_genmsg
  ${Protobuf_INCLUDE_DIRS}
)


# Link dependencies
target_link_libraries(ThermalPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-msgs10::gz-msgs10
  ${Protobuf_LIBRARIES}
)

# Output lib
set_target_properties(ThermalPlugin PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../lib
)

# Install
install(TARGETS ThermalPlugin
  LIBRARY DESTINATION lib
)
