cmake_minimum_required(VERSION 3.5)
project(land_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  land_plugin_base
  rclcpp
  rclcpp_action
  nav_msgs
  as2_core
  as2_msgs
  motion_reference_handlers
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

add_library(land_plugins src/land_plugin_speed.cpp)
target_include_directories(land_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  land_plugins
  ${PROJECT_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(land_plugin_base plugins.xml)

install(
  TARGETS land_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  land_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
