cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME land_plugins)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
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
  std_srvs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

add_library(land_plugins src/land_plugin_speed.cpp src/land_plugin_platform.cpp)
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
