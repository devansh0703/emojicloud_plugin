cmake_minimum_required(VERSION 3.8)
project(emojicloud_plugin)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)

# Qt setup (similar to ROS1, but ensure Qt5/Qt6 compatibility)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets)

# Add definitions
add_definitions(-DQT_NO_KEYWORDS)

# Ensure rviz_ogre_vendor headers are used before system OGRE
include_directories(BEFORE /opt/ros/humble/opt/rviz_ogre_vendor/include)

# Source files
set(SOURCE_FILES
  src/point_cloud.cpp
  src/point_cloud_common.cpp
  src/emoji_laser_scan_display.cpp
  src/emoji_point_cloud2_display.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz2
  rviz_default_plugins
  rviz_rendering
  rviz_common
  sensor_msgs
  laser_geometry
  pluginlib
  ament_index_cpp
  message_filters
  tf2_ros
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)

install(DIRECTORY shaders/
  DESTINATION share/${PROJECT_NAME}/shaders
)

ament_package()


