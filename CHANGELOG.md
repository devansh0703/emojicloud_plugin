# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - 2025-11-16

### Added
- Initial ROS2 Humble support
- Ported from ROS1 to ROS2 APIs
- Updated to use rviz_common and rviz_default_plugins
- Added support for ROS2 message types (sensor_msgs/msg/PointCloud2, sensor_msgs/msg/LaserScan)
- Integrated with ROS2 build system (ament_cmake, colcon)

### Changed
- Migrated from rviz to rviz_common namespaces and APIs
- Updated selection handler to use RViz2's Handles struct instead of CollObjectHandle
- Changed TF buffer access to use FrameManager's connector and TFWrapper
- Fixed OGRE header conflicts by prioritizing rviz_ogre_vendor includes
- Updated material handling to use MaterialPtr instead of string names
- Replaced ROS1 property classes with rviz_common properties
- Adapted point cloud transformers to rviz_default_plugins APIs

### Fixed
- Compilation errors due to API changes in RViz2
- Selection box creation and destruction with correct handle types
- TF transformations for laser scan to point cloud conversion
- Material assignment for emoji rendering

### Removed
- ROS1 dependencies and APIs
- Old rviz namespace usage

## [1.0.0] - 2023-XX-XX (Original ROS1 Version)

- Initial release with emoji rendering for LaserScan and PointCloud2 in ROS1