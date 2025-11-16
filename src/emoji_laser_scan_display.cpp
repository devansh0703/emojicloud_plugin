/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rclcpp/time.hpp>

#include <laser_geometry/laser_geometry.hpp>

#include "point_cloud.h"
#include "point_cloud_common.h"
#include <rviz_default_plugins/transformation/tf_wrapper.hpp>

#include "emoji_laser_scan_display.h"

typedef rviz_common::MessageFilterDisplay<sensor_msgs::msg::LaserScan> MFDClass;

namespace emojicloud_plugin {
EmojiLaserScanDisplay::EmojiLaserScanDisplay()
    : point_cloud_common_(new PointCloudCommon(this)),
      projector_(new laser_geometry::LaserProjection()), filter_tolerance_(tf2::durationFromSec(0.0)) {}

EmojiLaserScanDisplay::~EmojiLaserScanDisplay() {
  delete point_cloud_common_;
  delete projector_;
}

void EmojiLaserScanDisplay::onInitialize() {
  MFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);

  static bool resource_locations_added = false;
  if (!resource_locations_added) {
    const std::string my_path =
        ament_index_cpp::get_package_share_directory("emojicloud_plugin") + "/shaders";
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        my_path, "FileSystem", "emojicloud_plugin");
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    resource_locations_added = true;
  }
}

void EmojiLaserScanDisplay::processMessage(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // Compute tolerance necessary for this scan
  tf2::Duration tolerance = tf2::durationFromSec(static_cast<double>(scan->time_increment) * scan->ranges.size());
  if (tolerance > filter_tolerance_) {
    filter_tolerance_ = tolerance;
    tf_filter_->setTolerance(filter_tolerance_);
  }

  try {
    auto tf_wrapper = std::dynamic_pointer_cast<rviz_default_plugins::transformation::TFWrapper>(
      context_->getFrameManager()->getConnector().lock());

    if (tf_wrapper) {
      projector_->transformLaserScanToPointCloud(
          fixed_frame_.toStdString(), *scan, *cloud, *tf_wrapper->getBuffer(), -1.0,
          laser_geometry::channel_option::Intensity);
    } else {
      RVIZ_COMMON_LOG_ERROR("Failed to get TF wrapper");
      return;
    }
  } catch (tf2::TransformException &e) {
    RVIZ_COMMON_LOG_DEBUG(QString("LaserScan [%1]: failed to transform scan: %2. This message should not repeat (tolerance should now be set on our tf2::MessageFilter).").arg(this->getName()).arg(e.what()).toStdString());
    return;
  }

  point_cloud_common_->addMessage(cloud);
}

void EmojiLaserScanDisplay::update(float wall_dt, float ros_dt) {
  point_cloud_common_->update(wall_dt, ros_dt);
}

void EmojiLaserScanDisplay::reset() {
  MFDClass::reset();
  point_cloud_common_->reset();
}

} // namespace emojicloud_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(emojicloud_plugin::EmojiLaserScanDisplay, rviz_common::Display)
