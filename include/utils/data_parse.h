#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <filesystem>
#include <string>

void CopyMeshFromWebToRepo(const std::string& web_path,
                           const std::string& repo_path);

geometry_msgs::TransformStamped ReadHandleMarker(
    const std::string& marker_name = "handle_marker");

void PubPoseToStaticTF(const geometry_msgs::Pose& pose, const std::string& name,
                       const std::string& header_frame = "panda_link0");
