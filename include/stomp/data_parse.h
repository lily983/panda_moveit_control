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
                           const std::string& repo_path) {
    if (std::filesystem::exists(repo_path)) {
        std::filesystem::remove(repo_path);
    }
    std::filesystem::copy_file(web_path, repo_path);
}

geometry_msgs::TransformStamped ReadHandleMarker(
    const std::string& marker_name = "handle_marker") {
    ROS_INFO("Read handle marker pose");
    geometry_msgs::TransformStamped tf_world2marker;
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener(tf2_buffer);
    try {
        tf_world2marker = tf2_buffer.lookupTransform(
            "panda_link0", marker_name, ros::Time::now(), ros::Duration(3.0));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
    }

    return tf_world2marker;
}

void PubPoseToStaticTF(const geometry_msgs::Pose& pose, const std::string& name,
                       const std::string& header_frame = "panda_link0") {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped tf;
    tf.transform.rotation = pose.orientation;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;

    tf.header.frame_id = header_frame;
    tf.header.stamp = ros::Time::now();
    tf.child_frame_id = name;

    static_broadcaster.sendTransform(tf);
    ROS_INFO("Publish tf from %s to %s", header_frame.c_str(), name.c_str());
}
