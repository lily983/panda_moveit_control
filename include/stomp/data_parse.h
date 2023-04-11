#pragma once

#include <ros/ros.h>
#include <string>
#include <filesystem>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

void CopyMeshFromWebToRepo(std::string web_path, std::string repo_path)
{
        if(std::filesystem::exists(repo_path)){
            std::filesystem::remove(repo_path);
        }
        std::filesystem::copy_file(web_path, repo_path);
}

geometry_msgs::TransformStamped ReadHandleMarker(std::string marker_name="handle_marker")
{
    ROS_INFO("Read handle marker pose");
    geometry_msgs::TransformStamped tf_world2marker;
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener(tf2_buffer);
    try
    {
        tf_world2marker = tf2_buffer.lookupTransform("panda_link0", marker_name, ros::Time::now(), ros::Duration(3.0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return tf_world2marker;
}