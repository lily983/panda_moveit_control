#include "include/planning_scene/planning_scene_operation.h"

PlanningSceneOperation::PlanningSceneOperation(ros::NodeHandle& nh)
    : nh_(nh), moveit_visual_tools_("panda_link0") {
    pub_trajectory_ =
        nh_.advertise<moveit_msgs::RobotTrajectory>("/visualize_trajectory", 1);
    pub_planning_scene_ = nh_.advertise<moveit_msgs::PlanningScene>(
        "/move_group/monitored_planning_scene", 1, true);

    ROS_INFO("[MoveitPlanningScene]: finished initialization!");
}

PlanningSceneOperation::~PlanningSceneOperation() {}

void PlanningSceneOperation::AddCollisionObj(const std::string& obj_name,
                                             const std::string& mesh_path,
                                             const std::string& header_frame) {
    moveit_msgs::PlanningScene planning_scene;

    moveit_msgs::CollisionObject collision_scene;
    collision_scene.header.frame_id = header_frame;
    collision_scene.header.stamp = ros::Time::now();
    collision_scene.id = obj_name;

    // Add scene mesh
    collision_scene.meshes.resize(1);
    collision_scene.mesh_poses.resize(1);
    collision_scene.mesh_poses.at(0).orientation.w = 1;
    collision_scene.operation = moveit_msgs::CollisionObject::ADD;

    shapes::Mesh* scene_mesh = new shapes::Mesh;
    scene_mesh = shapes::createMeshFromResource(mesh_path);
    shapes::ShapeMsg scene_mesh_msg;
    shapes::constructMsgFromShape(scene_mesh, scene_mesh_msg);
    delete scene_mesh;

    shape_msgs::Mesh scene_mesh_moveit =
        boost::get<shape_msgs::Mesh>(scene_mesh_msg);
    collision_scene.meshes[0].triangles = scene_mesh_moveit.triangles;
    collision_scene.meshes[0].vertices = scene_mesh_moveit.vertices;
    planning_scene.world.collision_objects.push_back(collision_scene);

    planning_scene.is_diff = true;
    pub_planning_scene_.publish(planning_scene);

    ROS_INFO("Publish collision object %s to planning scene", obj_name.c_str());
}

void PlanningSceneOperation::AddCollisionObj(const bool obj_or_scene) {
    if (obj_or_scene == true) {
        ROS_INFO("Remove collision object mesh");
        AddCollisionObj("collision_object",
                        "package://panda_moveit_control/models/mesh_obj.obj");
    } else {
        ROS_INFO("Add collision scene mesh");
        AddCollisionObj("collision_scene",
                        "package://panda_moveit_control/models/mesh_scene.obj");
    }
}

void PlanningSceneOperation::RemoveCollisionObj(
    const std::string& obj_name, const std::string& header_frame) {
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = obj_name;
    remove_object.header.frame_id = header_frame;
    remove_object.operation = remove_object.REMOVE;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    pub_planning_scene_.publish(planning_scene);
    ROS_INFO("Remove collision object %s in planning scene", obj_name.c_str());
}

void PlanningSceneOperation::RemoveCollisionObj(const bool obj_or_scene) {
    if (obj_or_scene == true) {
        ROS_INFO("Remove collision object mesh");
        RemoveCollisionObj("collision_object");
    } else {
        ROS_INFO("Remove collision scene mesh");
        RemoveCollisionObj("collision_scene");
    }
}

void PlanningSceneOperation::VisualizeTrajectory(
    const moveit_msgs::DisplayTrajectory& traj, const int max_num_attempt) {
    int16_t trail = 0;
    while (pub_trajectory_.getNumSubscribers() != 0 &&
           trail < max_num_attempt) {
        ROS_WARN(
            "No subscriber to visualize trajectory, check if Rviz subscribe "
            "the "
            "topic visualize_trajectory...");
        ros::Duration(1.0).sleep();
        trail += 1;
    }
    if (trail < max_num_attempt) {
        pub_trajectory_.publish(traj);
        ROS_INFO("Published trajectory to Rviz for visualization");
    } else {
        ROS_ERROR("Time out to get subscriber to visualize_trajectory in Rviz");
    }
}
