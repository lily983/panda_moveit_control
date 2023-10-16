#include "include/planning_scene/planning_scene_operation.h"

PlanningSceneOperation::PlanningSceneOperation(ros::NodeHandle& nh)
    : nh_(nh), moveit_visual_tools_("panda_link0") {
    // Subscribe stomp planned trajectory
    std::string stomp_traj_topic;
    nh_.getParam("stomp_trajectory_topic", stomp_traj_topic);
    sub_trajectory_ = nh_.subscribe(
        stomp_traj_topic, 1, &PlanningSceneOperation::CallbackStompTraj, this);

    // Publish the target execution trajectory to rviz
    std::string publish_stomp_trajectory_topic;
    nh_.getParam("publish_stomp_trajectory_topic",
                 publish_stomp_trajectory_topic);
    pub_trajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(
        publish_stomp_trajectory_topic, 1);

    // Operate planning scene
    pub_planning_scene_ = nh_.advertise<moveit_msgs::PlanningScene>(
        "/move_group/monitored_planning_scene", 1, true);

    // Setup service
    srv_visualize_stomp_trajectory_ = nh_.advertiseService(
        "visualize_stomp_trajectory",
        &PlanningSceneOperation::CallbackVisualizeStompTraj, this);

    srv_add_collision_mesh_ = nh_.advertiseService(
        "add_collision_mesh", &PlanningSceneOperation::CallbackAddCollisionMesh,
        this);

    srv_remove_collision_mesh_ = nh_.advertiseService(
        "remove_collision_mesh",
        &PlanningSceneOperation::CallbackRemoveCollisionMesh, this);

    ROS_INFO("[MoveitPlanningScene]: finished initialization!");
}

PlanningSceneOperation::~PlanningSceneOperation() {}

void PlanningSceneOperation::AddCollisionMesh(const std::string& obj_name,
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
    std::string mesh_path;
    mesh_path = "package://panda_moveit_control/models/" + obj_name + ".obj";
    scene_mesh = shapes::createMeshFromResource(mesh_path);
    if (scene_mesh == NULL) {
        ROS_ERROR(
            "!!!No mesh file named %s.obj exists in folder /models, please add "
            "its mesh file to /models before adding it to planning scene!!!",
            obj_name.c_str());
        return;
    }

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

void PlanningSceneOperation::RemoveCollisionMesh(
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

bool PlanningSceneOperation::CallbackVisualizeStompTraj(
    panda_moveit_control::VisualizeStompTraj::Request& req,
    panda_moveit_control::VisualizeStompTraj::Response& res) {
    ROS_INFO("Visualize %d th STOMP trajectory.... ", req.nth_traj);

    moveit_msgs::DisplayTrajectory visualize_stomp_traj;
    visualize_stomp_traj.trajectory.push_back(
        all_stomp_trajectory_.trajectory.at(req.nth_traj));
    visualize_stomp_traj.model_id = all_stomp_trajectory_.model_id;
    visualize_stomp_traj.trajectory_start =
        all_stomp_trajectory_.trajectory_start;
    if (visualize_stomp_traj.trajectory.size() == 0) {
        ROS_ERROR(
            "Failed to get STOMP planned path topic, unable to display "
            "trajectory!!!!");
        res.success = false;
        return false;
    }

    ROS_INFO("Publish the %d th STOMP trajectory to rviz....", req.nth_traj);
    pub_trajectory_.publish(visualize_stomp_traj);

    return true;
}

bool PlanningSceneOperation::CallbackAddCollisionMesh(
    panda_moveit_control::AddCollisionMesh::Request& req,
    panda_moveit_control::AddCollisionMesh::Response& res) {
    ROS_INFO(
        "Add mesh-formatted collision object named as %s into planning "
        "scene....",
        req.mesh_collision_name.c_str());
    AddCollisionMesh(req.mesh_collision_name, "panda_link0");
    return true;
}

bool PlanningSceneOperation::CallbackRemoveCollisionMesh(
    panda_moveit_control::RemoveCollisionMesh::Request& req,
    panda_moveit_control::RemoveCollisionMesh::Response& res) {
    ROS_INFO("Remove collision object named as %s from planning scene....",
             req.mesh_collision_name.c_str());
    RemoveCollisionMesh(req.mesh_collision_name, "panda_link0");
    return true;
}

void PlanningSceneOperation::CallbackStompTraj(
    const moveit_msgs::DisplayTrajectoryPtr& msg) {
    all_stomp_trajectory_ = *msg;
}
