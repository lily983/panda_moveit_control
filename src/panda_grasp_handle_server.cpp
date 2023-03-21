/* This node provide service to control franka arm to grasp the handle of drawer with marker on it. 
*/

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <string>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <panda_moveit_control/PandaGraspHandle.h>
#include <panda_moveit_control/GraspHandleInfo.h>

#include <franka_gripper/MoveAction.h>
#include <franka_gripper/HomingAction.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <geometric_shapes/shape_operations.h>

#define PI 3.1415926
typedef Eigen::Transform<double, 3, Eigen::Affine> T;

class PandaGraspHandleServer
{
    public:
    PandaGraspHandleServer(ros::NodeHandle& nh)
    : n(nh)
    , tf2_listener_(tf2_buffer_)
    , move_group_("panda_arm")
    , moveAC("franka_gripper/move")
    , homingAC("franka_gripper/homing")
    , moveit_visual_tools_("panda_link0")
    , planning_scene_interface_("", true)
    {
        // std::cout << "1" << "\n";
        ss_grasp_handle_ = n.advertiseService("panda_grasp_handle", &PandaGraspHandleServer::callBackPandaGraspHandle, this);

        // std::cout << "2" << "\n";
        ss_grasp_info_ = n.advertiseService("generate_grasp_info", &PandaGraspHandleServer::callBackGenerateGraspInfo, this);
        
        // std::cout << "3" << "\n";
        initializedFixedTransformations();
        ROS_INFO("Finished initialize fixed transformation");

        while(!moveAC.waitForServer(ros::Duration(5.0))){
            ROS_ERROR("Failed to wait for franka_gripper move server");
        }
        while(!homingAC.waitForServer(ros::Duration(5.0))){
            ROS_ERROR("Failed to wait for franka_gripper homing server");
        }
        ROS_INFO("Ready to send move/homing action to gripper...");

        sub_trajectory_ = n.subscribe("/server_stomp_planning/display_planned_path", 1, &PandaGraspHandleServer::callBackTrajectory, this);

        sub_aruco_marker_ = n.subscribe("/aruco_double/pose2", 1, &PandaGraspHandleServer::callBackAruco, this);

        sub_obstacle_marker_ = n.subscribe("/aruco_double/pose", 1, &PandaGraspHandleServer::callBackObstacleAruco, this);

        rviz_.reset(new rviz_visual_tools::RvizVisualTools("panda_link0", "/rviz_visual_tools"));

        planning_scene_diff_publisher_ = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
        ros::WallDuration sleep_t(0.5);
        while (planning_scene_diff_publisher_.getNumSubscribers() < 1)
        {
            sleep_t.sleep();
            ROS_INFO("Wait for planning scene subscriber...");
        }

        n.getParam("scene_mesh_path", mesh_path_);
        AddCollisionSceneMesh(mesh_path_);

        
    }

    void callBackObstacleAruco(const geometry_msgs::PosePtr& pose_c2o)
    {
        geometry_msgs::TransformStamped tf_w2c;
        try
        {
            tf_w2c = tf2_buffer_.lookupTransform("panda_link0", "camera_base", ros::Time::now(), ros::Duration(3.0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
        T ei_w2c;
        tf::transformMsgToEigen(tf_w2c.transform, ei_w2c);

        T ei_c2o;
        tf::poseMsgToEigen(*pose_c2o, ei_c2o);

        T ei_w2o;
        ei_w2o = ei_w2c * ei_c2o;
        tf::poseEigenToMsg(ei_w2o, obstacle_pose_);

        tf::transformEigenToMsg(ei_w2o, tf_w2o_.transform);
        tf_w2o_.header.frame_id = "pand_link0";
        tf_w2o_.header.stamp = tf_w2c.header.stamp;
        tf_w2o_.child_frame_id = "obstacle_frame";
        // ROS_INFO("Receive obstcale marker frame");

        addCollisionObjects();
        // ROS_INFO("Initialized collision objects in the planning scene");

    }

    void callBackAruco(const geometry_msgs::PosePtr& pose_c2d)
    {   
        geometry_msgs::TransformStamped tf_w2c;
        try
        {
            tf_w2c = tf2_buffer_.lookupTransform("panda_link0", "camera_base", ros::Time::now(), ros::Duration(3.0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            // return;
        }


        T ei_w2c;
        tf::transformMsgToEigen(tf_w2c.transform, ei_w2c);

        T ei_c2d;
        tf::poseMsgToEigen(*pose_c2d, ei_c2d);

        T ei_w2d;
        ei_w2d = ei_w2c * ei_c2d;

        T ei_w2L8;
        ei_w2L8 = ei_w2d * ei_d2L8_;
        tf::poseEigenToMsg(ei_w2L8, grasp_pose_);

        T ei_w2L8_post_grasp;
        ei_w2L8_post_grasp = ei_w2d * ei_d2L8_post_grasp_;
        tf::poseEigenToMsg(ei_w2L8_post_grasp, post_grasp_pose_);

        T ei_w2L8_pre_grasp;
        ei_w2L8_pre_grasp = ei_w2d * ei_d2L8_pre_grasp_;
        tf::poseEigenToMsg(ei_w2L8_pre_grasp, pre_grasp_pose_);

        tf::transformEigenToMsg(ei_w2d, tf_w2d_.transform);
        tf_w2d_.header.frame_id = "panda_link0";
        tf_w2d_.header.stamp = tf_w2c.header.stamp;
        tf_w2d_.child_frame_id = "aruco_marker_frame";

        // ROS_INFO("Receive aruco marker frame");

        // printPose(grasp_pose_);
        // std::cout<< tf_w2d_.transform.translation << "\n";
        // std::cout <<  tf_w2d_.transform.rotation.x  << tf_w2d_.transform.rotation.y  <<tf_w2d_.transform.rotation.z  <<tf_w2d_.transform.rotation.w  << "\n";
        publishGraspInfoToMoveit();
        
    }

    bool callBackGenerateGraspInfo(panda_moveit_control::GraspHandleInfo::Request& req,
                                panda_moveit_control::GraspHandleInfo::Response& res)
    {
        ROS_INFO("Received generate grasp info request...");

        try
        {
            tf_w2d_ = tf2_buffer_.lookupTransform("panda_link0", "aruco_marker_frame", ros::Time::now(), ros::Duration(2.0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            res.success = 0;
            return 0;
        }

        generateGraspInfo(tf_w2d_);

        // publish target_grasp_pose for primp server
        geometry_msgs::TransformStamped tf_grasp_pose;
        tf_grasp_pose.transform.rotation = grasp_pose_.orientation;
        tf_grasp_pose.transform.translation.x = grasp_pose_.position.x;
        tf_grasp_pose.transform.translation.y = grasp_pose_.position.y;
        tf_grasp_pose.transform.translation.z = grasp_pose_.position.z;

        tf_grasp_pose.header.frame_id = "panda_link0";
        tf_grasp_pose.header.stamp = ros::Time::now();
        tf_grasp_pose.child_frame_id = "target_grasp_pose";

        tf2_br_.sendTransform(tf_grasp_pose);

        res.success = 1;
        return 1;
        
    }

    void initializedFixedTransformations()
    {

        Eigen::Quaterniond q_d2L8(-0.000, 0.925, 0.379, 0.023);
        ei_d2L8_ = Eigen::Translation3d(Eigen::Vector3d(-0.010, -0.120, 0.053)) * q_d2L8.normalized().toRotationMatrix();

        Eigen::Quaterniond q_d2L8_post_grasp(-0.000, 0.925, 0.379, 0.023);
        ei_d2L8_post_grasp_ = Eigen::Translation3d(Eigen::Vector3d(-0.010, -0.220, 0.053)) * q_d2L8_post_grasp.normalized().toRotationMatrix();

        Eigen::Quaterniond q_d2L8_pre_grasp(-0.000, 0.925, 0.379, 0.023);
        ei_d2L8_pre_grasp_ = Eigen::Translation3d(Eigen::Vector3d(-0.010, -0.120, 0.3)) * q_d2L8_pre_grasp.normalized().toRotationMatrix();
       
    }

    // Receive the robot trajectory planned by STOMP, use that as the second executed trajectory
    // Planning a grasping trajectory based on the start joint configuration of the second executed trajectory
    void callBackTrajectory(const moveit_msgs::DisplayTrajectoryPtr& msg)
    {
        stomp_trajectory_.joint_trajectory.points.clear();
        stomp_trajectory_ = msg->trajectory.at(0);
        ROS_INFO("Received the STOMP planned trajectpsy...");
    }

    void publishGraspInfoToMoveit()
    {
        // publish target_grasp_pose for primp server
        geometry_msgs::TransformStamped tf_grasp_pose;
        tf_grasp_pose.transform.rotation = grasp_pose_.orientation;
        tf_grasp_pose.transform.translation.x = grasp_pose_.position.x;
        tf_grasp_pose.transform.translation.y = grasp_pose_.position.y;
        tf_grasp_pose.transform.translation.z = grasp_pose_.position.z;

        tf_grasp_pose.header.frame_id = "panda_link0";
        tf_grasp_pose.header.stamp = ros::Time::now();
        tf_grasp_pose.child_frame_id = "target_grasp_pose";

        // publish post_grasp_pose for primp server
        geometry_msgs::TransformStamped tf_post_grasp_pose;
        tf_post_grasp_pose.transform.rotation = post_grasp_pose_.orientation;
        tf_post_grasp_pose.transform.translation.x = post_grasp_pose_.position.x;
        tf_post_grasp_pose.transform.translation.y = post_grasp_pose_.position.y;
        tf_post_grasp_pose.transform.translation.z = post_grasp_pose_.position.z;

        tf_post_grasp_pose.header.frame_id = "panda_link0";
        tf_post_grasp_pose.header.stamp = ros::Time::now();
        tf_post_grasp_pose.child_frame_id = "goal_pose";

        // publish pre_grasp_pose for primp server
        geometry_msgs::TransformStamped tf_pre_grasp_pose;
        tf_pre_grasp_pose.transform.rotation = pre_grasp_pose_.orientation;
        tf_pre_grasp_pose.transform.translation.x = pre_grasp_pose_.position.x;
        tf_pre_grasp_pose.transform.translation.y = pre_grasp_pose_.position.y;
        tf_pre_grasp_pose.transform.translation.z = pre_grasp_pose_.position.z;

        tf_pre_grasp_pose.header.frame_id = "panda_link0";
        tf_pre_grasp_pose.header.stamp = ros::Time::now();
        tf_pre_grasp_pose.child_frame_id = "pre_grasp_pose";

        tf2_br_.sendTransform(tf_grasp_pose);
        tf2_br_.sendTransform(tf_post_grasp_pose);
        tf2_br_.sendTransform(tf_pre_grasp_pose);

        rviz_->publishAxisLabeled(post_grasp_pose_,"goal_pose", rviz_visual_tools::LARGE, rviz_visual_tools::GREEN);
        rviz_->publishAxisLabeled(grasp_pose_,"grasp_pose", rviz_visual_tools::LARGE, rviz_visual_tools::GREEN);
        rviz_->publishAxisLabeled(pre_grasp_pose_,"pre_grasp_pose", rviz_visual_tools::LARGE, rviz_visual_tools::GREEN);
        rviz_->trigger();
    }

// When receive the request to grasp the handle of the drawer, service callback function listen to the drawer marker pose and convert it to the 
// target grasp message(contained pre-grasp, grasp, post-grasp info). Then it calls the arm to execute the grasp
    bool callBackPandaGraspHandle(panda_moveit_control::PandaGraspHandle::Request& req,
                                panda_moveit_control::PandaGraspHandle::Response& res)
    {
        ROS_INFO("Received the grasping service call...");


        // trajectory_msgs::JointTrajectoryPoint stomp_start_point;
        // stomp_start_point = stomp_trajectory_.joint_trajectory.points.at(0);

        // moveit::planning_interface::MoveGroupInterface::Plan  my_plan;
        // const robot_state::JointModelGroup* joint_model_group =
        //     move_group_.getCurrentState()->getJointModelGroup("panda_arm");

        // if(stomp_trajectory_.joint_trajectory.points.size() == 0){
        //     ROS_ERROR("Haven't received STOMP trajectroy ");
        //     res.success = false;
        //     return 0;
        // }

        // // set robot state to the first stomp joint trajectory point 
        // moveit::core::RobotStatePtr start_state = move_group_.getCurrentState();

        // moveit::core::RobotStatePtr stomp_start_state = move_group_.getCurrentState();
        
        // std::vector<double> joint_group_positions;
        // joint_group_positions = stomp_start_point.positions;

        // stomp_start_state->setJointGroupPositions(joint_model_group, joint_group_positions);

        // move_group_.setStartState(*stomp_start_state);
        // ROS_INFO("Reverse planning, set start state to the first stomp joint trajectory point ");

        // std::vector<double> pre_grasp_pose_joint_values;
        

        // // set pose target to pre-grasp pose
        // move_group_.setPoseTarget(pre_grasp_pose_);
        // if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        // {
        //     pre_grasp_pose_joint_values = my_plan.trajectory_.joint_trajectory.points.at(my_plan.trajectory_.joint_trajectory.points.size() - 1).positions;
        //     ROS_INFO("Successfully planned reverse planning");
        //     moveit_visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        //     moveit_visual_tools_.trigger();
        //     // move_group_.execute(my_plan.trajectory_);
        // }
        // else{
        //     ROS_ERROR("Failed to plan in a reverse way");
        //     res.success = 0;
        //     return 0;
        // }

        // // Plan from start configuration to the pre_grasp_pose_joint_values

        // move_group_.setStartState(*start_state);
        // ROS_INFO("Start plan from arm start configuration to pre_grasp_pose_joint_values");
        // move_group_.setJointValueTarget(pre_grasp_pose_joint_values);

        // addCollisionDrawer();
        // ros::Duration(1.0).sleep();

        // if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        // {
        //     pre_grasp_trajectory_ = my_plan.trajectory_;
        //     ROS_INFO("Successfully planned from arm start configuration to pre_grasp_pose_joint_values");
        //     moveit_visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        //     moveit_visual_tools_.trigger();
        //     // move_group_.execute(my_plan.trajectory_);
        // }
        // else{
        //     ROS_ERROR("Failed to plan from arm start configuration to pre_grasp_pose_joint_values");
        //     res.success = 0;
        //     return 0;
        // }

        // moveit::core::RobotStatePtr pre_grasp_finished_state = move_group_.getCurrentState();
        // pre_grasp_finished_state->setJointGroupPositions(joint_model_group, pre_grasp_pose_joint_values);
        // move_group_.setStartState(*pre_grasp_finished_state);

        // move_group_.setJointValueTarget(stomp_start_point.positions);

        // removeCollisionDrawer();
        // ros::Duration(2.0).sleep();

        // if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        // {
        //     pre_grasp_2_grasp_trajectory_ = my_plan.trajectory_;
        //     moveit_visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        //     moveit_visual_tools_.trigger();
        //     // move_group_.execute(my_plan.trajectory_);
        // }
        // else{
        //     ROS_ERROR("Failed to plan from arm start configuration to pre_grasp_pose_joint_values");
        //     res.success = 0;
        //     return 0;
        // }
        
    
        // moveit_visual_tools_.publishTrajectoryLine(pre_grasp_trajectory_, joint_model_group);
        // moveit_visual_tools_.publishTrajectoryLine(pre_grasp_2_grasp_trajectory_, joint_model_group);
        // moveit_visual_tools_.publishTrajectoryLine(stomp_trajectory_, joint_model_group);
        // moveit_visual_tools_.trigger();


        // move_group_.setStartState(*start_state);
        // move_group_.execute(pre_grasp_trajectory_);

        // ros::Duration(1.0).sleep();

        // move_group_.execute(pre_grasp_2_grasp_trajectory_);

        // ros::Duration(0.5).sleep();
        // franka_gripper::MoveGoal move_goal;
        // move_goal.width = 0.03;
        // move_goal.speed = 0.03;
        // moveAC.sendGoal(move_goal);
        // if(moveAC.waitForResult(ros::Duration(3.0)))
        // {
        //     ROS_INFO("Franka gripper sucessfully complish move action.");
        // }
        // else
        // {
        //     ROS_ERROR("Franka gripper failed to complish move action.");
        // }

        // ros::Duration(0.5).sleep();

        trajectory_msgs::JointTrajectoryPoint stomp_start_point;
        stomp_start_point = stomp_trajectory_.joint_trajectory.points.at(0);

        moveit::planning_interface::MoveGroupInterface::Plan  my_plan;
        const robot_state::JointModelGroup* joint_model_group =
            move_group_.getCurrentState()->getJointModelGroup("panda_arm");

        move_group_.setJointValueTarget(stomp_start_point.positions);
        if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            move_group_.execute(my_plan.trajectory_);
            ROS_INFO("Reached start point, hand over cup ...");
        }
        else{
            ROS_ERROR("Failed to plan from arm start configuration to stomp first points");
            res.success = 0;
            return 0;
        }

        ros::Duration(10.0).sleep();
        franka_gripper::MoveGoal move_goal;
        move_goal.width = 0.05;
        move_goal.speed = 0.01;
        moveAC.sendGoal(move_goal);
        if(moveAC.waitForResult(ros::Duration(3.0)))
        {
            ROS_INFO("Franka gripper sucessfully complish move action.");
        }
        else
        {
            ROS_ERROR("Franka gripper failed to complish move action.");
        }

        ros::Duration(5.0).sleep();

        move_group_.execute(stomp_trajectory_);

        ros::Duration(1.0).sleep();

        res.success = 1;
        return 1;



        /*

        moveit::core::RobotStatePtr start_state = move_group_.getCurrentState();
        // Generate pre-grasp trajectory
        addCollisionDrawer();
        ros::Duration(0.5).sleep();
        // move to pre_grasp_pose
        move_group_.setPoseTarget(pre_grasp_pose_);
        if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            pre_grasp_trajectory_ = my_plan.trajectory_;
            ROS_INFO("Successfully planned pre-grasp trajectory");
            moveit_visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            moveit_visual_tools_.trigger();
            // move_group_.execute(my_plan.trajectory_);
        }
        else
        {
            ROS_ERROR("Failed to plan a path to the pre-grasp pose...");

            removeCollisionDrawer();
            ros::Duration(1).sleep();
            ROS_INFO("Pre-grasp not work for current setting");

            move_group_.setStartState(*start_state);
            move_group_.setJointValueTarget(stomp_start_point.positions);

            if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                direct_grasp_trajectory_ = my_plan.trajectory_;
            }
            else{
                ROS_ERROR("Failed to plan a path to the start joint configuration of STOMP trajectory...");
                return 0;
            }

            moveit_msgs::RobotTrajectory display_trajectory;
            display_trajectory = direct_grasp_trajectory_;
            display_trajectory.joint_trajectory.points.insert(display_trajectory.joint_trajectory.points.end(), 
                        stomp_trajectory_.joint_trajectory.points.begin(), stomp_trajectory_.joint_trajectory.points.end());
            moveit_visual_tools_.publishTrajectoryLine(display_trajectory, joint_model_group);
            moveit_visual_tools_.trigger();

             if(move_group_.execute(direct_grasp_trajectory_) == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                ros::Duration(0.5).sleep();
                franka_gripper::MoveGoal move_goal;
                move_goal.width = 0.03;
                move_goal.speed = 0.03;
                moveAC.sendGoal(move_goal);
                if(moveAC.waitForResult(ros::Duration(3.0)))
                {
                    ROS_INFO("Franka gripper sucessfully complish move action.");
                }
                else
                {
                    ROS_ERROR("Franka gripper failed to complish move action.");
                }

                ros::Duration(0.5).sleep();

                if(move_group_.execute(stomp_trajectory_) == moveit_msgs::MoveItErrorCodes::SUCCESS)
                {
                    ROS_INFO("Successfully execute all trajectory...");
                    res.success = 1;
                    return 1;
                }
                else{
                    res.success = 0;
                    return 0;
                }
            }
        }
        ros::Duration(2).sleep();
        removeCollisionDrawer();
        ros::Duration(1).sleep();
        
        // Use this to update robot state
        moveit::core::RobotStatePtr current_state = move_group_.getCurrentState();
        std::vector<double> joint_group_positions;
        // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        // for(auto i: joint_group_positions)
        //     std::cout << i << ' ';
        // std::cout << "\n";

        joint_group_positions = pre_grasp_trajectory_.joint_trajectory.points.back().positions;
        // for(auto i: joint_group_positions)
        //     std::cout << i << ' ';
        // std::cout << "\n";
    
        current_state->setJointGroupPositions(joint_model_group, joint_group_positions);

        move_group_.setStartState(*current_state);
        ROS_INFO("Set robot state to a new state");
        
        if(stomp_trajectory_.joint_trajectory.points.size() == 0){
            ROS_ERROR("Haven't received STOMP trajectroy ");
            res.success = false;
            return 0;
        }

        //Set joint value target to the end joint configuration of STOMP trajectory
        move_group_.setJointValueTarget(stomp_start_point.positions);

        if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            pre_grasp_2_grasp_trajectory_ = my_plan.trajectory_;
            ROS_INFO("Successfully planned pre_grasp_2_grasp_trajectory_");
            moveit_visual_tools_.publishTrajectoryLine( my_plan.trajectory_, joint_model_group);
            moveit_visual_tools_.trigger();
            // std::cout<< "STOMP trajectory points number is: " << stomp_trajectory_.joint_trajectory.points.size() << "\n";
            // std::cout<< "Execute trajectory points number is: " << execute_trajectory_.joint_trajectory.points.size() << "\n";
        }
        else
        {
            ROS_ERROR("Failed to plan a path from pre-grasp pose to the start joint configuration of STOMP trajectory...");
            return 0;
        }

        franka_gripper::HomingGoal homing_goal;
        homingAC.sendGoalAndWait(homing_goal, ros::Duration(0.5));

        if(pre_grasp_2_grasp_trajectory_.joint_trajectory.points.size() <= 100)
        {
            ROS_INFO("Pre-grasp works for current setting");

            moveit_msgs::RobotTrajectory display_trajectory;
            display_trajectory = pre_grasp_trajectory_;
            display_trajectory.joint_trajectory.points.insert(display_trajectory.joint_trajectory.points.end(), 
                        pre_grasp_2_grasp_trajectory_.joint_trajectory.points.begin(), pre_grasp_2_grasp_trajectory_.joint_trajectory.points.end());
            display_trajectory.joint_trajectory.points.insert(display_trajectory.joint_trajectory.points.end(), 
                        stomp_trajectory_.joint_trajectory.points.begin(), stomp_trajectory_.joint_trajectory.points.end());
            moveit_visual_tools_.publishTrajectoryLine(display_trajectory, joint_model_group);
            moveit_visual_tools_.trigger();

            if(move_group_.execute(pre_grasp_trajectory_) == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                if(move_group_.execute(pre_grasp_2_grasp_trajectory_) == moveit_msgs::MoveItErrorCodes::SUCCESS)
                {
                    ros::Duration(0.5).sleep();
                    franka_gripper::MoveGoal move_goal;
                    move_goal.width = 0.03;
                    move_goal.speed = 0.03;
                    moveAC.sendGoal(move_goal);
                    if(moveAC.waitForResult(ros::Duration(3.0)))
                    {
                        ROS_INFO("Franka gripper sucessfully complish move action.");
                    }
                    else
                    {
                        ROS_ERROR("Franka gripper failed to complish move action.");
                    }

                    ros::Duration(0.5).sleep();
                

                    if(move_group_.execute(stomp_trajectory_) == moveit_msgs::MoveItErrorCodes::SUCCESS)
                    {
                        ROS_INFO("Successfully execute all trajectory...");
                        res.success = 1;
                        return 1;
                    }
                    else{
                        res.success = 0;
                        return 0;
                    }
                }
            }
        }
        else{
            ROS_INFO("Pre-grasp not work for current setting");

            move_group_.setStartState(*start_state);
            move_group_.setJointValueTarget(stomp_start_point.positions);

            if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                direct_grasp_trajectory_ = my_plan.trajectory_;
            }
            else{
                ROS_ERROR("Failed to plan a path to the start joint configuration of STOMP trajectory...");
                return 0;
            }

            moveit_msgs::RobotTrajectory display_trajectory;
            display_trajectory = direct_grasp_trajectory_;
            display_trajectory.joint_trajectory.points.insert(display_trajectory.joint_trajectory.points.end(), 
                        stomp_trajectory_.joint_trajectory.points.begin(), stomp_trajectory_.joint_trajectory.points.end());
            moveit_visual_tools_.publishTrajectoryLine(display_trajectory, joint_model_group);
            moveit_visual_tools_.trigger();

             if(move_group_.execute(direct_grasp_trajectory_) == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                ros::Duration(0.5).sleep();
                franka_gripper::MoveGoal move_goal;
                move_goal.width = 0.03;
                move_goal.speed = 0.03;
                moveAC.sendGoal(move_goal);
                if(moveAC.waitForResult(ros::Duration(3.0)))
                {
                    ROS_INFO("Franka gripper sucessfully complish move action.");
                }
                else
                {
                    ROS_ERROR("Franka gripper failed to complish move action.");
                }

                ros::Duration(0.5).sleep();

                if(move_group_.execute(stomp_trajectory_) == moveit_msgs::MoveItErrorCodes::SUCCESS)
                {
                    ROS_INFO("Successfully execute all trajectory...");
                    res.success = 1;
                    return 1;
                }
                else{
                    res.success = 0;
                    return 0;
                }
            }
        }


        // if(move_group_.execute(execute_trajectory_) == moveit_msgs::MoveItErrorCodes::FAILURE)
        // {
        //     ROS_ERROR("Failed to exeucte the grasping trajectory");
        //     res.success = 0;
        //     return 0;
        // }

        // move_group_.setPoseTarget(grasp_pose_);
        // if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        // {
        //     moveit_visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        //     moveit_visual_tools_.trigger();
        //     if(move_group_.execute(my_plan.trajectory_) == moveit_msgs::MoveItErrorCodes::FAILURE)
        //     {
        //         ROS_ERROR("Failed to exeucte the grasping trajectory");
        //         res.success = 0;
        //         return 0;
        //     }
        // }
        // else
        // {
        //     ROS_ERROR("Failed to plan a path to the goal pose...");
        //     return 0;
        // }

        // Close finger
        // franka_gripper::MoveGoal move_goal;
        // move_goal.width = 0.03;
        // move_goal.speed = 0.03;
        // moveAC.sendGoal(move_goal);
        // if(moveAC.waitForResult(ros::Duration(3.0)))
        // {
        //     ROS_INFO("Franka gripper sucessfully complish move action.");
        // }
        // else
        // {
        //     ROS_ERROR("Franka gripper failed to complish move action.");
        // }

        // ros::Duration(0.5).sleep();

        // If the current robot state is not aligned with the first point of trajectory,
        // we insert the current robot state as the new first point of trajectory
        // moveit::core::RobotStatePtr current_state = move_group_.getCurrentState();
        
        // trajectory_msgs::JointTrajectoryPoint first_point;
        // first_point.positions = stomp_trajectory_.joint_trajectory.points[0].positions;
        // if(first_point.positions != move_group_.getCurrentJointValues())
        // {
        //     first_point.positions = move_group_.getCurrentJointValues();
        //     stomp_trajectory_.joint_trajectory.points.insert(stomp_trajectory_.joint_trajectory.points.begin(), first_point);
        // }

        // move_group_.setJointValueTarget(second_tra_start_point.positions);
        // if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        // {
        //     move_group_.execute(my_plan.trajectory_);
        //     ROS_INFO("Finished grasp end joint configuration to STOMP first joint configuration");
        //     std::cout << my_plan.trajectory_.joint_trajectory.points.size() <<"\n";
        // }

        // if(move_group_.execute(stomp_trajectory_) == moveit_msgs::MoveItErrorCodes::FAILURE)
        // {
        //     ROS_ERROR("Failed to execute the STOMP trajectory");
        //     res.success = 0;
        //     return 0;
        // }
        // else{
        //     ROS_INFO("Successfully grasped the handle and execute the STOMP trajectory");
        // }
        
        
        // move_group_.setPoseTarget(post_grasp_pose_);

        // if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        // {
        //     moveit_visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        //     moveit_visual_tools_.trigger();
        //     move_group_.execute(my_plan.trajectory_);
        // }
        // else
        // {
        //     ROS_ERROR("Failed to plan a path to the goal pose...");
        //     return 0;
        // }

*/
        

}

// Generate grasp info from the input transformation from world to drawer  
    void generateGraspInfo(geometry_msgs::TransformStamped T_w2d)
    {
        T ei_w2d;
        tf::transformMsgToEigen(T_w2d.transform, ei_w2d);
        geometry_msgs::Pose pose_w2d;
        pose_w2d.orientation = T_w2d.transform.rotation;
        pose_w2d.position.x = T_w2d.transform.translation.x;
        pose_w2d.position.y = T_w2d.transform.translation.y;
        pose_w2d.position.z = T_w2d.transform.translation.z;

        T ei_w2L8;
        ei_w2L8 = ei_w2d * ei_d2L8_;

        tf::poseEigenToMsg(ei_w2L8, grasp_pose_);

        // pre-grasp pose is higher than grasp pose in z-axis relative to world frame
        pre_grasp_pose_ = grasp_pose_;
        pre_grasp_pose_.position.z = grasp_pose_.position.z + 0.1;

        // Publish pose in rviz
        rviz_.reset(new rviz_visual_tools::RvizVisualTools("panda_link0", "/rviz_visual_markers"));
        rviz_->deleteAllMarkers();

        rviz_->publishAxisLabeled(pose_w2d,"drawer_pose", rviz_visual_tools::LARGE, rviz_visual_tools::GREEN);
        rviz_->publishAxisLabeled(pre_grasp_pose_,"grasp_pose", rviz_visual_tools::LARGE, rviz_visual_tools::GREEN);
        rviz_->publishAxisLabeled(grasp_pose_,"grasp_pose", rviz_visual_tools::LARGE, rviz_visual_tools::GREEN);
        rviz_->trigger();

        geometry_msgs::TransformStamped tf_grasp_pose;
        tf_grasp_pose.transform.rotation = grasp_pose_.orientation;
        tf_grasp_pose.transform.translation.x = grasp_pose_.position.x;
        tf_grasp_pose.transform.translation.y = grasp_pose_.position.y;
        tf_grasp_pose.transform.translation.z = grasp_pose_.position.z;

        tf_grasp_pose.header.frame_id = "panda_link0";
        tf_grasp_pose.header.stamp = T_w2d.header.stamp;
        tf_grasp_pose.child_frame_id = "target_grasp_pose";

        tf2_br_.sendTransform(tf_grasp_pose);

    }

    void printPose(geometry_msgs::Pose pose){
        ROS_INFO("The position xyz is: [%f, %f, %f]",pose.position.x, pose.position.y, pose.position.z);
        ROS_INFO("The quaternion xyzw is: [%f, %f, %f, %f]", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }

    void armMove(geometry_msgs::Pose target_pose){
        moveit::planning_interface::MoveGroupInterface::Plan  my_plan;
        
        const robot_state::JointModelGroup* joint_model_group =
            move_group_.getCurrentState()->getJointModelGroup("panda_arm");

        move_group_.setPoseTarget(target_pose);
        bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success)
        {
            moveit_visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            move_group_.move();
        }
        else{
            ROS_INFO("Failed to find solution to move the arm to the target pose");
        }
    }

    // Add table and obstacle to the planning scene
    void addCollisionObjects()
    {
        // ROS_INFO("Adding collision objects...");

        // geometry_msgs::Pose::ConstPtr obstacle_pose_ptr;
        // obstacle_pose_ptr = ros::topic::waitForMessage<geometry_msgs::Pose>("/aruco_double/pose", ros::Duration(10.0));
        // tf_w2o_.transform.translation.x = obstacle_pose_ptr->position.x;
        // tf_w2o_.transform.translation.y = obstacle_pose_ptr->position.y;
        // tf_w2o_.transform.translation.z = obstacle_pose_ptr->position.z;
        // tf_w2o_.transform.rotation = obstacle_pose_ptr->orientation;

        // Creating Environment
        // ^^^^^^^^^^^^^^^^^^^^
        // Create vector to hold collision objects.
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(2);

        // Add the first table where the cube will originally be kept.
        collision_objects[0].id = "table";
        collision_objects[0].header.frame_id = "world";

        /* Define the primitive and its dimensions. */
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        collision_objects[0].primitives[0].dimensions[0] = 1.0;
        collision_objects[0].primitives[0].dimensions[1] = 0.8;
        collision_objects[0].primitives[0].dimensions[2] = 0.1;

        /* Define the pose of the table. */
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.8;
        collision_objects[0].primitive_poses[0].position.y = 0.0;
        collision_objects[0].primitive_poses[0].position.z = -0.05;
        collision_objects[0].primitive_poses[0].orientation.w = 1.0;
        // END_SUB_TUTORIAL

        collision_objects[0].operation = collision_objects[0].ADD;

        // obstacle box
        // Add the second table where we will be placing the cube.
        collision_objects[1].id = "obstacle";
        collision_objects[1].header.frame_id = "world";

        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
        collision_objects[1].primitives[0].dimensions.resize(3);
        collision_objects[1].primitives[0].dimensions[0] = 0.15;
        collision_objects[1].primitives[0].dimensions[1] = 0.6;
        collision_objects[1].primitives[0].dimensions[2] = 0.39;

        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = tf_w2o_.transform.translation.x ;
        collision_objects[1].primitive_poses[0].position.y = tf_w2o_.transform.translation.y;
        collision_objects[1].primitive_poses[0].position.z = tf_w2o_.transform.translation.z - 0.195;
        collision_objects[1].primitive_poses[0].orientation.w = tf_w2o_.transform.rotation.w;
        collision_objects[1].primitive_poses[0].orientation.x = tf_w2o_.transform.rotation.x;
        collision_objects[1].primitive_poses[0].orientation.y = tf_w2o_.transform.rotation.y;
        collision_objects[1].primitive_poses[0].orientation.z = tf_w2o_.transform.rotation.z;

        collision_objects[1].operation = collision_objects[1].ADD;

        // printPose(collision_objects[1].primitive_poses[0]);

        // planning_scene_interface_.applyCollisionObjects(collision_objects);

        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(collision_objects[0]);
        planning_scene.world.collision_objects.push_back(collision_objects[1]);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher_.publish(planning_scene);
        // ROS_INFO("Publish collision objects in planning scene");
    }

    void addCollisionDrawer()
    {
        
        moveit_msgs::CollisionObject collision_drawer;
        collision_drawer.id = "drawer";
        collision_drawer.header.frame_id = "world";

        collision_drawer.primitives.resize(1);
        collision_drawer.primitives[0].type = collision_drawer.primitives[0].BOX;
        collision_drawer.primitives[0].dimensions.resize(3);
        collision_drawer.primitives[0].dimensions[0] = 0.135;
        collision_drawer.primitives[0].dimensions[1] = 0.185;
        collision_drawer.primitives[0].dimensions[2] = 0.27;

        collision_drawer.primitive_poses.resize(1);
        collision_drawer.primitive_poses[0].position.x = tf_w2d_.transform.translation.x ;
        collision_drawer.primitive_poses[0].position.y = tf_w2d_.transform.translation.y;
        collision_drawer.primitive_poses[0].position.z = tf_w2d_.transform.translation.z - 0.135;
        collision_drawer.primitive_poses[0].orientation.w = 1;
        collision_drawer.primitive_poses[0].orientation.x = 0;
        collision_drawer.primitive_poses[0].orientation.y = 0;
        collision_drawer.primitive_poses[0].orientation.z = 0;
        collision_drawer.operation = collision_drawer.ADD;
        // planning_scene_interface_.applyCollisionObject(collision_drawer);
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(collision_drawer);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher_.publish(planning_scene);

        ROS_INFO("Publish drawer collision to planning scene");
    }

    void removeCollisionDrawer()
    {
        moveit_msgs::CollisionObject remove_object;
        remove_object.id = "drawer";
        remove_object.header.frame_id = "world";
        remove_object.operation = remove_object.REMOVE;

        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.world.collision_objects.clear();
        planning_scene.world.collision_objects.push_back(remove_object);
        planning_scene_diff_publisher_.publish(planning_scene);
        ROS_INFO("Remove collision drawer...");
    }

    void AddCollisionSceneMesh(std::string mesh_path)
    {
        moveit_msgs::CollisionObject collision_scene;
        collision_scene.header.frame_id = "world";
        collision_scene.header.stamp = ros::Time::now();
        collision_scene.id = "collision_scene";
        
        // Add scene mesh
        collision_scene.meshes.resize(1);
        collision_scene.mesh_poses.resize(1);
        collision_scene.operation = moveit_msgs::CollisionObject::ADD;

        shapes::Mesh* scene_mesh = shapes::createMeshFromResource(mesh_path);
        shapes::ShapeMsg scene_mesh_msg;
        shapes::constructMsgFromShape(scene_mesh, scene_mesh_msg);

        shape_msgs::Mesh scene_mesh_moveit = boost::get<shape_msgs::Mesh>(scene_mesh_msg);
        collision_scene.meshes[0].triangles = scene_mesh_moveit.triangles;
        collision_scene.meshes[0].vertices = scene_mesh_moveit.vertices;

        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(collision_scene);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher_.publish(planning_scene);

        ROS_INFO("Publish scene collision to planning scene");
    }

    protected:
    ros::NodeHandle& n;
    ros::ServiceServer ss_grasp_handle_;
    ros::ServiceServer ss_grasp_info_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    moveit_msgs::Grasp grasp_;
    geometry_msgs::TransformStamped tf_w2d_;  // Drawer pose in the marker frame
    geometry_msgs::TransformStamped tf_w2o_;  // Obstacle pose in the world frame

    T ei_d2h_;
    T ei_h2EE_;
    T ei_EE2L8_;
    T ei_d2L8_;
    T ei_d2L8_post_grasp_;
    T ei_d2L8_pre_grasp_;

    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> moveAC;
    actionlib::SimpleActionClient<franka_gripper::HomingAction> homingAC;
    geometry_msgs::Pose grasp_pose_;
    geometry_msgs::Pose pre_grasp_pose_;
    geometry_msgs::Pose post_grasp_pose_;
    geometry_msgs::Pose obstacle_pose_;
    rviz_visual_tools::RvizVisualToolsPtr rviz_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools_;

    tf2_ros::TransformBroadcaster tf2_br_;
    
    ros::Subscriber sub_trajectory_;

    moveit_msgs::RobotTrajectory stomp_trajectory_;
    moveit_msgs::RobotTrajectory execute_trajectory_;
    moveit_msgs::RobotTrajectory pre_grasp_trajectory_;
    moveit_msgs::RobotTrajectory pre_grasp_2_grasp_trajectory_;
    moveit_msgs::RobotTrajectory direct_grasp_trajectory_;

    ros::Subscriber sub_aruco_marker_;
    ros::Subscriber sub_obstacle_marker_;
    ros::Publisher  planning_scene_diff_publisher_;

    std::string mesh_path_;

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "panda_grasp_handle_server");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    PandaGraspHandleServer panda_grasp_handle_server(nh);
    
    ros::waitForShutdown();
    return 0;
}