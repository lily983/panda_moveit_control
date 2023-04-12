#include "include/stomp/pouring.h"

Pouring::Pouring(ros::NodeHandle n)
: ExecuteStompTraj(n)
{
    ROS_INFO("===[Pouring]: finished initialization!===");
}

Pouring::~Pouring(){};

bool Pouring::CallbackExecuteStompTraj(panda_moveit_control::ExecuteStompTraj::Request &req,
                                panda_moveit_control::ExecuteStompTraj::Response &res)
{
    ROS_INFO("===Received request to execute STOMP trajectory for the pouring task!===");
    stomp_trajectory_ = all_stomp_trajectory_.trajectory.at(req.num);
    trajectory_msgs::JointTrajectoryPoint stomp_first_point;
    stomp_first_point = stomp_trajectory_.joint_trajectory.points.at(0);

    ROS_INFO("===Move to the first joint configuration!!!===");
    ros::Duration(1.0).sleep();
    if(panda_arm_.MoveToJointTarget(stomp_first_point.positions)==false){
        ROS_ERROR("===Failed to get to the first joint configuration!!!===");
        res.success = false;
        return false;
    }

    ROS_INFO("===Reached to the start joint configuration. Please handover the cup!===");
    ros::Duration(5.0).sleep();
    if(GripperMoveAction(0.055, 0.01)==false){
        ROS_WARN("===Franka gripper failed to execute move command!===");
    }
    ROS_INFO("===Ready to execute STOMP trajectory! Please leave the arm!===");
    ros::Duration(5.0).sleep();

    if(panda_arm_.ExecuteTrajectory(stomp_trajectory_)==false){
        ROS_ERROR("===Failed to execute STOMP trajectory!===");
        res.success = false;
        return false;
    }

    ROS_INFO("===Finished executing STOMP trajectory for the pouring task!===");
    res.success = true;
    return true;
}



