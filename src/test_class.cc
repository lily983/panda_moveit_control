#include <ros/ros.h>
#include "include/planning_scene/planning_scene_operation.h"
#include "include/robot/panda_arm_planning_control.h"
#include "include/robot/panda_gripper_control.h"
#include <moveit/robot_state/conversions.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_class");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
 
    ROS_INFO("Start test_class node");

    std::string name, path;
    nh.getParam("/test_class/collision_obj_path", path);
    nh.getParam("/test_class/obj_name", name);

    PlanningSceneOperation planning_scene_opreation(nh);

    PandaArmPlanningControl panda_robot;

    geometry_msgs::Pose pose_target;
    pose_target.position.x = 0.5;
    pose_target.position.z = 0.5;
    pose_target.orientation.w = 0;
    pose_target.orientation.y = 1;
    
    moveit_msgs::DisplayTrajectory dis_traj;
    moveit_msgs::RobotState state;
    robotStateToRobotStateMsg(panda_robot.GetMoveGroupState(), state);
    dis_traj.trajectory_start = state;
    dis_traj.trajectory.push_back(panda_robot.PlanningToPoseTarget(pose_target));
    
    planning_scene_opreation.VisualizeTrajectory(dis_traj);

    ros::waitForShutdown();
    return 0;
}
