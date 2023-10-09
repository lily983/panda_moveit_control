#include "include/robot/panda_arm_planning_control.h"
#include "include/robot/panda_gripper_control.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_panda_class");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    PandaArmPlanningControl panda_arm(0.1, 0.05);

    ROS_INFO("==========Send arm to home config===========");
    ros::Duration(2.0).sleep();
    panda_arm.GoHome();

    ROS_INFO("==========Send arm to new config===========");
    ros::Duration(2.0).sleep();
    std::vector<double> new_config = {
        -0.005279347751289606, -0.2439806076267309, -0.015340340548884663,
        -2.286185151819128,    0.11115300154485346, 1.9794540780915153,
        0.8009687196584763};
    panda_arm.MoveToJointTarget(new_config);

    ROS_INFO("==========Test Gripper==========");
    GripperMoveAction(0.02);

    GripperHomingAction();

    ROS_INFO("==========Send arm to home config===========");
    ros::Duration(2.0).sleep();
    panda_arm.GoHome();

    ROS_INFO("==========Send arm to pose target============");
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.939;
    target_pose.orientation.y = 0.063;
    target_pose.orientation.z = 0.3;
    target_pose.orientation.w = -0.156;
    target_pose.position.x = 0.3;
    target_pose.position.y = -0.224;
    target_pose.position.z = 0.501;
    panda_arm.MoveToPoseTarget(target_pose);

    ROS_INFO("==========Send arm to home config===========");
    ros::Duration(2.0).sleep();
    panda_arm.GoHome();

    ROS_INFO("=========Finish testing=======");
    ros::waitForShutdown();
    return 0;
}