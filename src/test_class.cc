#include <ros/ros.h>
#include "include/robot/panda_arm_planning_control.h"
#include "include/robot/panda_gripper_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_class");
    ros::NodeHandle nh;
    ros::spin();

    PlanningSceneOperation planning_scene_opreation(nh);
    
}
