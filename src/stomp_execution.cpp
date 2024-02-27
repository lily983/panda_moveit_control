#include "include/stomp/opening.h"
#include "include/stomp/pouring.h"
#include "include/stomp/scooping.h"
#include "include/stomp/transporting.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "stomp_execution_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    std::string motion_primitive;
    nh.getParam("motion_primitive", motion_primitive);
    ROS_INFO("Ready to execute %s task", motion_primitive.c_str());

    if (motion_primitive == "opening") {
        Opening opening(nh);
    } else if (motion_primitive == "pouring") {
        Pouring pouring(nh);
    } else if (motion_primitive == "scooping") {
        Scooping scooping(nh);
    } else if (motion_primitive == "transporting") {
        Transporting transporting(nh);
    }

    ros::waitForShutdown();
    return 0;
}