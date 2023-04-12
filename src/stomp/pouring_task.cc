#include "include/stomp/pouring.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pouring_task_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    Pouring pouring_task(nh);

    ros::waitForShutdown();
    return 0;
}