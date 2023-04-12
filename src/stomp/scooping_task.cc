#include "include/stomp/scooping.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scooping_task_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    Scooping scooping_task(nh);

    ros::waitForShutdown();
    return 0;
}