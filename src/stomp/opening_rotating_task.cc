#include "include/stomp/opening_rotating.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opening_rotating_task_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    OpeningRotating opening_rotating_task(nh);

    ros::waitForShutdown();
    return 0;
}