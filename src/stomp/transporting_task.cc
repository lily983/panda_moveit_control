#include "include/stomp/transporting.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transporting_task_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    Transporting transporting_task(nh);

    ros::waitForShutdown();
    return 0;
}