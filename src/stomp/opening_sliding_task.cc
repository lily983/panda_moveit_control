#include "include/stomp/opening_sliding.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opening_sliding_task_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    OpeningSliding opening_sliding_task(nh);

    ros::waitForShutdown();
    return 0;
}