#include "include/planning_scene/planning_scene_operation.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "operate_planning_scene");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    PlanningSceneOperation planning_scene_operation(nh);

    ros::waitForShutdown();
    return 0;
}