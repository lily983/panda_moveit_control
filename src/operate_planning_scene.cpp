#include "include/planning_scene/planning_scene_operation.h"
#include "include/utils/data_parser.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "operate_planning_scene");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    std::string web_mesh_scene;
    std::string web_mesh_object;
    std::string repo_mesh_scene;
    std::string repo_mesh_object;

    ros::param::get("web_mesh_scene", web_mesh_scene);
    ros::param::get("web_mesh_object", web_mesh_object);
    ros::param::get("repo_mesh_scene", repo_mesh_scene);
    ros::param::get("repo_mesh_object", repo_mesh_object);

    std::cout << web_mesh_scene << "\n";
    std::cout << web_mesh_object << "\n";
    std::cout << repo_mesh_scene << "\n";
    std::cout << repo_mesh_object << "\n";

    ROS_INFO("web mesh scene path is %s", web_mesh_scene.c_str());
    ROS_INFO("web mesh object path is %s", web_mesh_object.c_str());
    ROS_INFO("repo mesh scene path is %s", repo_mesh_scene.c_str());
    ROS_INFO("repo mesh object path is %s", repo_mesh_object.c_str());

    CopyMeshFromWebToRepo(web_mesh_scene, repo_mesh_scene);
    CopyMeshFromWebToRepo(web_mesh_object, repo_mesh_object);

    PlanningSceneOperation planning_scene_operation(nh);

    ros::waitForShutdown();
    return 0;
}