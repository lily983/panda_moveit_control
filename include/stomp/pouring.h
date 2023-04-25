#include "include/stomp/execute_stomp_traj.h"

class Pouring : ExecuteStompTraj {
  public:
    Pouring(ros::NodeHandle n);
    ~Pouring();
    bool CallbackExecuteStompTraj(
        panda_moveit_control::ExecuteStompTraj::Request& req,
        panda_moveit_control::ExecuteStompTraj::Response& res);
};