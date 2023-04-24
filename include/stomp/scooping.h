#include "include/stomp/execute_stomp_traj.h"

class Scooping : ExecuteStompTraj
{
    public:
    Scooping(ros::NodeHandle n);
    ~Scooping();
    bool CallbackExecuteStompTraj(panda_moveit_control::ExecuteStompTraj::Request &req,
                                panda_moveit_control::ExecuteStompTraj::Response &res);                      
};