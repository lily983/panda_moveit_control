#include "include/stomp/execute_stomp_traj.h"

class Transporting: ExecuteStompTraj
{
    public:
    Transporting(ros::NodeHandle n);
    ~Transporting();
    bool CallbackExecuteStompTraj(panda_moveit_control::ExecuteStompTraj::Request &req,
                                panda_moveit_control::ExecuteStompTraj::Response &res);
};