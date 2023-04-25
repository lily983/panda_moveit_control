#include "include/stomp/execute_stomp_traj.h"

class Transporting : public ExecuteStompTraj {
  public:
    Transporting(ros::NodeHandle n);
    ~Transporting();

  protected:
    bool CallbackExecuteStompTraj(
        panda_moveit_control::ExecuteStompTraj::Request& req,
        panda_moveit_control::ExecuteStompTraj::Response& res);
};