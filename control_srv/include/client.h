#include <ros/ros.h>    
#include <thread>
#include <chrono>
                          // ROS Default Header File
#include <actionlib/client/simple_action_client.h>// action Library Header File
#include <actionlib/client/terminal_state.h>      // Action Goal Status Header File
#include "control_srv/control_taskAction.h"

#include "control_srv/robot_state.h"
#include "control_srv/get_control_result.h"
#include "control_srv/start_control.h"

enum ServerRespone
{
   OK = 1,
   BUSY = 2,
   FAIL = 3
};

void print_server_task_res(ServerRespone res)
{
  switch (res)
  {
    case ServerRespone::OK:
      std::cout<<"Server call OK\n";
      break;
    case ServerRespone::BUSY:
      std::cout<<"Server BUSY\n";
      break;
    case ServerRespone::FAIL:
      std::cout<<"Server FAIL\n";
      break;
    default:
      break;
  }
}

class ClientNode 
{    
public:
    ClientNode(std::string name)
    :srv_name(name){};

    ServerRespone run_ctrl_task(const control_srv::robot_state goal_state, int robot_id);
    
    void run_ctrl_action(const control_srv::robot_state goal_state, int robot_id, float timeout);
private:
  std::string srv_name;
  control_srv::start_control srv;
  ros::NodeHandle nh;
};
