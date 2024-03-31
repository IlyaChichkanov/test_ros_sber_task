#include "client.h"


ServerRespone ClientNode::run_ctrl_task(const control_srv::robot_state goal_state, int robot_id)
{
  ros::ServiceClient client = nh.serviceClient<control_srv::start_control>(srv_name.c_str());
  srv.request.target_position = goal_state;
  srv.request.robot_id = robot_id;
  if(client.call(srv))
  { 
      return srv.response.done? ServerRespone::OK : ServerRespone::BUSY;
  }
  else
  {
      return ServerRespone::FAIL;
  }
}

// work in block mode
void ClientNode::run_ctrl_action(const control_srv::robot_state goal_state, int robot_id, float timeout)
{ 
    // Action Client Declaration (Action Name: ros_tutorial_action)
  actionlib::SimpleActionClient<control_srv::control_taskAction> ac(srv_name.c_str(), true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //wait for the action server to start, will wait for infinite time
  ROS_INFO("Action server started, sending goal.");
  control_srv::control_taskGoal goal; // Declare Action Goal
  goal.robot_id = robot_id;  
  goal.target_pos  = goal_state;    
  ac.sendGoal(goal);  // Transmit Action Goal

  bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
};

int main (int argc, char **argv)          // Node Main Function
{
  ros::init(argc, argv, "control_client"); // Node Name Initialization
  ClientNode client = ClientNode("ctrl_server"); 
  while(1)
  {
    std::cout<<"Enter robot id to control (1 or 2):  ";
    int id;
    std::cin>>id;
    if(id < 0)
    {
      break;
    }
    if(id == 1 || id == 2)
    {
      std::cout<<"Enter robot goal position (x,y,z):  ";
      int x, y, z;
      std::cin>> x;
      std::cin>> y;
      std::cin>> z;
      control_srv::robot_state goal_state;
      goal_state.pos.x = x;
      goal_state.pos.y = y;   
      goal_state.pos.z = z;  

      ServerRespone res = client.run_ctrl_task(goal_state, id);
      print_server_task_res(res);

      ////use for blocking mode
      // float timeout = 15.0  
      // client.run_ctrl_action(goal_state, id, timeout);
    }
    else
    {
      std::cout<<"Wrong robor id\n";
    }
    
  }



  // std::chrono::milliseconds dur(4000);
  // std::this_thread::sleep_for(dur);

  return 0;
}