#ifndef __ServerNode__

#include <thread>
#include <condition_variable>
#include <actionlib/server/simple_action_server.h>// action Library Header File
#include "ros/ros.h"
#include "ros/package.h"
#include <string>
#include "geometry_msgs/Vector3.h"

#include "control_srv/robot_state.h"
#include "control_srv/get_control_result.h"
#include "control_srv/start_control.h"
#include "control_srv/control_taskAction.h" 
#include "controllers.h"


typedef actionlib::SimpleActionServer<control_srv::control_taskAction> ActServer;

class ServerNode 
{    
public:
    ServerNode(std::string name);
    int run();
protected:
    ros::NodeHandle nh;
    ActServer as1_; 
    std::string srv_name_;
    control_srv::control_taskFeedback feedback_;
    control_srv::control_taskResult result_; 
    void control_acrionCB(const control_srv::control_taskGoalConstPtr &goal);  
    RobotDriver controller1;
    RobotDriver controller2;
    int current_robot_id;
private:
    ros::Subscriber StateSub;
    void StateCb(const control_srv::robot_state& msg); 
    control_srv::robot_state curr_position;
private:
    bool control_srvCB(control_srv::start_control::Request &req,
                       control_srv::start_control::Response &res);
    bool start_control;
    std::thread controlthread;
    std::condition_variable data_cond;
    std::mutex m;
    control_srv::start_control::Request  ctrl_request;
    control_srv::start_control::Response ctrl_result;
    void control_tread_func();
};


#endif
