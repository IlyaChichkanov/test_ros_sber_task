
#include <stdio.h>
#include <stdlib.h>

#include "server.h"
#include <thread>
#include <chrono>

ServerNode::ServerNode(std::string name):
    as1_(nh, name, boost::bind(&ServerNode::control_acrionCB, this, _1), false)
    ,srv_name_(name)
{
    as1_.start();
}


int ServerNode::run()
{
    try {
        auto control_service = nh.advertiseService(srv_name_.c_str(), &ServerNode::control_srvCB, this);
        StateSub = nh.subscribe("robot_state", 1, &ServerNode::StateCb, this);
        controlthread = std::thread{&ServerNode::control_tread_func, this};
        controller1.set_topic_name("driver_location1");
        controller1.set_controller(std::make_shared<RobotControllerKuka>());
        controller2.set_topic_name("driver_location2");
        controller2.set_controller(std::make_shared<RobotControllerFuka>());
        start_control = false;
        ROS_INFO("ServerNode:: init  ok");
        ros::spin();
    }
    catch(const std::exception& e)
    {
        ROS_FATAL(e.what());
        return EXIT_FAILURE;
    }

    ROS_INFO("Done");
    return EXIT_SUCCESS;
}

void ServerNode::control_tread_func()
{
    while(1)
    {
        std::unique_lock<std::mutex> lock(m);
        data_cond.wait(lock, [&] {
          return start_control;
        });
    
        ros::Rate r(1); 
        current_robot_id = ctrl_request.robot_id;
        RobotDriver* controller = (current_robot_id == 1)? &controller1: &controller2;
        controller->set_goal_state(ctrl_request.target_position);
        control_srv::robot_state goal_state = controller->get_goal_state();
        std::cout<<"-------------START CONTROL ROBOT "<<current_robot_id<<"  "<<controller->get_name()<<" to "<<goal_state.pos.x<<" "<<goal_state.pos.y<<" "<<goal_state.pos.z<<" -----------------"<<std::endl;
        while(1)
        {
            bool finished = controller->go_to_state(curr_position);
            if(finished)
            {
               std::cout<<"------------FINISH CONTROL "<<controller->get_name()<<" ------------"<<std::endl;
               break;
            }
            r.sleep();
        }
        start_control = false;
    }
}


void ServerNode::StateCb(const control_srv::robot_state& msg)
{
    curr_position = msg;
}

bool ServerNode::control_srvCB(control_srv::start_control::Request &req,
                               control_srv::start_control::Response &res)
{
    std::cout<<"control server got new task\n";
    if(!start_control) // RObots ready to mode
    {
        res.done = true;
        std::unique_lock<std::mutex> lock(m);
        ctrl_request = req;
        start_control = true;
        data_cond.notify_one();
    }
    else
    {
        bool done = (current_robot_id == req.robot_id);
        if(done)
        {
            RobotDriver* controller = (req.robot_id == 1)? &controller1: &controller2;
            controller->set_goal_state(req.target_position);
        }
        else
        {
            std::cout<<"LINE BUSY: robot "<<current_robot_id<<"  is working\n";
        }
        res.done = done;

    }
    return true;
}


  void ServerNode::control_acrionCB(const control_srv::control_taskGoalConstPtr &goal)
  {
    std::cout<<"control server got new ROS action task\n";
    if(start_control)
    {
        result_.finish = false;
        as1_.setSucceeded(result_);
        return;
    }
    ros::Rate r(1);       // Loop Rate: 1Hz
    bool success = true;  // Used as a variable to store the success or failure of an action
    RobotDriver* controller = (goal->robot_id == 1)? &controller1: &controller2;
    controller->set_goal_state(goal->target_pos);
    control_srv::robot_state goal_state = controller->get_goal_state();
    std::cout<<"-------------START CONTROL ROBOT "<<goal->robot_id<<"  "<<controller->get_name()<<" to "<<goal_state.pos.x<<" "<<goal_state.pos.y<<" "<<goal_state.pos.z<<" -----------------"<<std::endl;
    int cnt = 0;
    while(1)
    {
        if (as1_.isPreemptRequested() || !ros::ok())
        {
            // Notify action cancellation
            ROS_INFO("%s: Preempted", srv_name_.c_str());
            // Action cancellation and consider action as failure and save to variable
            as1_.setPreempted();
            success = false;
            break;
        }
        bool finished = controller->go_to_state(curr_position);
        if(finished)
        {
            break;
        }
        feedback_.cnt = cnt;
        as1_.publishFeedback(feedback_);
        cnt++;
        r.sleep();
    }
    result_.finish = success;
    as1_.setSucceeded(result_);
    std::cout<<"------------FINISH CONTROL "<<controller->get_name()<<" ------------"<<std::endl;
  }


int main(int argc,char** argv)
{
    ros::init(argc, argv, "wr_surf_control");
    ServerNode  srv("ctrl_server");
    srv.run();
    return 0;
}
