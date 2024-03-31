#pragma once

#include <string>
#include "control_srv/robot_state.h"

struct Control_msg
{
    float u;
    float v;
};


class RobotController
{
public:
    RobotController(){};
    virtual Control_msg calculate_control(const control_srv::robot_state& curr_state, 
                                          const control_srv::robot_state& goal_state) = 0;
    virtual std::string get_name() const = 0;
private:
    // ----------fill---------------

};


class RobotControllerKuka:  public RobotController
{
public:
    RobotControllerKuka()
    :name("Kuka")
    {};
    Control_msg calculate_control(const control_srv::robot_state& curr_state, 
                                  const control_srv::robot_state& goal_state) override 
    {
        std::cout<<"calculate_control for RobotControllerKuka\n";
        Control_msg res;
        res.u = 1.1;
        // ----------fill---------------
        return res;
    };
    std::string get_name() const 
    {
        return name;
    }
private:
    std::string name;
};

class RobotControllerFuka:  public RobotController
{
public:
    RobotControllerFuka()
    :name("Fuka"){}
    Control_msg calculate_control(const control_srv::robot_state& curr_state, 
                                  const control_srv::robot_state& goal_state) override
    {
        std::cout<<"calculate_control for RobotControllerFuka\n";
        Control_msg res;
        res.u = 2.1;
        // ----------fill---------------
        return res;
    };
    std::string get_name() const 
    {
        return name;
    }
private:
    std::string name;
};


class RobotDriver
{
public:
    RobotDriver(){ };
    bool go_to_state(const control_srv::robot_state& curr_state)
    {
        controller->calculate_control(curr_state, goal_state);
        std::cout<<"send control to: "<<topic_name<<"\n";
        cnt++;
        return cnt > 10;
    }

    void set_topic_name(std::string driver_topic)
    {
        topic_name = driver_topic;
    };

    void set_goal_state(const control_srv::robot_state& goal)
    {
        std::cout<<"set new state for "<<get_name()<<" to "<<goal.pos.x<<" "<<goal.pos.y<<" "<<goal.pos.z<<std::endl;
        goal_state = goal;
        cnt = 0;
    }
    control_srv::robot_state get_goal_state() const
    {
        return goal_state;
    }
    void set_controller(std::shared_ptr<RobotController> c)
    {
        controller = c;
    }
    std::string get_name() const
    {
        if(controller)
        {
            return controller->get_name();
        }
        return "";
    }
private:
    control_srv::robot_state goal_state;
    std::string topic_name;
    std::shared_ptr<RobotController> controller;
    int cnt;
};

