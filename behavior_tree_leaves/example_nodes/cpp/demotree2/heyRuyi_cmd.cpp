/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


#include <string>
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include<sstream>
#include<vector>


//using namespace std;

enum Status {RUNNING, SUCCESS, FAILURE};
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class BTAction
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange errors may occur.
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    behavior_tree_core::BTFeedback feedback_;
    behavior_tree_core::BTResult result_;
    bool person_point;
    bool hey_msg;
    int count = 0;
    geometry_msgs::Twist msg;
    std_msgs::Int16 msg2;
    std_msgs::String hello_msg;

public:

    explicit BTAction(std::string name) :
    as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false),
    action_name_(name)
    {
        // start the action server (action in sense of Actionlib not BT action)
        as_.start();
        ROS_INFO("Condition Server Started");
        person_point = false;
        hey_msg = false;

    }

    //ros::Subscriber sub = nh_.subscribe("hello_scenario", 1000, &BTAction::conditionSetCallback, this); //Used with old application. Not used any more
    ros::Subscriber sub = nh_.subscribe("cmd_frm_tablet", 1000, &BTAction::conditionSetCallback, this);

    ~BTAction(void)
    { }

    void conditionSetCallback(const std_msgs::String::ConstPtr& msg)
    {    
        std::string my_str = msg->data;
        std::vector<std::string> result;
        std::stringstream s_stream(my_str); //create string stream from the string

        while(s_stream.good()) 
        {
            std::string substr;
            getline(s_stream, substr, ','); //get first string delimited by comma
            result.push_back(substr);
        }

        //cout<<"Location ID: "<<result.at(4) << endl;
        //msg2.data = atoi(result.at(4).c_str());

        std::string hello_str = result.at(1).c_str();
        //std::string my_str = msg->data;

        if(hello_str == "hello")
        {
            hey_msg = true;
            ROS_INFO("Hey Ruyi string True");
            hello_str = "";
        }
        else 
        {
            hey_msg = false;
            ROS_INFO("Hey Ruyi string False");
            hello_str = "";
        }

    }

    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
    {
        ROS_INFO("Executing Call back");
        if (hey_msg == true) 
        {
            std::cout<<"Hey msg is TRUE" << std::endl;
            set_status(FAILURE);

            if (as_.isPreemptRequested())
            {
                ROS_INFO("Action Halted");

                // set the action state to preempted
                as_.setPreempted();
                //ac.cancelGoal();
                //ROS_INFO("Canceling All Goals");
                set_status(SUCCESS);
                hey_msg = false;
            }
        
            ROS_INFO("**Rotating the Robot to find the person");
            hey_msg = false;
        
        }
        else
        {
            std::cout<<"Hey msg is FALSE" << std::endl;
            set_status(SUCCESS);
        }
    }

    //  returns the status to the client (Behavior Tree)
    void set_status(int status)
    {
        // Set The feedback and result of BT.action
        feedback_.status = status;
        result_.status = feedback_.status;
        // publish the feedback
        as_.publishFeedback(feedback_);
        // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
        as_.setSucceeded(result_);

        switch (status)  // Print for convenience
        {
        case SUCCESS:
            ROS_INFO("Condition %s Succeeded", ros::this_node::getName().c_str() );
            break;
        case FAILURE:
            ROS_INFO("Condition %s Failed", ros::this_node::getName().c_str() );
            break;
        default:
            break;
        }
    }
};

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "heyRuyi_cmd");
    ROS_INFO(" Enum: %d", RUNNING);
    ROS_INFO(" condition Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());

    ros::spin();
    return 0;
}
