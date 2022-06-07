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
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


#include <string>


enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class BTAction
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    MoveBaseClient ac;
    move_base_msgs::MoveBaseGoal move_base_goal;
    // create messages that are used to published feedback/result
    behavior_tree_core::BTFeedback feedback_;  // action feedback (SUCCESS, FAILURE)
    behavior_tree_core::BTResult result_;  // action feedback  (same as feedback for us)


public:
    explicit BTAction(std::string name) : ac("move_base", true),  as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false),
    action_name_(name)
    {
        // Starts the action server
        as_.start();
    }



    ~BTAction(void)
    {}

    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
    {
        if (as_.isPreemptRequested())
        {
            ROS_INFO("Action Halted");

            // set the action state to preempted
            as_.setPreempted();
            ac.cancelGoal();
            ROS_INFO("Canceling All Goals");
        }
    
        ROS_INFO("**Going to Home (Charging Dock");
        
        move_base_goal.target_pose.header.frame_id = "map";
        move_base_goal.target_pose.header.stamp = ros::Time::now();

        //Coordinates for both Simulation and Actual Map added. Later on this will be added 
        //to the yaml file in next versions after testing on actual robot//
        //For Simulation//
        // move_base_goal.target_pose.pose.position.x = 2.42;
        // move_base_goal.target_pose.pose.position.y = 2.47;
        // move_base_goal.target_pose.pose.position.z = 0.0;
        // move_base_goal.target_pose.pose.orientation.w = 1.0;

        //For actual Robot//
        move_base_goal.target_pose.pose.position.x = 0.60;
        move_base_goal.target_pose.pose.position.y = 2.37;
        move_base_goal.target_pose.pose.position.z = 0.0;
        move_base_goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("**Sending New goal");
        ac.sendGoal(move_base_goal);
        ac.waitForResult();
        set_status(SUCCESS);
    }


    // returns the status to the client (Behavior Tree)
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
            ROS_INFO("Action %s Succeeded", ros::this_node::getName().c_str() );
            break;
        case FAILURE:
            ROS_INFO("Action %s Failed", ros::this_node::getName().c_str() );
            break;
        default:
            break;
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Navi_H");
    ROS_INFO(" Enum: %d", RUNNING);
    ROS_INFO(" Action Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());
    ros::spin();

    return 0;
}
