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
#include<rehab_person_loc/location_info.h>
// using namespace std;

enum Status {RUNNING, SUCCESS, FAILURE};
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class BTAction
{
protected:
    ros::NodeHandle nh_;
    // ros::Rate r(10);
    // NodeHandle instance must be created before this line. Otherwise strange errors may occur.
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    MoveBaseClient ac;
    move_base_msgs::MoveBaseGoal move_base_goal;
    // create messages that are used to published feedback/result
    behavior_tree_core::BTFeedback feedback_;
    behavior_tree_core::BTResult result_;
    bool person_point;
    bool hey_msg;
    int count = 0;
    geometry_msgs::Twist msg;
    std_msgs::String feedback_msg;
    bool publish_once = true;
    bool person_distance_threshold_high = false; // Initial false 
    geometry_msgs::PointStamped person_loc_in_MAP;
    bool person_inside_map = false;
    int person_inside_map_counter = 0;
    bool goal_assigned = false;

 public:

    //explicit BTAction(std::string name) : as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false),
    //action_name_(name)
    explicit BTAction(std::string name) : ac("move_base", true),  as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false),
    action_name_(name)
    {
        // start the action server (action in sense of Actionlib not BT action)
        as_.start();
        ROS_INFO("Condition Server Started");
        person_point = false;
        hey_msg = false;
        

    }

    ros::Subscriber point_sub = nh_.subscribe("/person_loc",1000, &BTAction::cameraCallBack,this);
    
    ros::Subscriber person_point_sub = nh_.subscribe("/person_loc_estimated",1000, &BTAction::personPOSMAPCallBack,this);

    ros::Subscriber loc_tag = nh_.subscribe("/location_tag",1000, &BTAction::locationTagMAPCallBack,this);



    ros::Publisher pub_vel = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000); //For Actual Hardware
    //ros::Publisher pub_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); //For simulation only

    ros::Publisher pub_feedback = nh_.advertise<std_msgs::String>("/feedback_to_tablet", 1000); //Publsiher for Feedback to Tablet



    ~BTAction(void)
    { }


    void cameraCallBack(const geometry_msgs::PointStamped::ConstPtr &ptr)
    {
        //Unit Conversion: 6ft = 1828mm//


         if (ptr->point.x != -9999 || ptr->point.y != -9999 || ptr->point.z != -9999)
        //if (ptr->point.x > -920 && ptr->point.x < 920)
        {
           person_point = true; 
           //ROS_INFO("Got Person Point");
           
           if(ptr->point.z  > 1828 )
           {
             person_distance_threshold_high = true;
           }
           else
           {
            person_distance_threshold_high = false;

            if(goal_assigned == true)
            {
                ac.cancelGoal();
                ROS_INFO("Canceling All Goals");
                goal_assigned = false;
            }
             
           }

        }
        else 
        {
           person_point = false; 
           //ROS_INFO("Person Point False");
        }
    }


    //Subscribing to the Person Location in MAP 
    void locationTagMAPCallBack(const rehab_person_loc::location_info::ConstPtr &ptr)
    {
        if(ptr->person_location != "Away")
        {
            //std::cout<<"person detected!!!!!!!!!"<<std::endl;
            // person_inside_map = true;
            person_inside_map_counter++;
        }

        else
        {
            //person_inside_map = false;
            person_inside_map_counter = 0;
        } 

        if(person_inside_map_counter >= 3)
            person_inside_map = true;

        else 
            person_inside_map = false;


    }

    void personPOSMAPCallBack(const geometry_msgs::PointStamped::ConstPtr &ptr)
    {
        if(person_inside_map == true)
        //if (ptr->point.x != -9999 || ptr->point.y != -9999 || ptr->point.z != -9999)
        {
            person_loc_in_MAP.point.x = ptr->point.x;
            person_loc_in_MAP.point.y = ptr->point.y;
            person_loc_in_MAP.point.z = ptr->point.z;

        }
    }

    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
    {
        ROS_INFO("Executing Call back");
        //set_status(FAILURE);

        if (as_.isPreemptRequested())
        {
            ROS_INFO("Action Halted");

            // set the action state to preempted
            as_.setPreempted();
            ac.cancelGoal();
            //ROS_INFO("Canceling All Goals");
            msg.angular.z = 0.0;
            pub_vel.publish(msg);
            msg.angular.z = 0.0;
            pub_vel.publish(msg);
            msg.angular.z = 0.0;
            pub_vel.publish(msg);
            msg.angular.z = 0.0;
            pub_vel.publish(msg);
            person_point = true;
            ROS_INFO("Prempted Task and Stoping Robot!!!");
            //set_status(SUCCESS);
        }
    
        ROS_INFO("**Rotating the Robot to find the person");
    
        if (person_inside_map == false) 
        {
            if(publish_once == true)
            {

                feedback_msg.data = "Finding Person";
                pub_feedback.publish(feedback_msg);
                publish_once = false;
            }
            
            while(person_inside_map == false)
            {
                msg.linear.x = 0.0;
                msg.angular.z = 0.1;   //0.3 for simulations if the robot moves too slow
                pub_vel.publish(msg);
                ROS_INFO("Rotating & Finding person");
            }
            
            feedback_msg.data = "Person Found";
            pub_feedback.publish(feedback_msg);
            ROS_INFO("Person Found");

            ros::Duration(2.0).sleep();
            if (person_inside_map == true )
            {
                // Give goal of the Person to the robot
                move_base_goal.target_pose.header.frame_id = "map";
                move_base_goal.target_pose.header.stamp = ros::Time::now();

                move_base_goal.target_pose.pose.position.x = person_loc_in_MAP.point.x;
                move_base_goal.target_pose.pose.position.y = person_loc_in_MAP.point.y;
                move_base_goal.target_pose.pose.orientation.w = 1.0;

                goal_assigned = true;

                std::cout<<"X= "<<move_base_goal.target_pose.pose.position.x<<std::endl;
                std::cout<<"Y= "<<move_base_goal.target_pose.pose.position.y<<std::endl;
                std::cout<<"Z= "<<move_base_goal.target_pose.pose.position.z<<std::endl;
                ROS_INFO("**Sending Person Location goal 1, Person initially not in MAP");
                ac.sendGoal(move_base_goal);

                feedback_msg.data = "Goal Sent 1";
                pub_feedback.publish(feedback_msg);
                ac.waitForResult();

            }
            
            ROS_INFO("Goal Reached 1");
            ac.cancelGoal();
            feedback_msg.data = "Goal Reached 1";
            pub_feedback.publish(feedback_msg);
            set_status(SUCCESS);
            feedback_msg.data = "";
            publish_once = true;
        }

        else if (person_inside_map == true) //if person already in map 
        {
            ros::Duration(2.0).sleep();

            if(person_inside_map == true)
            {
                // Give goal of the Person to the robot
                move_base_goal.target_pose.header.frame_id = "map";
                move_base_goal.target_pose.header.stamp = ros::Time::now();

                move_base_goal.target_pose.pose.position.x = person_loc_in_MAP.point.x;
                move_base_goal.target_pose.pose.position.y = person_loc_in_MAP.point.y;
                move_base_goal.target_pose.pose.orientation.w = 1.0;
                goal_assigned = true;
                ROS_INFO("**Sending Person Location goal 2, Person Already in MAP ");
                ac.sendGoal(move_base_goal);
                ac.waitForResult();
                std::cout<<"X= "<<move_base_goal.target_pose.pose.position.x<<std::endl;
                std::cout<<"Y= "<<move_base_goal.target_pose.pose.position.y<<std::endl;
                std::cout<<"Z= "<<move_base_goal.target_pose.pose.position.z<<std::endl;

                feedback_msg.data = "Goal Sent 2";
                pub_feedback.publish(feedback_msg);
                set_status(SUCCESS);
                feedback_msg.data = "";
                publish_once = true;
            }
            
        }

        else
        {
            set_status(FAILURE);   //set_status(SUCCESS);
        }


        // while (goal_assigned == true)
        // {
        //     ROS_INFO("now waiting if person is inside 6 feet");
        //     if(person_distance_threshold_high == false)
        //     {   
        //         ROS_INFO("**Cancelling Goal, Now person inside 6 feet");
        //         ac.cancelGoal();
        //         set_status(SUCCESS);
        //         goal_assigned = false;
        //     }
        // }
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
    
    ros::init(argc, argv, "search_person");
    ROS_INFO(" Enum: %d", RUNNING);
    ROS_INFO(" search_person condition Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());

    ros::spin();
    return 0;
}




