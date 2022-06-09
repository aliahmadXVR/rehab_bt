![Version](https://img.shields.io/badge/MRR-XAVOR-yellow)<img align="right" src="images/xavor.png" width="40" height="40">  
# Rehab Robot Behavior Tree Based Navigation & Integration with Android Tablet


## Overview
This package contains the behavior tree based Rehab robot control framework. The baseline of this implementation can be found [here](https://github.com/miccol/ROS-Behavior-Tree.git). Reading material can be seen in this [document](/home/ali-ahmed/catkin_ws/src/rehab_bt/BTUserManual.pdf) for basics of Behavior trees. The aim to develop behavior tree based framework is to enable the robot perform different actions and to respond to user commands. The interconnection of mobile tablet on the robot with ROS will enable 


As a first implementation, a basic scenerio is implemented in this repository. A basic block diagram of overall system is shown below:
![overall-system-image](images/Basic_comm_concept.png)



## Pre-Requisites

1. This package is tested with ROS Melodic so preferably use the package with same ROS version.
2. This package depends highly on the navigation stack & all the nessasry bring up nodes requuired to fully power up the robot. The complete instruction to power up and to run the navgiation can be found [here](https://github.com/Xavorcorp/Rehabbot-EETeam.git).


## Setup 

After following the step 2 in the pre-requisites, you must have the following in working:

* Robot must be running navigation. Verify that the navigation stack is running & the robot is able to navigate by accepting goal from *RVIZ*. 



## Usage 
After following the above setup process, run the following nodes: 

1. Run the following launch file which will launch all the tree nodes and relevant rosparameters.  


   
   ```bash 
   rosrun rehab_person_loc publish_text_marker_act
   ``` 
   to display the markers on the living lab map. These markers locations are hardcoded in the code. To modify the location of any marker, edit the respective marker location in ```publish_text_marker_act.cpp``` file. The map in the RVIZ should look similar to the one shown below after adding the relevant marker topics.  


2. Run the following to run the main node.,
   
   ```bash 
   rosrun rehab_person_loc getTime_act
   ```
   

   ### Subsribed Topics: 
   1. ```amcl_pose``` (geometry_msgs/PoseWithCovarianceStamped)
   
         Pose of the robot in the MAP frame.

   2. ```person_loc``` (geometry_msgs/PointStamped)

         Location of person in camera frame.
   
   ### Published Topics
   1. ```person_loc_estimated``` (geometry_msgs/PointStamped) 
   
         Computed location of person in MAP frame

   2. ```location_tag```(rehab_person_loc/location_info)
   
         Location of the robot and person in MAP.

   3. ```time_info```(rehab_person_loc/time_info) 
      
         The duration the person spents @ various locations in the tagged area in MAP. 


   ### Messages
   1.  ```location_tag.msg```

   ```bash
   time stamp
   string frame_id
   string robot_location
   string person_location
   ``` 

   2. ```time_info.msg```

   ```bash
   int32 kitchen_time
   int32 lounge_time
   int32 entrance_time
   int32 lobby_time
   int32 tvRoom_time
   int32 bedRoom_time
   int32 away_time
   ```


## Output
Check the below two published topics for the output shown below: 

* The topic ```location_tag``` will show the person and robot location in the MAP frame. Ouput will be like the one shown below. 
  

* The topic ```time_info``` will show the duration (in seconds) the person spends at various locations inside the MAP. Ouput will be like the one shown below. 


* Add the topic ```person_loc_estimated``` to *RVIZ* for visualizing the location of the person inside the MAP. The output will be like the one shown below (person location shown as purple colored marker on the MAP)
  

* The package also logs the person and robot location data along with the system clock time stamps in a text file. The log file is stored in ```rehab_person_loc/logs``` folder with the name ```rehab_person_loc.txt```. Sample file can be checked [here](rehab_person_loc/../logs/rehab_person_loc.txt) 



___
<br></br>

## Contact
**Author:** Ali Ahmad  <br/>
**Designation:** Senior Robotics Engineer <br/>
**Email:** ali.ahmad@xavor.com <br/>