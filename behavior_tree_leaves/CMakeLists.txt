cmake_minimum_required(VERSION 2.8.3)
project(behavior_tree_leaves)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
  actionlib
  message_generation
  roscpp
  roslib
  rospy
  std_msgs
  behavior_tree_core
  roslaunch
  #simple_navigation_goals
  move_base_msgs

)
roslaunch_add_file_check(launch)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system thread program_options)



# Settings
add_definitions(-Wall -g -O0 -Wno-deprecated -static -Bstatic -std=gnu++0x)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
#add_action_files(
#  DIRECTORY action
#  FILES BT.action
#)




# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here


#generate_messages(
#  DEPENDENCIES actionlib_msgs std_msgs behavior_tree_core# Or other packages containing msgs
#)


# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_actionlib
#  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
#)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(learning_actionlib
#   src/${PROJECT_NAME}/learning_actionlib.cpp
# )

set(COMMON_LIB ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(action_client src/action_client.cpp)
target_link_libraries(action_client ${catkin_LIBRARIES})
add_dependencies(action_client
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)


add_executable(condition_client src/condition_client.cpp)
target_link_libraries(condition_client  ${catkin_LIBRARIES})
add_dependencies(
 condition_client
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)


#-------------------Custom Nodes Added-----------------------#
add_executable(Navi_H example_nodes/cpp/demotree1/Navi_H.cpp)

target_link_libraries(
  Navi_H
  ${catkin_LIBRARIES}
)
add_dependencies(
  Navi_H
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(navitoA example_nodes/cpp/demotree1/navitoA.cpp)
target_link_libraries(
  navitoA
  ${catkin_LIBRARIES}
)

add_executable(stop example_nodes/cpp/demotree1/stop.cpp)
target_link_libraries(
  stop
  ${catkin_LIBRARIES}
)

#Added for New Timed Forward Reverse-------------------
add_executable(forward example_nodes/cpp/demotree1/forward.cpp)
target_link_libraries(
  forward
  ${catkin_LIBRARIES}
)

add_executable(reverse example_nodes/cpp/demotree1/reverse.cpp)
target_link_libraries(
  reverse
  ${catkin_LIBRARIES}
)

add_executable(lessfive example_nodes/cpp/demotree1/lessfive.cpp)
target_link_libraries(
  lessfive
  ${catkin_LIBRARIES}
)

add_executable(lessfive2 example_nodes/cpp/demotree1/lessfive2.cpp)
target_link_libraries(
  lessfive2
  ${catkin_LIBRARIES}
)

add_executable(noforward_key example_nodes/cpp/demotree1/noforward_key.cpp)
target_link_libraries(
  noforward_key
  ${catkin_LIBRARIES}
)

add_executable(noreverse_key example_nodes/cpp/demotree1/noreverse_key.cpp)
target_link_libraries(
  noreverse_key
  ${catkin_LIBRARIES}
)

add_executable(nokey_press example_nodes/cpp/demotree1/nokey_press.cpp)
target_link_libraries(
  nokey_press
  ${catkin_LIBRARIES}
)

add_executable(get_key example_nodes/cpp/demotree1/get_key.cpp)
target_link_libraries(
  get_key
  ${catkin_LIBRARIES}
)

add_executable(stop_key example_nodes/cpp/demotree1/stop_key.cpp)
target_link_libraries(
  stop_key
  ${catkin_LIBRARIES}
)

#----------------FOR DEMO-2-----------------------#
add_executable(kitCmd example_nodes/cpp/demotree2/kitCmd.cpp)
target_link_libraries(kitCmd  ${catkin_LIBRARIES})
add_dependencies(
  kitCmd
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(batteryOK example_nodes/cpp/demotree2/batteryOK.cpp)
target_link_libraries(batteryOK  ${catkin_LIBRARIES})
add_dependencies(
  batteryOK
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoToKit example_nodes/cpp/demotree2/GoToKit.cpp)
target_link_libraries(GoToKit  ${catkin_LIBRARIES})
add_dependencies(
  GoToKit
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(navi_to_H example_nodes/cpp/demotree2/navi_to_H.cpp)
target_link_libraries(navi_to_H  ${catkin_LIBRARIES})
add_dependencies(
  navi_to_H
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(send_goal example_nodes/cpp/demotree2/send_goal.cpp)
target_link_libraries(send_goal  ${catkin_LIBRARIES})
add_dependencies(
 send_goal
 ${behavior_tree_leaves_EXPORTED_TARGETS}
 ${PROJECT_NAME}_generate_messages_cpp
 behavior_tree_core_generate_messages_cpp
)


add_executable(loungeCmd example_nodes/cpp/demotree2/loungeCmd.cpp)
target_link_libraries(loungeCmd  ${catkin_LIBRARIES})
add_dependencies(
  loungeCmd
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(entranceCmd example_nodes/cpp/demotree2/entranceCmd.cpp)
target_link_libraries(entranceCmd  ${catkin_LIBRARIES})
add_dependencies(
  entranceCmd
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)


add_executable(GoTolounge example_nodes/cpp/demotree2/GoTolounge.cpp)
target_link_libraries(GoTolounge  ${catkin_LIBRARIES})
add_dependencies(
  GoTolounge
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoToEntrance example_nodes/cpp/demotree2/GoToEntrance.cpp)
target_link_libraries(GoToEntrance  ${catkin_LIBRARIES})
add_dependencies(
  GoToEntrance
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoToLobby example_nodes/cpp/demotree2/GoToLobby.cpp)
target_link_libraries(GoToLobby  ${catkin_LIBRARIES})
add_dependencies(
  GoToLobby
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoToTvroom example_nodes/cpp/demotree2/GoToTvroom.cpp)
target_link_libraries(GoToTvroom  ${catkin_LIBRARIES})
add_dependencies(
  GoToTvroom
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoTobedRoom example_nodes/cpp/demotree2/GoTobedRoom.cpp)
target_link_libraries(GoTobedRoom  ${catkin_LIBRARIES})
add_dependencies(
  GoTobedRoom
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoTobathRoom example_nodes/cpp/demotree2/GoTobathRoom.cpp)
target_link_libraries(GoTobathRoom  ${catkin_LIBRARIES})
add_dependencies(
  GoTobathRoom
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(lobbycmd example_nodes/cpp/demotree2/lobbycmd.cpp)
target_link_libraries(lobbycmd  ${catkin_LIBRARIES})
add_dependencies(
  lobbycmd
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(tvRoomcmd example_nodes/cpp/demotree2/tvRoomcmd.cpp)
target_link_libraries(tvRoomcmd  ${catkin_LIBRARIES})
add_dependencies(
  tvRoomcmd
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(bedRoomcmd example_nodes/cpp/demotree2/bedRoomcmd.cpp)
target_link_libraries(bedRoomcmd  ${catkin_LIBRARIES})
add_dependencies(
  bedRoomcmd
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(navCmd example_nodes/cpp/demotree2/navCmd.cpp)
target_link_libraries(navCmd  ${catkin_LIBRARIES})
add_dependencies(
  navCmd
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(fallHzCmd example_nodes/cpp/demotree2/fallHzCmd.cpp)
target_link_libraries(fallHzCmd  ${catkin_LIBRARIES})
add_dependencies(
  fallHzCmd
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(heyRuyi_cmd example_nodes/cpp/demotree2/heyRuyi_cmd.cpp)
target_link_libraries(heyRuyi_cmd  ${catkin_LIBRARIES})
add_dependencies(
  heyRuyi_cmd
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(search_person example_nodes/cpp/demotree2/search_person.cpp)
target_link_libraries(search_person  ${catkin_LIBRARIES})
add_dependencies(
  search_person
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(search_person_wn example_nodes/cpp/demotree2/search_person_wn.cpp)
target_link_libraries(search_person_wn  ${catkin_LIBRARIES})
add_dependencies(
  search_person_wn
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

###--------------------------------------------------

###---------For Fall Hazard Scan----------------------

add_executable(GoToKit_f example_nodes/cpp/demotree2/fallhazardnodes/GoToKit_f.cpp)
target_link_libraries(GoToKit_f  ${catkin_LIBRARIES})
add_dependencies(
  GoToKit_f
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoTolounge_f example_nodes/cpp/demotree2/fallhazardnodes/GoTolounge_f.cpp)
target_link_libraries(GoTolounge_f  ${catkin_LIBRARIES})
add_dependencies(
  GoTolounge_f
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoToEntrance_f example_nodes/cpp/demotree2/fallhazardnodes/GoToEntrance_f.cpp)
target_link_libraries(GoToEntrance_f  ${catkin_LIBRARIES})
add_dependencies(
  GoToEntrance_f
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoToLobby_f example_nodes/cpp/demotree2/fallhazardnodes/GoToLobby_f.cpp)
target_link_libraries(GoToLobby_f  ${catkin_LIBRARIES})
add_dependencies(
  GoToLobby_f
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoToTvroom_f example_nodes/cpp/demotree2/fallhazardnodes/GoToTvroom_f.cpp)
target_link_libraries(GoToTvroom_f  ${catkin_LIBRARIES})
add_dependencies(
  GoToTvroom_f
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(GoTobedRoom_f example_nodes/cpp/demotree2/fallhazardnodes/GoTobedRoom_f.cpp)
target_link_libraries(GoTobedRoom_f  ${catkin_LIBRARIES})
add_dependencies(
  GoTobedRoom_f
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)
  ###--------------------------------------------------

