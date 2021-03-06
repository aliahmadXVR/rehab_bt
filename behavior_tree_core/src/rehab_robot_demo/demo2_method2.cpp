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


#include <behavior_tree.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo2_method2");
    ros::NodeHandle n;
    try
    {
        int TickPeriod_milliseconds = 50;

        //Condition Nodes//
        BT::ROSCondition* kitCmd = new BT::ROSCondition("kitCmd");
        BT::ROSCondition* batteryOK = new BT::ROSCondition("batteryOK");
        BT::ROSCondition* loungeCmd = new BT::ROSCondition("loungeCmd");
        BT::ROSCondition* entranceCmd = new BT::ROSCondition("entranceCmd");
        BT::ROSCondition* lobbycmd = new BT::ROSCondition("lobbycmd");
        BT::ROSCondition* tvRoomcmd = new BT::ROSCondition("tvRoomcmd");
        BT::ROSCondition* bedRoomcmd = new BT::ROSCondition("bedRoomcmd");

        //Action Nodes//
        BT::ROSAction* navi_to_H = new BT::ROSAction("navi_to_H");
        BT::ROSAction* GoToKit = new BT::ROSAction("GoToKit");
        BT::ROSAction* GoTolounge = new BT::ROSAction("GoTolounge");
        BT::ROSAction* GoToEntrance = new BT::ROSAction("GoToEntrance");
        BT::ROSAction* GoToLobby = new BT::ROSAction("GoToLobby");
        BT::ROSAction* GoToTvroom = new BT::ROSAction("GoToTvroom");
        BT::ROSAction* GoTobedRoom = new BT::ROSAction("GoTobedRoom");


        //Sequences//
        BT:: SequenceNode* sequence0 = new BT::SequenceNode("sequence0");
        BT:: SequenceNode* sequence1 = new BT::SequenceNode("sequence1");
        BT:: SequenceNode* sequence2 = new BT::SequenceNode("sequence2");
        BT:: SequenceNode* sequence3 = new BT::SequenceNode("sequence3");

        //Fallback//
        BT:: FallbackNode* fallback1 = new BT:: FallbackNode("fallback1");
        BT:: FallbackNode* fallback2 = new BT:: FallbackNode("fallback2");
        BT:: FallbackNode* fallback3 = new BT:: FallbackNode("fallback3");
        BT:: FallbackNode* fallback4 = new BT:: FallbackNode("fallback4");
        BT:: FallbackNode* fallback5 = new BT:: FallbackNode("fallback5");
        BT:: FallbackNode* fallback6 = new BT:: FallbackNode("fallback6");
        BT:: FallbackNode* fallback7 = new BT:: FallbackNode("fallback7");
        BT:: FallbackNode* fallback8 = new BT:: FallbackNode("fallback8");


        fallback1->AddChild(batteryOK);
        fallback1->AddChild(sequence1);

        sequence1->AddChild(fallback3);
        sequence1->AddChild(fallback4);
        sequence1->AddChild(fallback5);
        sequence1->AddChild(fallback6);
        sequence1->AddChild(fallback7);
        sequence1->AddChild(fallback8);

        fallback3->AddChild(kitCmd);
        fallback3->AddChild(GoToKit);

        fallback4->AddChild(loungeCmd);
        fallback4->AddChild(GoTolounge);

        fallback5->AddChild(entranceCmd);
        fallback5->AddChild(GoToEntrance);

        fallback6->AddChild(lobbycmd);
        fallback6->AddChild(GoToLobby);

        fallback7->AddChild(tvRoomcmd);
        fallback7->AddChild(GoToTvroom);

        fallback8->AddChild(bedRoomcmd);
        fallback8->AddChild(GoTobedRoom);

        //Execute the BT from Root Node//
        Execute(fallback1, TickPeriod_milliseconds, &n);  // from BehaviorTree.cpp

    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}
