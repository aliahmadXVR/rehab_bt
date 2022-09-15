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
    ros::init(argc, argv, "demo3");
    ros::NodeHandle n;
    try
    {
        int TickPeriod_milliseconds = 100; //100ms tested on actual robot. testing same in simulation.
        //Condition Nodes//
        BT::ROSCondition* GoTo_cmd    = new BT::ROSCondition("GoTo_cmd");
        BT::ROSCondition* batteryOK   = new BT::ROSCondition("batteryOK");
        BT::ROSCondition* heyRuyi_cmd = new BT::ROSCondition("heyRuyi_cmd");
        BT::ROSCondition* FallHaz_cmd  = new BT::ROSCondition("FallHaz_cmd");

       
        //Action Nodes//
        BT::ROSAction* GoTo_behav     = new BT::ROSAction("GoTo_behav");
        BT::ROSAction* search_person  = new BT::ROSAction("search_person");
        BT::ROSAction* FallHaz_scan    = new BT::ROSAction("FallHaz_scan");
        
        //Sequences//
        BT:: SequenceNode* sequence1 = new BT::SequenceNode("sequence1");

        //Fallback//
        BT:: FallbackNode* fallback1 = new BT:: FallbackNode("fallback1");
        BT:: FallbackNode* fallback2 = new BT:: FallbackNode("fallback2");
        BT:: FallbackNode* fallback3 = new BT:: FallbackNode("fallback3");
        BT:: FallbackNode* fallback4 = new BT:: FallbackNode("fallback4");

        fallback1->AddChild(batteryOK);
        fallback1->AddChild(sequence1);

        sequence1->AddChild(fallback2);
        sequence1->AddChild(fallback3);
        sequence1->AddChild(fallback4);

        fallback2->AddChild(GoTo_cmd);
        fallback2->AddChild(GoTo_behav);

        fallback3->AddChild(heyRuyi_cmd);
        fallback3->AddChild(search_person);

        fallback4->AddChild(FallHaz_cmd);
        fallback4->AddChild(FallHaz_scan);


        //Execute the BT from Root Node//
        Execute(fallback1, TickPeriod_milliseconds, &n);  // from BehaviorTree.cpp

    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}
