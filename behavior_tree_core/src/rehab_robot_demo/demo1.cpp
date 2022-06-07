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
    ros::init(argc, argv, "Move_Robot_BT_demo");
    try
    {
        int TickPeriod_milliseconds = 50;


        BT::ROSAction* forward = new BT::ROSAction("forward");
        BT::ROSAction* reverse = new BT::ROSAction("reverse");
        BT::ROSAction* stop = new BT::ROSAction("stop");

        BT::ROSCondition* noforward_key = new BT::ROSCondition("noforward_key");
        BT::ROSCondition* noreverse_key = new BT::ROSCondition("noreverse_key");
        BT::ROSCondition* nokey_press = new BT::ROSCondition("get_key");
        BT::ROSCondition* stop_key = new BT::ROSCondition("no_stop_key");


        BT:: SequenceNode* sequence1 = new BT::SequenceNode("seq0");
        BT:: SequenceNode* sequence2 = new BT::SequenceNode("seq1");

        BT:: FallbackNode* fallback1 = new BT:: FallbackNode("fallback1");
        BT:: FallbackNode* fallback2 = new BT:: FallbackNode("fallback2");
        BT:: FallbackNode* fallback3 = new BT:: FallbackNode("fallback3");

        sequence1->AddChild(fallback1);
        sequence1->AddChild(fallback2);
        sequence1->AddChild(fallback3);

        fallback1->AddChild(noforward_key);
        fallback1->AddChild(forward);
       
        fallback2->AddChild(noreverse_key);
        fallback2->AddChild(reverse);
        
        fallback3->AddChild(stop_key);
        fallback3->AddChild(stop);

        Execute(sequence1, TickPeriod_milliseconds);  // from BehaviorTree.cpp

    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}
