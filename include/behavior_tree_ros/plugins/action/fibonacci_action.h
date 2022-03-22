#ifndef _BEHAVIOR_TREE__PLUGINS__ACTION__FIBONACCI_ACTION_H_
#define _BEHAVIOR_TREE__PLUGINS__ACTION__FIBONACCI_ACTION_H_

#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behavior_tree_ros/FibonacciAction.h>

#include "behavior_tree_ros/bt_action_node.h"

namespace behavior_tree
{
    class FibonacciServer : public RosActionNode<behavior_tree_ros::FibonacciAction>
    {

    public:
        FibonacciServer(const std::string &server_name, const std::string &name, const BT::NodeConfiguration &conf);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("order"),
                BT::OutputPort<int>("result")};
        }

        bool sendGoal(GoalType &goal) override;
        BT::NodeStatus onResult(const ResultType &res) override;
        virtual BT::NodeStatus onFailedRequest(FailureCause failure) override;
        void halt() override;

    private:
        int expected_result_;
    };

} // namespace behavior_tree

#endif // _BEHAVIOR_TREE__PLUGINS__ACTION__FIBONACCI_ACTION_H_
