#include <string>

#include <ros/ros.h>
#include <behavior_tree_ros/FibonacciAction.h>
#include "behavior_tree_ros/plugins/action/fibonacci_action.h"

namespace behavior_tree
{
    FibonacciServer::FibonacciServer(const std::string &server_name, const std::string &name, const BT::NodeConfiguration &conf)
        : RosActionNode<behavior_tree_ros::FibonacciAction>(server_name, name, conf) {}

    bool FibonacciServer::sendGoal(GoalType &goal)
    {
        if (!getInput<int>("order", goal.order))
        {
            // abourt the entire action. Result in a FAILURE
            return false;
        }
        expected_result_ = 0 + 1 + 1 + 2 + 3 + 5 + 8; // supposing order is 5
        ROS_INFO("FibonacciAction: sending request");
        return true;
    }

    BT::NodeStatus FibonacciServer::onResult(const ResultType &res)
    {
        ROS_INFO("FibonacciAction: result received");
        int fibonacci_result = 0;
        for (int n : res.sequence)
        {
            fibonacci_result += n;
        }
        if (fibonacci_result == expected_result_)
        {
            setOutput<int>("result", fibonacci_result);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("FibonacciAction replied something unexpected: %d", fibonacci_result);
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus FibonacciServer::onFailedRequest(FailureCause failure)
    {
        ROS_ERROR_STREAM("FibonacciAction request failed: " << FailureCauseToString[failure]);
        return BT::NodeStatus::FAILURE;
    }

    void FibonacciServer::halt()
    {
        if (status() == BT::NodeStatus::RUNNING)
        {
            ROS_WARN("FibonacciAction halted");
            BaseClass::halt();
        }
    }

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<behavior_tree::FibonacciServer>("/fibonacci", name, config);
    };

    BT::TreeNodeManifest manifest;
    manifest.type = BT::getType<behavior_tree::FibonacciServer>();
    manifest.ports = behavior_tree::FibonacciServer::providedPorts();
    manifest.registration_ID = "Fibonacci";
    factory.registerBuilder(manifest, builder);
}
