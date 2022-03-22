#include <string>

#include <ros/ros.h>
#include <behavior_tree_ros/FibonacciAction.h>
#include "behavior_tree_ros/plugins/action/move_base_action.h"
#include <behavior_tree_ros/bt_conversions.hpp>

namespace behavior_tree
{
    MoveBaseActionClient::MoveBaseActionClient(const std::string &server_name, const std::string &name, const BT::NodeConfiguration &conf)
        : RosActionNode<move_base_msgs::MoveBaseAction>(server_name, name, conf) {}

    bool MoveBaseActionClient::sendGoal(GoalType &goal)
    {
        if (!getInput<geometry_msgs::PoseStamped>("target_pose", goal.target_pose))
        {
            return false;
        }

        ROS_INFO_STREAM("move base action client sending request: " << goal);

        return true;
    }

    BT::NodeStatus MoveBaseActionClient::onResult(const ResultType &res)
    {
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus MoveBaseActionClient::onFailedRequest(FailureCause failure)
    {
        ROS_ERROR_STREAM("MoveBaseActionClient request failed: " << FailureCauseToString[failure]);
        return BT::NodeStatus::FAILURE;
    }

    void MoveBaseActionClient::halt()
    {
        if (status() == BT::NodeStatus::RUNNING)
        {
            ROS_WARN("MoveBaseActionClient halted");
            BaseClass::halt();
        }
    }

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<behavior_tree::MoveBaseActionClient>("/move_base", name, config);
    };

    BT::TreeNodeManifest manifest;
    manifest.type = BT::getType<behavior_tree::MoveBaseActionClient>();
    manifest.ports = behavior_tree::MoveBaseActionClient::providedPorts();
    manifest.registration_ID = "MoveBaseAction";
    factory.registerBuilder(manifest, builder);
}
