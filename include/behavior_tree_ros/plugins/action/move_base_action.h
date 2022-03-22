#ifndef _BEHAVIOR_TREE__PLUGINS__MOVE_BASE_ACTION_CLIENT_H_
#define _BEHAVIOR_TREE__PLUGINS__MOVE_BASE_ACTION_CLIENT_H_

#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "behavior_tree_ros/bt_action_node.h"

namespace behavior_tree
{
    class MoveBaseActionClient : public RosActionNode<move_base_msgs::MoveBaseAction>
    {

    public:
        MoveBaseActionClient(const std::string &server_name, const std::string &name, const BT::NodeConfiguration &conf);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<geometry_msgs::PoseStamped>("target_pose", "target pose to plan to")};
        }

        virtual bool sendGoal(GoalType &goal) override;
        virtual BT::NodeStatus onResult(const ResultType &res) override;
        virtual BT::NodeStatus onFailedRequest(FailureCause failure) override;
        virtual void halt() override;

    private:
        int expected_result_;
    };

} // namespace behavior_tree

#endif // _BEHAVIOR_TREE__PLUGINS__MOVE_BASE_ACTION_CLIENT_H_
