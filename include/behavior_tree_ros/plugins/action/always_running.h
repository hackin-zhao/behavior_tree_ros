#if !defined(_BEHAVIOR_TREE__PLUGINS__ACTION__ALWAYS_RUNNING_H_)
#define _BEHAVIOR_TREE__PLUGINS__ACTION__ALWAYS_RUNNING_H_

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>

namespace behavior_tree
{
    class AlwaysRunning : public BT::AsyncActionNode
    {
    public:
        AlwaysRunning(const std::string &name, const BT::NodeConfiguration &config)
            : BT::AsyncActionNode(name, config)
        {
            _aborted = false;
        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

        virtual BT::NodeStatus tick() override
        {
            printf("Running......\n");
            ros::Duration(3).sleep();
            return BT::NodeStatus::SUCCESS;
        }

        virtual void halt() override
        {
            _aborted = true;
        }

    private:
        bool _aborted;
    };
} // namespace behavior_tree

#endif // _BEHAVIOR_TREE__PLUGINS__ACTION__ALWAYS_RUNNING_H_
