#if !defined(_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_VALUE__H_)
#define _BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_VALUE__H_

#include <behaviortree_cpp_v3/action_node.h>

namespace behavior_tree
{

    class PrintValue : public BT::SyncActionNode
    {
    public:
        PrintValue(const std::string &name, const BT::NodeConfiguration &config);

        BT::NodeStatus tick() override
        {
            int value = 0;
            if (getInput("message", value))
            {
                std::cout << "PrintValue: " << value << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "PrintValue FAILED " << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<int>("message")};
        }
    };

} // namespace behavior_tree

#endif // _BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_VALUE__H_
