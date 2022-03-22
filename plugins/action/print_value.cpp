#include <string>
#include "behavior_tree_ros/plugins/action/print_value.h"

namespace behavior_tree
{
    PrintValue::PrintValue(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::PrintValue>("PrintValue");
}
