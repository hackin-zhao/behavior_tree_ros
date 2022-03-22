#include <string>
#include "behavior_tree_ros/plugins/action/always_running.h"
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::AlwaysRunning>("AlwaysRunning");
}
