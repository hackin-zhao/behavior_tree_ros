#ifndef _BEHAVIOR_TREE__PLUGINS__ACTION__ADD_TWO_INT_H_
#define _BEHAVIOR_TREE__PLUGINS__ACTION__ADD_TWO_INT_H_

#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include "behavior_tree_ros/bt_service_node.h"
#include <behavior_tree_ros/AddTwoInts.h>

namespace behavior_tree
{
    class AddTwoIntsAction : public RosServiceNode<behavior_tree_ros::AddTwoInts>
    {

    public:
        AddTwoIntsAction(const std::string &service_name, const std::string &node_name, const ::BT::NodeConfiguration &conf);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("first_int"),
                BT::InputPort<int>("second_int"),
                BT::OutputPort<int>("sum")};
        }

        void sendRequest(RequestType &request) override;

        BT::NodeStatus onResponse(const ResponseType &rep) override;

        virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override;

    private:
        int expected_result_;
    };

} // namespace behavior_tree

#endif // _BEHAVIOR_TREE__PLUGINS__ACTION__ADD_TWO_INT_H_
