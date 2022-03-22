#include <string>
#include "behavior_tree_ros/plugins/action/add_two_int.h"

namespace behavior_tree
{
    AddTwoIntsAction::AddTwoIntsAction(
        const std::string &service_name,
        const std::string &name,
        const BT::NodeConfiguration &conf)
        : RosServiceNode<behavior_tree_ros::AddTwoInts>(service_name, name, conf)
    {
    }

    void AddTwoIntsAction::sendRequest(RequestType &request)
    {
        getInput("first_int", request.a);
        getInput("second_int", request.b);
        expected_result_ = request.a + request.b;
        ROS_INFO("AddTwoInts: sending request");
    }

    BT::NodeStatus AddTwoIntsAction::onResponse(const ResponseType &rep)
    {
        ROS_INFO("AddTwoInts: response received");
        if (rep.sum == expected_result_)
        {
            setOutput<int>("sum", rep.sum);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("AddTwoInts replied something unexpected: %d", rep.sum);
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus AddTwoIntsAction::onFailedRequest(RosServiceNode::FailureCause failure)
    {
        ROS_ERROR_STREAM("AddTwoInts request failed: " << FailureCauseToString[failure]);
        return BT::NodeStatus::FAILURE;
    }

} // namespace behavior_tree

#include <behaviortree_cpp_v3/bt_factory.h>
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<behavior_tree::AddTwoIntsAction>("/add_two_ints", name, config);
    };

    BT::TreeNodeManifest manifest;
    manifest.type = BT::getType<behavior_tree::AddTwoIntsAction>();
    manifest.ports = behavior_tree::AddTwoIntsAction::providedPorts();
    manifest.registration_ID = "AddTwoInts";
    factory.registerBuilder(manifest, builder);
}
