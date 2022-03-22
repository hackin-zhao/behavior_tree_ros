#include <string>
#include <memory>

#include <behaviortree_cpp_v3/decorator_node.h>

#include "behavior_tree_ros/plugins/decorator/cmd_updater_node.h"

namespace behavior_tree
{
  CmdUpdater::CmdUpdater(
      const std::string &xml_tag_name,
      const BT::NodeConfiguration &conf)
      : BT::DecoratorNode(xml_tag_name, conf)
  {
    cmd_updater_sub_opts_ = ros::SubscribeOptions::create<std_msgs::String>("/control_cmd", 1,
                                                                                 boost::bind(&CmdUpdater::cmdUpdatedCallback, this, _1),
                                                                                 ros::VoidPtr(), &cmd_updater_queue_);

    cmd_updater_sub_ = n_.subscribe(cmd_updater_sub_opts_);
  }

  inline BT::NodeStatus CmdUpdater::tick()
  {
    // geometry_msgs::msg::PoseStamped goal;

    // getInput("input_goal", goal);

    ros::spinOnce();

    // if (rclcpp::Time(last_goal_received_.header.stamp) > rclcpp::Time(goal.header.stamp))
    // {
    //   goal = last_goal_received_;
    // }

    // setOutput("output_goal", goal);
    return child_node_->executeTick();
  }

  void CmdUpdater::cmdUpdatedCallback(const std_msgs::String::ConstPtr &msg)
  {
    ROS_INFO_STREAM("receive meg: " << *msg);
    last_cmd_received_ = *msg;
  }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_tree::CmdUpdater>("CmdUpdater");
}
