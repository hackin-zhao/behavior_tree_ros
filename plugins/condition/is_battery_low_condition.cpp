#include <string>

#include "behavior_tree_ros/plugins/condition/is_battery_low_condition.hpp"

namespace behavior_tree
{
  IsBatteryLowCondition::IsBatteryLowCondition(
      const std::string &condition_name,
      const BT::NodeConfiguration &conf)
      : BT::ConditionNode(condition_name, conf),
        battery_topic_("/battery_status"),
        min_battery_(0.15),
        is_voltage_(false),
        is_battery_low_(false)
  {
    getInput("min_battery", min_battery_);
    getInput("battery_topic", battery_topic_);
    getInput("is_voltage", is_voltage_);

    node_ = config().blackboard->template get<std::shared_ptr<ros::NodeHandle>>("node_handle");
    battery_sub_opts_ = ros::SubscribeOptions::create<sensor_msgs::BatteryState>(battery_topic_, 1,
                                                                                 boost::bind(&IsBatteryLowCondition::batteryCallback, this, _1),
                                                                                 ros::VoidPtr(), &battery_queue_);

    battery_sub_ = node_->subscribe(battery_sub_opts_);
  }

  BT::NodeStatus IsBatteryLowCondition::tick()
  {
    battery_queue_.callAvailable(ros::WallDuration(0.01));
    if (is_battery_low_)
    {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  void IsBatteryLowCondition::batteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg)
  {
    if (is_voltage_)
    {
      is_battery_low_ = msg->voltage <= min_battery_;
    }
    else
    {
      is_battery_low_ = msg->percentage <= min_battery_;
    }
  }

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_tree::IsBatteryLowCondition>("IsBatteryLow");
}
