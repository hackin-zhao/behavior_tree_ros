#ifndef _BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
#define _BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/BatteryState.h>
#include <behaviortree_cpp_v3/condition_node.h>

namespace behavior_tree
{

  /**
 * @brief A BT::ConditionNode that listens to a battery topic and
 * returns SUCCESS when battery is low and FAILURE otherwise
 */
  class IsBatteryLowCondition : public BT::ConditionNode
  {
  public:
    /**
   * @brief A constructor for behavior_tree::IsBatteryLowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
    IsBatteryLowCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf);

    IsBatteryLowCondition() = delete;

    /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
    BT::NodeStatus tick() override;

    /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<double>("min_battery", "Minimum battery percentage/voltage"),
          BT::InputPort<std::string>(
              "battery_topic", std::string("/battery_status"), "Battery topic"),
          BT::InputPort<bool>(
              "is_voltage", false, "If true voltage will be used to check for low battery"),
      };
    }

  private:
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg);

    std::shared_ptr<ros::NodeHandle> node_;
    ros::Subscriber battery_sub_;
    ros::SubscribeOptions battery_sub_opts_;
    ros::CallbackQueue battery_queue_;
    std::string battery_topic_;
    double min_battery_;
    bool is_voltage_;
    bool is_battery_low_;
  };

} // namespace behavior_tree

#endif // _BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
