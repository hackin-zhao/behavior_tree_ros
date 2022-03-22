#ifndef _CMD_UPDATER_NODE_H_
#define _CMD_UPDATER_NODE_H_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

namespace behavior_tree
{

  /**
   * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
   * the current goal on the blackboard
   */
  class CmdUpdater : public BT::DecoratorNode
  {
  public:
    /**
     * @brief A constructor for nav2_behavior_tree::CmdUpdater
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    CmdUpdater(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<geometry_msgs::PoseStamped>("input_goal", "Original Goal"),
          BT::OutputPort<geometry_msgs::PoseStamped>(
              "output_goal",
              "Received Goal by subscription"),
      };
    }

  private:
    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Callback function for goal update topic
     * @param msg Shared pointer to geometry_msgs::msg::PoseStamped message
     */
    void cmdUpdatedCallback(const std_msgs::String::ConstPtr &msg);

    ros::NodeHandle n_;
    ros::Subscriber cmd_updater_sub_;
    ros::SubscribeOptions cmd_updater_sub_opts_;
    ros::CallbackQueue cmd_updater_queue_;
    std::string battery_topic_;

    std_msgs::String last_cmd_received_;
  };

} // namespace nav2_behavior_tree

#endif // _CMD_UPDATER_NODE_H_
