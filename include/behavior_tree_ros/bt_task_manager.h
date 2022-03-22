#if !defined(_BT_TASK_MANAGER_)
#define _BT_TASK_MANAGER_

#include <ros/ros.h>

#include <std_msgs/String.h>

#include <behaviortree_cpp_v3/behavior_tree.h>

#include "behavior_tree_ros/behavior_tree_engine.hpp"

namespace bt_task_manager
{
    class BtTaskManager
    {
    private:
        /* data */
    protected:
        std::unique_ptr<behavior_tree::BehaviorTreeEngine> bt_;
        BT::Tree tree_;
        BT::Blackboard::Ptr blackboard_;
        ros::Duration bt_loop_duration_;

        std::string current_bt_xml_filename_;
        std::string default_bt_xml_filename_;

        std::vector<std::string> plugin_lib_names_;

        std::shared_ptr<ros::NodeHandle> nh_ptr_;

        ros::Subscriber control_cmd_sub_;

        std::unique_ptr<std::thread> bt_thread_ptr_;

        std::promise<BT::NodeStatus> bt_task_status_;

        bool enable_groot_monitoring_;
        bool enable_behavior_tree_log_;

        void controlCmdCallback(const std_msgs::String::ConstPtr &msg);

    public:
        BtTaskManager(/* args */);
        ~BtTaskManager();

        void haltTree() { tree_.rootNode()->halt(); }

        bool loadBehaviorTree(const std::string &bt_xml_filename = "");

        void executeCallback(std::promise<behavior_tree::BtStatus> &p);
    };

} // namespace bt_task_manager

#endif // _BT_TASK_MANAGER_
