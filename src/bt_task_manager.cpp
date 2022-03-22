#include <fstream>

#include "behavior_tree_ros/bt_task_manager.h"
#include "behavior_tree_ros/rosout_logger.h"

namespace bt_task_manager
{
    BtTaskManager::BtTaskManager(/* args */)
    {
        nh_ptr_ = std::make_shared<ros::NodeHandle>("~");

        plugin_lib_names_.clear();
        nh_ptr_->param<std::vector<std::string>>("behavior_tree_plugins", plugin_lib_names_, plugin_lib_names_);
        nh_ptr_->param<std::string>("behavior_tree_xml_filename", default_bt_xml_filename_, "");
        nh_ptr_->param<bool>("enable_groot_monitoring", enable_groot_monitoring_, false);
        nh_ptr_->param<bool>("enable_behavior_tree_log", enable_behavior_tree_log_, false);

        control_cmd_sub_ = nh_ptr_->subscribe("/control_cmd", 1, &BtTaskManager::controlCmdCallback, this);

        bt_ = std::make_unique<behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);
        blackboard_ = BT::Blackboard::create();
        blackboard_->set<std::shared_ptr<ros::NodeHandle>>("node_handle", nh_ptr_);
        // blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_); // NOLINT
        // blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);     // NOLINT

        if (!loadBehaviorTree(default_bt_xml_filename_))
        {
            ROS_ERROR("Error loading XML file: %s", default_bt_xml_filename_.c_str());
        }

        if (enable_groot_monitoring_)
        {
            int publisher_port, server_port;
            nh_ptr_->param<int>("groot_zmq_publisher_port", publisher_port, 1666);
            nh_ptr_->param<int>("groot_zmq_server_port", server_port, 1667);

            try
            {
                bt_->addGrootMonitoring(&tree_, publisher_port, server_port);
            }
            catch (const std::logic_error &e)
            {
                ROS_ERROR("ZMQ already enabled, Error: %s", e.what());
            }
        }

        ROS_INFO("bt task manager initial success!");
    }

    BtTaskManager::~BtTaskManager()
    {
        if (enable_groot_monitoring_)
        {
            bt_->resetGrootMonitor();
        }
    }

    void BtTaskManager::controlCmdCallback(const std_msgs::String::ConstPtr &msg)
    {
        ROS_INFO_STREAM("receive cmd: " << msg->data);

        std::promise<behavior_tree::BtStatus> p;
        std::thread t(std::bind(&BtTaskManager::executeCallback, this, std::ref(p)));

        std::future<behavior_tree::BtStatus> f = p.get_future();

        switch (f.get())
        {
        case behavior_tree::BtStatus::SUCCEEDED:
            ROS_INFO("task complete");
            break;

        case behavior_tree::BtStatus::FAILED:
            ROS_INFO("task failed");
            break;

        case behavior_tree::BtStatus::CANCELED:
            ROS_INFO("task canceled");
            break;
        }
        // std::cout << "get promis future is:" << f.get() << std::endl;
        t.join();
    }

    bool BtTaskManager::loadBehaviorTree(const std::string &bt_xml_filename)
    {
        // Empty filename is default for backward compatibility
        auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

        // Use previous BT if it is the existing one
        if (current_bt_xml_filename_ == filename)
        {
            ROS_DEBUG("BT will not be reloaded as the given xml is already loaded");
            return true;
        }

        if (enable_groot_monitoring_)
        {
            bt_->resetGrootMonitor();
        }

        // Read the input BT XML from the specified file into a string
        std::ifstream xml_file(filename);

        if (!xml_file.good())
        {
            ROS_ERROR("Couldn't open input XML file: %s", filename.c_str());
            return false;
        }

        auto xml_string = std::string(std::istreambuf_iterator<char>(xml_file),
                                      std::istreambuf_iterator<char>());

        // Create the Behavior Tree from the XML input
        tree_ = bt_->createTreeFromText(xml_string, blackboard_);

        current_bt_xml_filename_ = filename;
        return true;
    }

    void BtTaskManager::executeCallback(std::promise<behavior_tree::BtStatus> &p)
    {
        auto is_canceling = [&]()
        { return false; };
        auto on_loop = [&]() {};

        std::unique_ptr<BT::RosoutLogger> topic_logger;

        if (enable_behavior_tree_log_)
        {
            topic_logger = std::make_unique<BT::RosoutLogger>(tree_.rootNode());
        }

        // Execute the BT that was previously created in the configure step
        behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

        p.set_value(rc);

        // Make sure that the Bt is not in a running state from a previous execution
        // note: if all the ControlNodes are implemented correctly, this is not needed.
        bt_->haltAllActions(tree_.rootNode());
    }

} // namespace bt_task_manager
