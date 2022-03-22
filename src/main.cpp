#include <ros/ros.h>
#include "behavior_tree_ros/bt_task_manager.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_manager");

    auto task_manager_node = std::make_shared<bt_task_manager::BtTaskManager>();

    ros::spin();
    ros::shutdown();

    return 0;
}
