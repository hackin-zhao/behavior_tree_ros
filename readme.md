# behavior tree ros

基于 [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) 的 ROS 封装，能够使得
项目便于工程化落地。

# 目前支持的功能

* service，action 封装
* 通过加载库的方式方便叶子节点的灵活添加
* behavior tree log 通过 ROS log 接口输出
* 支持 Groot

# todo

* 开发 behavior_tree.xml 编译工具以提高程序加载速度
* 通用机器人功能叶子节点开发
* 添加单元测试模块
* behavior_tree 性能评价工具
