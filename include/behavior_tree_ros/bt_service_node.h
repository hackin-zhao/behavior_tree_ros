// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
#define _BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

#include <string>
#include <memory>

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include "behavior_tree_ros/bt_conversions.hpp"

namespace behavior_tree
{
  /**
   * Base Action to implement a ROS Service
   */
  template <class ServiceT>
  class RosServiceNode : public BT::SyncActionNode
  {
  protected:
    RosServiceNode(const std::string &service_name, const std::string &name, const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(name, conf), srv_name(service_name)
    {
      node_ = config().blackboard->template get<std::shared_ptr<ros::NodeHandle>>("node_handle");
    }

  private:
    const std::string srv_name;

  public:
    using BaseClass = RosServiceNode<ServiceT>;
    using ServiceType = ServiceT;
    using RequestType = typename ServiceT::Request;
    using ResponseType = typename ServiceT::Response;

    RosServiceNode() = delete;

    virtual ~RosServiceNode() = default;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return {};
    }

    /// User must implement this method.
    virtual void sendRequest(RequestType &request) = 0;

    /// Method (to be implemented by the user) to receive the reply.
    /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
    virtual BT::NodeStatus onResponse(const ResponseType &rep) = 0;

    enum FailureCause
    {
      MISSING_SERVER = 0,
      FAILED_CALL = 1
    };

    std::map<const enum FailureCause, const std::string> FailureCauseToString {
      {MISSING_SERVER, "MISSING_SERVER"},
      {FAILED_CALL, "FAILED_CALL"}
    };

    /// Called when a service call failed. Can be overriden by the user.
    virtual BT::NodeStatus onFailedRequest(FailureCause failure)
    {
      return BT::NodeStatus::FAILURE;
    }

  protected:
    ros::ServiceClient service_client_;

    typename ServiceT::Response reply_;

    // The node that will be used for any ROS operations
    std::shared_ptr<ros::NodeHandle> node_;

    BT::NodeStatus tick() override
    {
      if (!service_client_.isValid())
      {
        service_client_ = node_->serviceClient<ServiceT>(srv_name);
      }

      bool connected = service_client_.waitForExistence(ros::Duration(20.0));
      if (!connected)
      {
        return onFailedRequest(MISSING_SERVER);
      }

      typename ServiceT::Request request;
      sendRequest(request);
      bool received = service_client_.call(request, reply_);
      if (!received)
      {
        return onFailedRequest(FAILED_CALL);
      }
      return onResponse(reply_);
    }
  };

} // namespace behavior_tree

#endif // _BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
