// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BT_PATROLLING__CTRL_SUPPORT__BTACTIONNODE__HPP_
#define BT_PATROLLING__CTRL_SUPPORT__BTACTIONNODE__HPP_

#include <memory>
#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace bt_patrolling
{
using namespace std::chrono_literals;
template<class ActionT, class NodeT = rclcpp::Node>
class BtActionNode : public BT::ActionNodeBase
{
public:
    BtActionNode(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf)
        : BT::ActionNodeBase(xml_tag_name, conf), action_name(action_name)
        {
            node_ = config().blackboard->get<typename NodeT::SharedPtr>("node");
            server_timeout_ = 1s;

            goal_ = typename ActionT::Goal();
            result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrapperResult();

            std::string remapped_action_name;
            if(getInput("server_name", remapped_action_name))
            {
                action_name_ = remapped_action_name;
            }
            createActionClient(action_name)

            RCLCPP_INFO(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
        }

        BtActionNode() = delete;

        virtual ~BtActionNode()
        {
        }

        void createActionClient(const std::string & action_name)
        {
            action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

            RCLCPP_INFO(node->get_logger(), "Wating for \"%s\" action server", action_name.c_str());

            action_client_->wait_for_action_server();
        }

        static BT::PortsList providedBasicPorts(BT::PortsList addition)
        {
            BT::PortsList basic = {
                BT::InputPort<std::string>("server_name", "Action server name"),
                BT::InputPort<std::chrono::milliseconds>("server_timeout")
            };
            basic.insert(addition.begin(), addition.end());

            return basic;
        }

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({});
        }

        virtual void on_tick()
        {
        }

        virtual void on_wait_for_result()
        {
        }

        virtual BT::NodeStatus on_success()
        {
            return BT::NodeStatus::SUCCESS;
        }

        virtual BT::NodeStatus on_aborted()
        {
            return BT::NodeStatus::FAILURE;
        }

        virtual BT::NodeStatus on_cancelled()
        {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus tick() override
        {
            if(status() == BT::NodeStatus::IDLE)
            {
                createActionClient(action_name);

                setStatus(BT::NodeStatus::RUNNING);

                on_tick();

                on_new_goal_received();
            }

            if(rclcpp::ok() && !goal_result_available_)
            {
                on_wait_for_result();

                auto goal_status = goal_handle->get_status();
                if(goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING || goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
                {
                    goal_updated_ = false;
                    on_new_goal_received();
                }

                rclcpp::spin_some(node_->get_node_base_interface());

                if(!goal_result_available_)
                {
                    return BT::NodeStatus::RUNNING;
                }
            }

            switch(result_.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    return on_success();
                
                case rclcpp_action::ResultCode::ABORTED:
                    return on_aborted();
                
                case rclcpp_action::ResultCode::CANCELED:
                    return on_cancelled();

                default:
                    throw std::logic_error("BtActionNode::Tick: invalid status value");
            }
        }

        void halt() override
        {
            if(should_cancel_goal())
            {
                auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
                if(rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_cancel, server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to cancel action server for %s", action_name_.c_str());

                }
            }

            setStatus(BT::NodeStatus::IDLE);
        }


protected:
    bool should_cancel_goal()
    {
        if(status() != BT::NodeStatus::RUNNING)
        {
            return false;
        }

        rclcpp::spin_some(node_->get_node_base_interface());
        auto status = goal_handle_->get_status();

        return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED || status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;

    }    

    void on_new_goal_received()
    {
        goal_result_available_ = false;
        auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
        send_goal_options.result_callback = [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)
        {
            if(this->goal_handle_->get_goal_id() == result.goal_id)
            {
                goal_result_available_ = true;
                result_ = result;
            }
        };

        auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);

        if(rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_goal_handle, server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
        {
            throw std::runtime_error("send_goal failed");
        }

        goal_handle_ = future_goal_handle.get();
        if(!goal_handle_)
        {
            throw std::runtime_error("Goal was rejected by the action server");
        }
    }

    void increment_recovery_count()
    {
        int recovery_count = 0;
        config().blackboard->get<int>("number_recoveries", recovery_count);
        recovery_count += 1;
        config().blackboard->set<int>("number_recoveries", recovery_count);
    }

    std::string action_name;
    typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

    typename ActionT::Goal goal_;
    bool goal_updated_{false};
    bool goal_result_available_{false};
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

    typename NodeT::SharedPtr node_;

    std::chrono::milliseconds server_timeout_;
};

}

#endif