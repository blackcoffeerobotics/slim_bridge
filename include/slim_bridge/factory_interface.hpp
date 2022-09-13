/*
 * Copyright 2020, 2021, 2022 Black Coffee Robotics LLP (https://www.blackcoffeerobotics.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_SLIM_BRIDGE_FACTORY_INTERFACE_HPP_
#define INCLUDE_SLIM_BRIDGE_FACTORY_INTERFACE_HPP_


#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros/ros.h"

namespace slim_bridge {

class FactoryInterface {
    public:
        virtual
        rclcpp::PublisherBase::SharedPtr
        create_ros2_publisher(
            rclcpp::Node::SharedPtr ros2_node,
            const std::string & topic_name,
            const rclcpp::QoS & qos) = 0;

        virtual
        std::shared_ptr<ros::Publisher>
        create_ros1_publisher(
            std::shared_ptr<ros::NodeHandle> ros1_node,
            const std::string & topic_name,
            const size_t queue_size,
            const bool latch) = 0;

        virtual
        rclcpp::SubscriptionBase::SharedPtr
        create_ros2_subscriber(
            rclcpp::Node::SharedPtr ros2_node,
            const std::string & topic_name,
            const rclcpp::QoS & qos,
            const std::shared_ptr<ros::Publisher> publisher) = 0;

        virtual
        std::shared_ptr<ros::Subscriber>
        create_ros1_subscriber(
            std::shared_ptr<ros::NodeHandle> ros1_node,
            const std::string & topic_name,
            const size_t queue_size,
            const rclcpp::PublisherBase::SharedPtr publisher) = 0;
};

}  // namespace slim_bridge

#endif  //  INCLUDE_SLIM_BRIDGE_FACTORY_INTERFACE_HPP_
