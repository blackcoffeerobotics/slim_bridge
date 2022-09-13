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

#ifndef INCLUDE_SLIM_BRIDGE_FACTORY_HPP_
#define INCLUDE_SLIM_BRIDGE_FACTORY_HPP_

#include <string>
#include <memory>
#include "factory_interface.hpp"
#include "convert.hpp"

namespace slim_bridge {
/*
* Factory Design Pattern: The factory is for creating objects
* that convert a certain message from ROS1 to ROS2 or vice versa.
* The inbuilt ros1_callback and ros2_callback are determined by
* the template inputs. This function is the one requiring explicit
* template specialization.
*/
template<typename ROS1_T, typename ROS2_T>
class Factory : public FactoryInterface {
    public:
        Factory(const std::string package_name,
            const std::string message_type):
            _package_name(package_name),
            _message_type(message_type)
        {}
        rclcpp::PublisherBase::SharedPtr
        create_ros2_publisher(
            rclcpp::Node::SharedPtr ros2_node,
            const std::string & topic_name,
            const rclcpp::QoS & qos
        ) {
            auto returnable = ros2_node->create_publisher<ROS2_T>(
                topic_name, qos);
            return returnable;
        }

        std::shared_ptr<ros::Publisher>
        create_ros1_publisher(
            std::shared_ptr<ros::NodeHandle> ros1_node,
            const std::string & topic_name,
            const size_t queue_size,
            const bool latch
        ) {
            auto returnable = std::make_shared<ros::Publisher>();
            *(returnable.get()) = ros1_node->advertise<ROS1_T>(
                topic_name,
                queue_size,
                latch);
            return returnable;
        }

        rclcpp::SubscriptionBase::SharedPtr
        create_ros2_subscriber(
            rclcpp::Node::SharedPtr ros2_node,
            const std::string & topic_name,
            const rclcpp::QoS & qos,
            std::shared_ptr<ros::Publisher> publisher
        ) {
            return ros2_node->create_subscription<ROS2_T>(
                topic_name,
                qos,
                [=](const std::shared_ptr<ROS2_T> msg) {
                    ros2_callback(msg, publisher);
                });
        }

        std::shared_ptr<ros::Subscriber>
        create_ros1_subscriber(
            std::shared_ptr<ros::NodeHandle> ros1_node,
            const std::string & topic_name,
            const size_t queue_size,
            rclcpp::PublisherBase::SharedPtr publisher
        ) {
            auto returnable = std::make_shared<ros::Subscriber>();
            *(returnable.get()) = ros1_node->subscribe<ROS1_T>(
                topic_name,
                queue_size,
                [=](const boost::shared_ptr<ROS1_T const>& msg) {
                    ros1_callback(msg, publisher);
                });
            return returnable;
        }

        std::string _package_name;
        std::string _message_type;

    protected:
        static void
        ros2_callback(
            std::shared_ptr<const ROS2_T> ros2_msg,
            const std::shared_ptr<ros::Publisher> & ros1_publisher
        ) {
            ROS1_T ros1_msg;
            convert_ros2_to_1(*(ros2_msg.get()), ros1_msg);
            ros1_publisher->publish(ros1_msg);
        }

        static void
        ros1_callback(
            boost::shared_ptr<ROS1_T const> ros1_msg,
            const rclcpp::PublisherBase::SharedPtr & ros2_publisher
        ) {
            ROS2_T ros2_msg;
            convert_ros1_to_2(*(ros1_msg.get()), ros2_msg);
            std::shared_ptr<rclcpp::Publisher<ROS2_T>> publisher
                = std::dynamic_pointer_cast<rclcpp::Publisher<ROS2_T>>
                    (ros2_publisher);
            publisher->publish(ros2_msg);
        }
};
}  // namespace slim_bridge

#endif  //  INCLUDE_SLIM_BRIDGE_FACTORY_HPP_
