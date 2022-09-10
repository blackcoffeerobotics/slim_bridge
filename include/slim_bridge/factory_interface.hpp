#ifndef __FACTORY_INTERFACE_HPP__
#define __FACTORY_INTERFACE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "ros/ros.h"

#include <memory>
#include <string>

namespace slim_bridge 
{

    class FactoryInterface
    {

        public:

            virtual
            rclcpp::PublisherBase::SharedPtr
            create_ros2_publisher(
                rclcpp::Node::SharedPtr ros2_node,
                const std::string & topic_name,
                const rclcpp::QoS & qos
            ) = 0;

            virtual
            std::shared_ptr<ros::Publisher>
            create_ros1_publisher(
                std::shared_ptr<ros::NodeHandle> ros1_node,
                const std::string & topic_name,
                const size_t queue_size,
                const bool latch
            ) = 0;

            virtual
            rclcpp::SubscriptionBase::SharedPtr
            create_ros2_subscriber(
                rclcpp::Node::SharedPtr ros2_node,
                const std::string & topic_name,
                const rclcpp::QoS & qos,
                const std::shared_ptr<ros::Publisher> publisher
            ) = 0;

            virtual
            std::shared_ptr<ros::Subscriber>
            create_ros1_subscriber(
                std::shared_ptr<ros::NodeHandle> ros1_node,
                const std::string & topic_name,
                const size_t queue_size,
                const rclcpp::PublisherBase::SharedPtr publisher
            ) = 0;

    };

}

#endif