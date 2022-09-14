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

#include "slim_bridge/bridge.hpp"

// using namespace slim_bridge;

void get_robot_name(std::string& returnable, std::string& robot_name) {
    try {
        robot_name = std::string(std::getenv("ROBOT_NAME"));
        auto robot_name_split = slim_bridge::tokenize(robot_name);
        for (auto split : robot_name_split)
            returnable += split + "_";
    }
    catch (const std::exception& e) {
        std::cout << "No robot name specified, continuining without namespace"
            << std::endl;
        robot_name = "";
        returnable = "";
    }
    robot_name += "/";
}

int main(int argc, char * argv[]) {
    // robot_name_split is the robot name with underscores instead of '/'
    std::string robot_name, robot_name_split;
    get_robot_name(robot_name_split, robot_name);

    ros::init(argc, argv, robot_name_split+"slim_bridge");
    rclcpp::init(argc, argv);

    auto ros1_node = std::make_shared<ros::NodeHandle>();
    auto ros2_node = rclcpp::Node::make_shared(robot_name_split+"slim_bridge");
    RCLCPP_INFO(ros2_node->get_logger(), "Initializing slim bridge...");

    ros2_node->declare_parameter("initial_topics_filepath");
    std::cout << "Initial topics path: " << ros2_node->get_parameter
        ("initial_topics_filepath").as_string() << "\n";
    YAML::Node root = YAML::LoadFile(ros2_node->get_parameter
        ("initial_topics_filepath").as_string());

    YAML::Node ros2_to_1_topics = root["ros2_to_1"].as<YAML::Node>();
    YAML::Node ros1_to_2_topics = root["ros1_to_2"].as<YAML::Node>();

    auto bridge = std::make_shared<slim_bridge::BridgeHandle>();
    bridge->ros1_node = ros1_node;
    bridge->ros2_node = ros2_node;

    slim_bridge::BridgeOptions initial_topic_options;
    std::string initial_topic_package_name;
    std::string initial_topic_message_type;
    std::string initial_topic_name;

    for (YAML::const_iterator it = ros2_to_1_topics.begin();
        it != ros2_to_1_topics.end(); ++it) {
        initial_topic_name = it->first.as<std::string>();
        if (initial_topic_name.at(0) != '/')
            initial_topic_name = robot_name + initial_topic_name;
        read_yaml(
            it->second.as<YAML::Node>(),
            initial_topic_name,
            initial_topic_options,
            initial_topic_package_name,
            initial_topic_message_type);
        try {
            create_bridge_2_to_1(
                initial_topic_package_name,
                initial_topic_message_type,
                initial_topic_options,
                bridge);
        }
        catch(const std::exception & e) {
            std::cout << e.what() << "\n";
        }
    }

    for (YAML::const_iterator it = ros1_to_2_topics.begin();
        it != ros1_to_2_topics.end(); ++it) {
        initial_topic_name = it->first.as<std::string>();
        if (initial_topic_name.at(0) != '/') {
            initial_topic_name = robot_name + initial_topic_name;
        }
        read_yaml(
            it->second.as<YAML::Node>(),
            initial_topic_name,
            initial_topic_options,
            initial_topic_package_name,
            initial_topic_message_type);
        try {
            create_bridge_1_to_2(
                initial_topic_package_name,
                initial_topic_message_type,
                initial_topic_options,
                bridge);
        }
        catch(const std::exception & e) {
            std::cout << e.what() << "\n";
        }
    }

    auto sub1_to_2_ros1 = ros1_node->subscribe<std_msgs::String>(
        robot_name+"ros1_to_2", 100,
        [=](const boost::shared_ptr<std_msgs::String const>& msg) {
            slim_bridge::BridgeOptions options;
            std::string package_name;
            std::string message_type;
            slim_bridge::read_string(msg->data, options, package_name,
                message_type);
            try {
                create_bridge_1_to_2(
                    package_name,
                    message_type,
                    options,
                    bridge);
            }
            catch(const std::exception & e){
                std::cout << e.what() << "\n";
            }
        });

    auto sub2_to_1_ros1 = ros1_node->subscribe<std_msgs::String>(
        robot_name+"ros2_to_1", 100,
        [=](const boost::shared_ptr<std_msgs::String const>& msg) {
            slim_bridge::BridgeOptions options;
            std::string package_name;
            std::string message_type;
            slim_bridge::read_string(
                msg->data,
                options,
                package_name,
                message_type);
            try {
                create_bridge_2_to_1(
                    package_name,
                    message_type,
                    options,
                    bridge);
            }
            catch(const std::exception & e) {
                std::cout << e.what() << "\n";
            }
        });

    auto sub1_to_2_ros2 = ros2_node->create_subscription<std_msgs::msg::String>(
        robot_name+"ros1_to_2", reliable_qos,
        [=](const std::shared_ptr<std_msgs::msg::String> msg) {
            slim_bridge::BridgeOptions options;
            std::string package_name;
            std::string message_type;
            slim_bridge::read_string(
                msg->data,
                options,
                package_name,
                message_type);
            try {
                create_bridge_1_to_2(
                    package_name,
                    message_type,
                    options,
                    bridge);
            }
            catch(const std::exception & e) {
                std::cout << e.what() << "\n";
            }
        });

    auto sub2_to_1_ros2 = ros2_node->create_subscription<std_msgs::msg::String>(
        robot_name+"ros2_to_1", reliable_qos,
        [=](const std::shared_ptr<std_msgs::msg::String> msg) {
            slim_bridge::BridgeOptions options;
            std::string package_name;
            std::string message_type;
            slim_bridge::read_string(
                msg->data,
                options,
                package_name,
                message_type);
            try {
                create_bridge_2_to_1(
                    package_name,
                    message_type,
                    options,
                    bridge);
            }
            catch(const std::exception & e) {
                std::cout << e.what() << "\n";
            }
        });

    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    rclcpp::executors::MultiThreadedExecutor executor;
    RCLCPP_INFO(ros2_node->get_logger(), "Bridging topics now...");
    while (ros1_node->ok() && rclcpp::ok()) {
        executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
    }

    return 0;
}
