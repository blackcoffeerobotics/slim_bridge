#ifndef __BRIDGE_HPP__
#define __BRIDGE_HPP__

#include "factories.hpp"
#include <vector>
#include "qos.hpp"
#include <sstream>
#include "yaml-cpp/yaml.h"
#include <cstdlib>

namespace slim_bridge{

    typedef
        struct BridgeOptions{
            std::string topic_name;
            rclcpp::QoS qos=reliable_qos;
            size_t queue_size;
            bool latch=false;
        }
    BridgeOptions;

    typedef
        struct BridgeHandle{
            std::shared_ptr<ros::NodeHandle> ros1_node;
            rclcpp::Node::SharedPtr ros2_node;
            std::vector<std::shared_ptr<ros::Publisher>> ros1_publishers;
            std::vector<std::shared_ptr<ros::Subscriber>> ros1_subscribers;
            std::vector<rclcpp::PublisherBase::SharedPtr> ros2_publishers;
            std::vector<rclcpp::SubscriptionBase::SharedPtr> ros2_subscribers;
            std::vector<std::shared_ptr<FactoryInterface>> factories;
        }
    BridgeHandle;

    void create_bridge_2_to_1(
        const std::string package_name,
        const std::string message_type,
        const BridgeOptions options,
        const std::shared_ptr<BridgeHandle> & bridge
    ){
        auto factory = get_factory(package_name, message_type);
        
        if(factory){
            auto publisher = factory->create_ros1_publisher(
                bridge->ros1_node,
                options.topic_name,
                options.queue_size,
                options.latch
            );
            bridge->ros1_publishers.push_back(publisher);            
            auto subscriber = factory->create_ros2_subscriber(
                bridge->ros2_node,
                options.topic_name,
                options.qos,
                publisher
            );
            bridge->ros2_subscribers.push_back(subscriber);
            bridge->factories.push_back(factory);
        }

        else{
            throw std::runtime_error(
                "No Template Specialization for "+
                package_name + "/" + message_type
            );
        }
    }

    void create_bridge_1_to_2(
        const std::string package_name,
        const std::string message_type,
        const BridgeOptions options,
        const std::shared_ptr<BridgeHandle> & bridge
    ){
        auto factory = get_factory(package_name, message_type);

        if(factory){
            auto publisher = factory->create_ros2_publisher(
                bridge->ros2_node,
                options.topic_name,
                options.qos
            );
            bridge->ros2_publishers.push_back(publisher);
            auto subscriber = factory->create_ros1_subscriber(
                bridge->ros1_node,
                options.topic_name,
                options.queue_size,
                publisher
            );
            bridge->ros1_subscribers.push_back(subscriber);
            bridge->factories.push_back(factory);
        }

        else{
            throw std::runtime_error(
                "No Template Specialization for "+
                package_name + "/" + message_type
            );
        }
    }

    std::vector<std::string> tokenize(
        std::string s,
        char del='/'
    ){
        std::stringstream ss(s);
        std::string segment;
        std::vector<std::string> seglist;

        while(std::getline(ss, segment, del))
        {
            seglist.push_back(segment);
        }

        if(seglist.size() != 2){
            throw std::runtime_error(
                "Incorrect Message type provided " + s
            );
        }
        return seglist;
    }

    void read_yaml(const YAML::Node & data,
        std::string & topic_name,
        BridgeOptions & options,
        std::string & package_name,
        std::string & message_type
    ){
        options.topic_name = topic_name;
        
        if (data["latch"]){options.latch = data["latch"].as<bool>();}
        else {options.latch=true;}

        options.qos = reliable_qos;
        options.qos.reliability(reliability_table[data["reliability"].as<std::string>()]);
        options.qos.durability(durability_table[data["durability"].as<std::string>()]);
        if(data["history"].as<std::string>() == "keep_last"){
            options.qos.keep_last(data["depth"].as<int>());
        }
        else if(data["history"].as<std::string>() == "keep_all"){
            options.qos.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        }
        options.queue_size = data["queue_size"].as<int>();

        try {
            auto names = tokenize(data["type"].as<std::string>());
            package_name = names[0];
            message_type = names[1];
        }
        catch (const std::exception & e)
        {
            std::cout << e.what() << "\n";
        }
    }

    void read_string(const std::string & data,
        BridgeOptions & options,
        std::string & package_name,
        std::string & message_type
    ){
        auto yaml_obj = YAML::Load(data);
        std::string topic_name = yaml_obj["topic_name"].as<std::string>();
        read_yaml(yaml_obj,
            topic_name,
            options,
            package_name,
            message_type
        );
    }

}

#endif