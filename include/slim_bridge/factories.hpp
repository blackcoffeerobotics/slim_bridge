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

#ifndef INCLUDE_SLIM_BRIDGE_FACTORIES_HPP_
#define INCLUDE_SLIM_BRIDGE_FACTORIES_HPP_

#include <iostream>
#include <string>
#include <memory>

#include "slim_bridge/factory.hpp"

namespace slim_bridge {
    /*
    * Design pattern: get_factory returns a ptr to conversion from
    * 'package_name/message_type' msg. The mapping is done by standard if else
    * to keep things simple. But due to the possibility of how huge the function
    * can get, the functionality/implementation is outsourced to a static
    * function. This is simply for neatness.
    */

    static void
    get_factory_impl(const std::string package_name,
        const std::string message_type,
        std::shared_ptr<FactoryInterface> & factory
    ) {
        if (package_name == "std_msgs") {
            if (message_type == "Bool") {
                factory = std::make_shared<
                    Factory<
                        std_msgs::Bool,
                        std_msgs::msg::Bool
                    >
                >(package_name, message_type);
            } else if (message_type == "Int32") {
                factory = std::make_shared<
                    Factory<
                        std_msgs::Int32,
                        std_msgs::msg::Int32
                    >
                >(package_name, message_type);
            } else if (message_type == "Float32") {
                factory =  std::make_shared<
                    Factory<
                        std_msgs::Float32,
                        std_msgs::msg::Float32
                    >
                >(package_name, message_type);
            } else if (message_type == "Header") {
                factory =  std::make_shared<
                    Factory<
                        std_msgs::Header,
                        std_msgs::msg::Header
                    >
                >(package_name, message_type);
            } else if (message_type == "String") {
                factory =  std::make_shared<
                    Factory<
                        std_msgs::String,
                        std_msgs::msg::String
                    >
                >(package_name, message_type);
            }
        } else if (package_name == "geometry_msgs") {
            if (message_type == "Point") {
                factory =  std::make_shared<
                    Factory<
                        geometry_msgs::Point,
                        geometry_msgs::msg::Point
                    >
                >(package_name, message_type);
            } else if (message_type == "Vector3") {
                factory =  std::make_shared<
                    Factory<
                        geometry_msgs::Vector3,
                        geometry_msgs::msg::Vector3
                    >
                >(package_name, message_type);
            } else if (message_type == "Quaternion") {
                factory =  std::make_shared<
                    Factory<
                        geometry_msgs::Quaternion,
                        geometry_msgs::msg::Quaternion
                    >
                >(package_name, message_type);
            } else if (message_type == "Pose") {
                factory =  std::make_shared<
                    Factory<
                        geometry_msgs::Pose,
                        geometry_msgs::msg::Pose
                    >
                >(package_name, message_type);
            } else if (message_type == "PoseStamped") {
                factory =  std::make_shared<
                    Factory<
                        geometry_msgs::PoseStamped,
                        geometry_msgs::msg::PoseStamped
                    >
                >(package_name, message_type);
            } else if (message_type == "Pose2D") {
                factory =  std::make_shared<
                    Factory<
                        geometry_msgs::Pose2D,
                        geometry_msgs::msg::Pose2D
                    >
                >(package_name, message_type);
            } else if (message_type == "Transform") {
                factory = std::make_shared<
                    Factory<
                        geometry_msgs::Transform,
                        geometry_msgs::msg::Transform
                    >
                >(package_name, message_type);
            }  else if (message_type == "TransformStamped") {
                factory = std::make_shared<
                    Factory<
                        geometry_msgs::TransformStamped,
                        geometry_msgs::msg::TransformStamped
                    >
                >(package_name, message_type);
            }
        } else if (package_name == "nav_msgs") {
            if (message_type == "Path") {
                factory =  std::make_shared<
                    Factory<
                        nav_msgs::Path,
                        nav_msgs::msg::Path
                    >
                >(package_name, message_type);
            }
        } else if (package_name == "sensor_msgs") {
            if (message_type == "PointField") {
                factory = std::make_shared<
                    Factory<
                        sensor_msgs::PointField,
                        sensor_msgs::msg::PointField
                    >
                >(package_name, message_type);
            } else if (message_type == "PointCloud2") {
                factory = std::make_shared<
                    Factory<
                        sensor_msgs::PointCloud2,
                        sensor_msgs::msg::PointCloud2
                    >
                >(package_name, message_type);
            } else if (message_type == "Image") {
                factory = std::make_shared<
                    Factory<
                        sensor_msgs::Image,
                        sensor_msgs::msg::Image
                    >
                >(package_name, message_type);
            } else if (message_type == "CompressedImage") {
                factory = std::make_shared<
                    Factory<
                        sensor_msgs::CompressedImage,
                        sensor_msgs::msg::CompressedImage
                    >
                >(package_name, message_type);
            } else if (message_type == "LaserScan") {
                factory = std::make_shared<
                    Factory<
                        sensor_msgs::LaserScan,
                        sensor_msgs::msg::LaserScan
                    >
                >(package_name, message_type);
            } else if (message_type == "CameraInfo") {
                factory = std::make_shared<
                    Factory<
                        sensor_msgs::CameraInfo,
                        sensor_msgs::msg::CameraInfo
                    >
                >(package_name, message_type);
            } else if (message_type == "NavSatStatus") {
                factory = std::make_shared<
                    Factory<
                        sensor_msgs::NavSatStatus,
                        sensor_msgs::msg::NavSatStatus
                    >
                >(package_name, message_type);
            } else if (message_type == "NavSatFix") {
                factory = std::make_shared<
                    Factory<
                        sensor_msgs::NavSatFix,
                        sensor_msgs::msg::NavSatFix
                    >
                >(package_name, message_type);
            }
        } else if (package_name == "tf2_msgs") {
            if (message_type == "TFMessage") {
                factory = std::make_shared<
                    Factory<
                        tf2_msgs::TFMessage,
                        tf2_msgs::msg::TFMessage
                    >
                >(package_name, message_type);
            }
        }
    }

    std::shared_ptr<FactoryInterface>
    get_factory(const std::string & package_name,
        const std::string & message_type
    ) {
        std::shared_ptr<FactoryInterface> factory;
        get_factory_impl(package_name, message_type, factory);
        return factory;
    }
}  // namespace slim_bridge

#endif  //  INCLUDE_SLIM_BRIDGE_FACTORIES_HPP_
