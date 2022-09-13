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

#ifndef INCLUDE_SLIM_BRIDGE_CONVERT_HPP_
#define INCLUDE_SLIM_BRIDGE_CONVERT_HPP_

// ROS2 Message Inclusions
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"

// ROS1 Message Inclusions
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace slim_bridge {

    template<typename ROS1_T, typename ROS2_T>
    void convert_ros1_to_2(const ROS1_T & ros1_msg, ROS2_T & ros2_msg);

    template<>
    void convert_ros1_to_2(
        const std_msgs::Bool & ros1_msg,
        std_msgs::msg::Bool & ros2_msg) {
        ros2_msg.data = ros1_msg.data;
    }

    template<>
    void convert_ros1_to_2(
        const std_msgs::Int32 & ros1_msg, 
        std_msgs::msg::Int32 & ros2_msg
    ) {
        ros2_msg.data = ros1_msg.data;
    }

    template<>
    void convert_ros1_to_2(
        const std_msgs::Float32 & ros1_msg, 
        std_msgs::msg::Float32 & ros2_msg
    ) {
        ros2_msg.data = ros1_msg.data;
    }

    template<>
    void convert_ros1_to_2(
        const std_msgs::Header & ros1_msg, 
        std_msgs::msg::Header & ros2_msg
    ) {
        ros2_msg.frame_id = ros1_msg.frame_id;
        ros2_msg.stamp.sec = ros1_msg.stamp.sec;
        ros2_msg.stamp.nanosec = ros1_msg.stamp.nsec;
    }

    template<>
    void convert_ros1_to_2(
        const std_msgs::String & ros1_msg, 
        std_msgs::msg::String & ros2_msg
    ) {
        ros2_msg.data = ros1_msg.data;
    }

    template<>
    void convert_ros1_to_2(
        const geometry_msgs::Point & ros1_msg,
        geometry_msgs::msg::Point & ros2_msg
    ) {
        ros2_msg.x = ros1_msg.x;
        ros2_msg.y = ros1_msg.y;
        ros2_msg.z = ros1_msg.z;
    }

    template<>
    void convert_ros1_to_2(
        const geometry_msgs::Vector3 & ros1_msg,
        geometry_msgs::msg::Vector3 & ros2_msg
    ) {
        ros2_msg.x = ros1_msg.x;
        ros2_msg.y = ros1_msg.y;
        ros2_msg.z = ros1_msg.z;
    }

    template<>
    void convert_ros1_to_2(
        const geometry_msgs::Quaternion & ros1_msg,
        geometry_msgs::msg::Quaternion & ros2_msg
    ) {
        ros2_msg.x = ros1_msg.x;
        ros2_msg.y = ros1_msg.y;
        ros2_msg.z = ros1_msg.z;
        ros2_msg.w = ros1_msg.w;
    }

    template<>
    void convert_ros1_to_2(
        const geometry_msgs::Pose & ros1_msg, 
        geometry_msgs::msg::Pose & ros2_msg
    ) {
        convert_ros1_to_2(ros1_msg.position, ros2_msg.position);
        convert_ros1_to_2(ros1_msg.orientation, ros2_msg.orientation);
    }

    template<>
    void convert_ros1_to_2(
        const geometry_msgs::PoseStamped & ros1_msg,
        geometry_msgs::msg::PoseStamped & ros2_msg
    ) {
        convert_ros1_to_2(ros1_msg.header, ros2_msg.header);
        convert_ros1_to_2(ros1_msg.pose, ros2_msg.pose);
    }

    template<>
    void convert_ros1_to_2(
        const geometry_msgs::Pose2D & ros1_msg,
        geometry_msgs::msg::Pose2D & ros2_msg
    ) {
        ros2_msg.x = ros1_msg.x;
        ros2_msg.y = ros1_msg.y;
        ros2_msg.theta = ros1_msg.theta;
    }

    template<>
    void convert_ros1_to_2(
        const nav_msgs::Path & ros1_msg,
        nav_msgs::msg::Path & ros2_msg
    ) {
        convert_ros1_to_2(ros1_msg.header, ros2_msg.header);
        std::transform(ros1_msg.poses.begin(), ros1_msg.poses.end(),
            std::back_inserter(ros2_msg.poses),
            [](geometry_msgs::PoseStamped pose) {
                geometry_msgs::msg::PoseStamped returnable;
                convert_ros1_to_2(pose.header, returnable.header);
                convert_ros1_to_2(pose.pose, returnable.pose);
                return returnable;
            });
    }

    template<>
    void convert_ros1_to_2(
        const sensor_msgs::PointField & ros1_msg,
        sensor_msgs::msg::PointField & ros2_msg
    ) {
        ros2_msg.count = ros1_msg.count;
        ros2_msg.datatype = ros1_msg.datatype;
        ros2_msg.name = ros1_msg.name;
        ros2_msg.offset = ros1_msg.offset;
    }

    template<>
    void convert_ros1_to_2(
        const sensor_msgs::PointCloud2 & ros1_msg,
        sensor_msgs::msg::PointCloud2 & ros2_msg
    ) {
        ros2_msg.data = ros1_msg.data;
        std::transform(ros1_msg.fields.begin(), ros1_msg.fields.end(),
            std::back_inserter(ros2_msg.fields),
            [](sensor_msgs::PointField field) {
                sensor_msgs::msg::PointField returnable;
                convert_ros1_to_2(field, returnable);
                return returnable;
            });
        convert_ros1_to_2(ros1_msg.header, ros2_msg.header);
        ros2_msg.height = ros1_msg.height;
        ros2_msg.is_bigendian = ros1_msg.is_bigendian;
        ros2_msg.is_dense = ros1_msg.is_dense;
        ros2_msg.point_step = ros1_msg.point_step;
        ros2_msg.row_step = ros1_msg.row_step;
        ros2_msg.width = ros1_msg.width;
    }

    template<>
    void convert_ros1_to_2(
        const sensor_msgs::Image & ros1_msg,
        sensor_msgs::msg::Image & ros2_msg
    ) {
        ros2_msg.data = ros1_msg.data;
        ros2_msg.encoding = ros1_msg.encoding;
        convert_ros1_to_2(ros1_msg.header, ros2_msg.header);
        ros2_msg.height = ros1_msg.height;
        ros2_msg.is_bigendian = ros1_msg.is_bigendian;
        ros2_msg.step = ros1_msg.step;
        ros2_msg.width = ros1_msg.width;
    }

    template<>
    void convert_ros1_to_2(
        const sensor_msgs::CompressedImage & ros1_msg,
        sensor_msgs::msg::CompressedImage & ros2_msg
    ) {
        ros2_msg.data = ros1_msg.data;
        convert_ros1_to_2(ros1_msg.header, ros2_msg.header);
        ros2_msg.format = ros1_msg.format;
    }

    template<>
    void convert_ros1_to_2(
        const sensor_msgs::LaserScan & ros1_msg,
        sensor_msgs::msg::LaserScan & ros2_msg
    ) {
        convert_ros1_to_2(ros1_msg.header, ros2_msg.header);
        ros2_msg.angle_increment = ros1_msg.angle_increment;
        ros2_msg.angle_max = ros1_msg.angle_max;
        ros2_msg.angle_min = ros1_msg.angle_min;
        ros2_msg.intensities = ros1_msg.intensities;
        ros2_msg.range_max = ros1_msg.range_max;
        ros2_msg.range_min = ros1_msg.range_min;
        ros2_msg.ranges = ros1_msg.ranges;
        ros2_msg.scan_time = ros1_msg.scan_time;
        ros2_msg.time_increment = ros1_msg.time_increment;
    }

    template<>
    void convert_ros1_to_2(
        const sensor_msgs::CameraInfo & ros1_msg,
        sensor_msgs::msg::CameraInfo & ros2_msg
    ) {
        convert_ros1_to_2(ros1_msg.header, ros2_msg.header);
        ros2_msg.binning_x = ros1_msg.binning_x;
        ros2_msg.binning_y = ros1_msg.binning_y;
        ros2_msg.distortion_model = ros1_msg.distortion_model;
        ros2_msg.height = ros1_msg.height;
        ros2_msg.width = ros1_msg.width;
    }

    template<>
    void convert_ros1_to_2(
        const geometry_msgs::Transform & ros1_msg,
        geometry_msgs::msg::Transform & ros2_msg
    ) {
        convert_ros1_to_2(ros1_msg.translation, ros2_msg.translation);
        convert_ros1_to_2(ros1_msg.rotation, ros2_msg.rotation);
    }

    template<>
    void convert_ros1_to_2(
        const geometry_msgs::TransformStamped & ros1_msg,
        geometry_msgs::msg::TransformStamped & ros2_msg
    ) {
        convert_ros1_to_2(ros1_msg.header, ros2_msg.header);
        convert_ros1_to_2(ros1_msg.transform, ros2_msg.transform);
        ros2_msg.child_frame_id = ros1_msg.child_frame_id;
    }

    template<>
    void convert_ros1_to_2(
        const tf2_msgs::TFMessage & ros1_msg,
        tf2_msgs::msg::TFMessage & ros2_msg
    ) {
        std::transform(ros1_msg.transforms.begin(), ros1_msg.transforms.end(),
            std::back_inserter(ros2_msg.transforms),
            [](geometry_msgs::TransformStamped transform) {
                geometry_msgs::msg::TransformStamped returnable;
                convert_ros1_to_2(transform, returnable);
                return returnable;
            });
    }

    template<typename ROS1_T, typename ROS2_T>
    void convert_ros2_to_1(const ROS2_T & ros2_msg, ROS1_T & ros1_msg);

    template<>
    void convert_ros2_to_1(
        const std_msgs::msg::Bool & ros2_msg,
        std_msgs::Bool & ros1_msg
    ) {
        ros1_msg.data = ros2_msg.data;
    }

    template<>
    void convert_ros2_to_1(
        const std_msgs::msg::Int32 & ros2_msg,
        std_msgs::Int32 & ros1_msg
    ) {
        ros1_msg.data = ros2_msg.data;
    }

    template<>
    void convert_ros2_to_1(
        const std_msgs::msg::Float32 & ros2_msg,
        std_msgs::Float32 & ros1_msg
    ) {
        ros1_msg.data = ros2_msg.data;
    }

    template<>
    void convert_ros2_to_1(
        const std_msgs::msg::Header & ros2_msg,
        std_msgs::Header & ros1_msg
    ) {
        ros1_msg.frame_id = ros2_msg.frame_id;
        ros1_msg.stamp.nsec = ros2_msg.stamp.nanosec;
        ros1_msg.stamp.sec = ros2_msg.stamp.sec;
    }

    template<>
    void convert_ros2_to_1(
        const std_msgs::msg::String & ros2_msg, 
        std_msgs::String & ros1_msg
    ) {
        ros1_msg.data = ros2_msg.data;
    }

    template<>
    void convert_ros2_to_1(
        const geometry_msgs::msg::Point & ros2_msg,
        geometry_msgs::Point & ros1_msg
    ) {
        ros1_msg.x = ros2_msg.x;
        ros1_msg.y = ros2_msg.y;
        ros1_msg.z = ros2_msg.z;
    }

    template<>
    void convert_ros2_to_1(
        const geometry_msgs::msg::Vector3 & ros2_msg,
        geometry_msgs::Vector3 & ros1_msg
    ) {
        ros1_msg.x = ros2_msg.x;
        ros1_msg.y = ros2_msg.y;
        ros1_msg.z = ros2_msg.z;
    }

    template<>
    void convert_ros2_to_1(
        const geometry_msgs::msg::Quaternion & ros2_msg,
        geometry_msgs::Quaternion & ros1_msg
    ) {
        ros1_msg.x = ros2_msg.x;
        ros1_msg.y = ros2_msg.y;
        ros1_msg.z = ros2_msg.z;
        ros1_msg.w = ros2_msg.w;
    }

    template<>
    void convert_ros2_to_1(
        const geometry_msgs::msg::Pose & ros2_msg, 
        geometry_msgs::Pose & ros1_msg
    ) {
        convert_ros2_to_1(ros2_msg.position, ros1_msg.position);
        convert_ros2_to_1(ros2_msg.orientation, ros1_msg.orientation);
    }

    template<>
    void convert_ros2_to_1(
        const geometry_msgs::msg::PoseStamped & ros2_msg,
        geometry_msgs::PoseStamped & ros1_msg
    ) {
        convert_ros2_to_1(ros2_msg.header, ros1_msg.header);
        convert_ros2_to_1(ros2_msg.pose, ros1_msg.pose);
    }

    template<>
    void convert_ros2_to_1(
        const geometry_msgs::msg::Pose2D & ros2_msg,
        geometry_msgs::Pose2D & ros1_msg
    ) {
        ros1_msg.x = ros2_msg.x;
        ros1_msg.y = ros2_msg.y;
        ros1_msg.theta = ros2_msg.theta;
    }

    template<>
    void convert_ros2_to_1(
        const nav_msgs::msg::Path & ros2_msg,
        nav_msgs::Path & ros1_msg
    ) {
        convert_ros2_to_1(ros2_msg.header, ros1_msg.header);
        std::transform(ros2_msg.poses.begin(), ros2_msg.poses.end(),
            std::back_inserter(ros1_msg.poses),
            [](geometry_msgs::msg::PoseStamped pose) {
                geometry_msgs::PoseStamped returnable;
                convert_ros2_to_1(pose.header, returnable.header);
                convert_ros2_to_1(pose.pose, returnable.pose);
                return returnable;
            });
    }

    template<>
    void convert_ros2_to_1(
        const sensor_msgs::msg::PointField & ros2_msg,
        sensor_msgs::PointField & ros1_msg
    ) {
        ros1_msg.count = ros2_msg.count;
        ros1_msg.datatype = ros2_msg.datatype;
        ros1_msg.name = ros2_msg.name;
        ros1_msg.offset = ros2_msg.offset;
    }

    template<>
    void convert_ros2_to_1(
        const sensor_msgs::msg::PointCloud2 & ros2_msg,
        sensor_msgs::PointCloud2 & ros1_msg
    ) {
        ros1_msg.data = ros2_msg.data;
        std::transform(ros2_msg.fields.begin(), ros2_msg.fields.end(),
            std::back_inserter(ros1_msg.fields),
            [](sensor_msgs::msg::PointField field) {
                sensor_msgs::PointField returnable;
                convert_ros2_to_1(field, returnable);
                return returnable;
            });
        convert_ros2_to_1(ros2_msg.header, ros1_msg.header);
        ros1_msg.height = ros2_msg.height;
        ros1_msg.is_bigendian = ros2_msg.is_bigendian;
        ros1_msg.is_dense = ros2_msg.is_dense;
        ros1_msg.point_step = ros2_msg.point_step;
        ros1_msg.row_step = ros2_msg.row_step;
        ros1_msg.width = ros2_msg.width;
    }

    template<>
    void convert_ros2_to_1(
        const sensor_msgs::msg::Image & ros2_msg,
        sensor_msgs::Image & ros1_msg
    ) {
        ros1_msg.data = ros2_msg.data;
        ros1_msg.encoding = ros2_msg.encoding;
        convert_ros2_to_1(ros2_msg.header, ros1_msg.header);
        ros1_msg.height = ros2_msg.height;
        ros1_msg.is_bigendian = ros2_msg.is_bigendian;
        ros1_msg.step = ros2_msg.step;
        ros1_msg.width = ros2_msg.width;
    }

    template<>
    void convert_ros2_to_1(
        const sensor_msgs::msg::CompressedImage & ros2_msg,
        sensor_msgs::CompressedImage & ros1_msg
    ) {
        ros1_msg.data = ros2_msg.data;
        convert_ros2_to_1(ros2_msg.header, ros1_msg.header);
        ros1_msg.format = ros2_msg.format;
    }

    template<>
    void convert_ros2_to_1(
        const sensor_msgs::msg::LaserScan & ros2_msg,
        sensor_msgs::LaserScan & ros1_msg
    ) {
        convert_ros2_to_1(ros2_msg.header, ros1_msg.header);
        ros1_msg.angle_increment = ros2_msg.angle_increment;
        ros1_msg.angle_max = ros2_msg.angle_max;
        ros1_msg.angle_min = ros2_msg.angle_min;
        ros1_msg.intensities = ros2_msg.intensities;
        ros1_msg.range_max = ros2_msg.range_max;
        ros1_msg.range_min = ros2_msg.range_min;
        ros1_msg.ranges = ros2_msg.ranges;
        ros1_msg.scan_time = ros2_msg.scan_time;
        ros1_msg.time_increment = ros2_msg.time_increment;
    }

    template<>
    void convert_ros2_to_1(
        const sensor_msgs::msg::CameraInfo & ros2_msg,
        sensor_msgs::CameraInfo & ros1_msg
    ) {
        convert_ros2_to_1(ros2_msg.header, ros1_msg.header);
        ros1_msg.binning_x = ros2_msg.binning_x;
        ros1_msg.binning_y = ros2_msg.binning_y;
        ros1_msg.distortion_model = ros2_msg.distortion_model;
        ros1_msg.height = ros2_msg.height;
        ros1_msg.width = ros2_msg.width;
    }

    template<>
    void convert_ros2_to_1(
        const geometry_msgs::msg::Transform & ros2_msg,
        geometry_msgs::Transform & ros1_msg
    ) {
        convert_ros2_to_1(ros2_msg.rotation, ros1_msg.rotation);
        convert_ros2_to_1(ros2_msg.translation, ros1_msg.translation);
    }

    template<>
    void convert_ros2_to_1(
        const geometry_msgs::msg::TransformStamped & ros2_msg,
        geometry_msgs::TransformStamped & ros1_msg
    ) {
        convert_ros2_to_1(ros2_msg.transform, ros1_msg.transform);
        convert_ros2_to_1(ros2_msg.header, ros1_msg.header);
        ros1_msg.child_frame_id = ros2_msg.child_frame_id;
    }

    template<>
    void convert_ros2_to_1(
        const tf2_msgs::msg::TFMessage & ros2_msg,
        tf2_msgs::TFMessage & ros1_msg
    ) {
        std::transform(ros2_msg.transforms.begin(), ros2_msg.transforms.end(),
            std::back_inserter(ros1_msg.transforms),
            [](geometry_msgs::msg::TransformStamped transform) {
                geometry_msgs::TransformStamped returnable;
                convert_ros2_to_1(transform, returnable);
                return returnable;
            });
    }

}  // namespace slim_bridge

#endif  //  INCLUDE_SLIM_BRIDGE_CONVERT_HPP_
