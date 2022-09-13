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

#ifndef INCLUDE_SLIM_BRIDGE_QOS_HPP_
#define INCLUDE_SLIM_BRIDGE_QOS_HPP_

#include <string>
#include <unordered_map>
#include "rmw/types.h"

static const rmw_qos_profile_t reliability_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    300,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

static const rmw_qos_profile_t best_effort_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    100,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto reliable_qos = rclcpp::QoS(
    rclcpp::QoSInitialization::from_rmw(reliability_profile),
    reliability_profile
);

auto best_effort_qos = rclcpp::QoS(
    rclcpp::QoSInitialization::from_rmw(best_effort_profile),
    best_effort_profile
);

std::unordered_map<std::string, rmw_qos_reliability_policy_t>reliability_table
     = {{"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
        {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
};

std::unordered_map<std::string, rmw_qos_durability_policy_t>durability_table
     = {{"transient_local", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
        {"volatile", RMW_QOS_POLICY_DURABILITY_VOLATILE}
};

#endif  // INCLUDE_SLIM_BRIDGE_QOS_HPP_
