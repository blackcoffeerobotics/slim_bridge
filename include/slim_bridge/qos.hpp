#ifndef __QOS_HPP_
#define __QOS_HPP_

#include <unordered_map>
#include "rmw/types.h"

static const rmw_qos_profile_t reliability_profile =
{
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

static const rmw_qos_profile_t best_effort_profile =
{
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

std::unordered_map<std::string, rmw_qos_reliability_policy_t>reliability_table = 
{
    {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
    {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
};

std::unordered_map<std::string, rmw_qos_durability_policy_t>durability_table = 
{
    {"transient_local", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
    {"volatile", RMW_QOS_POLICY_DURABILITY_VOLATILE}
};

#endif