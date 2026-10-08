/* Mock of rmw/qos_profiles.h (host tests). */
#pragma once
#include <stddef.h>
typedef enum { RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
               RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT } rmw_qos_reliability_policy_t;
typedef enum { RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT = 0, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
               RMW_QOS_POLICY_DURABILITY_VOLATILE } rmw_qos_durability_policy_t;
typedef enum { RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0, RMW_QOS_POLICY_HISTORY_KEEP_LAST,
               RMW_QOS_POLICY_HISTORY_KEEP_ALL } rmw_qos_history_policy_t;
typedef struct {
    rmw_qos_history_policy_t     history;
    size_t                       depth;
    rmw_qos_reliability_policy_t reliability;
    rmw_qos_durability_policy_t  durability;
} rmw_qos_profile_t;
/* Same values as the real profiles. */
static const rmw_qos_profile_t rmw_qos_profile_default = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10, RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE };
static const rmw_qos_profile_t rmw_qos_profile_sensor_data = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE };
