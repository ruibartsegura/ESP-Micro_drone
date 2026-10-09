/**
 * Made by Rui B.S.
 * Date: 24/09/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Odometry estimator of the drone. It runs in its own FreeRTOS task
 *   every 10 ms. The estimation itself is not implemented yet.
 *
 * Functions:
 *   - estimate_odom(): estimates the odometry (empty for now).
 *   - odom_task(): FreeRTOS task that calls estimate_odom() periodically.
 *   - odom_estimator_init(): creates the odometry task.
 *   - odom_estimator_test(): checks that the module started correctly.
 */

#include <stdbool.h>

#include "odometry_estimator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG = "estimate_odom_task";

#define ODOM_TASK_PERIOD_MS 10

static bool is_init = false;


void estimate_odom() {

}

void odom_task(void *arg) {
    (void)arg;

    int log_counter = 0;

    while (1) {
        estimate_odom();

        if (++log_counter >= 500) {
            log_counter = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            // ESP_LOGI(TAG, "stack libre (min historico): %u bytes",
                    //  (unsigned)(free_words * sizeof(StackType_t)));
        }

        vTaskDelay(pdMS_TO_TICKS(ODOM_TASK_PERIOD_MS));
    }


}

void odom_estimator_init(void) {
    if (is_init) {
        return;
    }

    xTaskCreate(odom_task, "odom_task", CONFIG_ODOM_TASK_STACK, NULL, CONFIG_ODOM_TASK_PRIO, NULL);


    is_init = true;
}

bool odom_estimator_test(void) {
    if (!is_init) {
        return false;
    }

    return true;
}


// std_msgs/Header header
// 	builtin_interfaces/Time stamp
// 		int32 sec
// 		uint32 nanosec
// 	string frame_id

// # Frame id the pose points to. The twist is in this coordinate frame.
// string child_frame_id

// # Estimated pose that is typically relative to a fixed world frame.
// geometry_msgs/PoseWithCovariance pose
// 	Pose pose
// 		Point position
// 			float64 x
// 			float64 y
// 			float64 z
// 		Quaternion orientation
// 			float64 x 0
// 			float64 y 0
// 			float64 z 0
// 			float64 w 1
// 	float64[36] covariance

// # Estimated linear and angular velocity relative to child_frame_id.
// geometry_msgs/TwistWithCovariance twist
// 	Twist twist
// 		Vector3  linear
// 			float64 x
// 			float64 y
// 			float64 z
// 		Vector3  angular
// 			float64 x
// 			float64 y
// 			float64 z
// 	float64[36] covariance