/* Reset every mock (call it from setUp()). */
#pragma once
void mock_reset_all(void);
void mock_reset_esp(void);
void mock_reset_ros(void);
/* Number of ESP_ERROR_CHECK() failures and the last error. */
extern int mock_esp_error_check_fails;
extern int mock_esp_error_check_last;
