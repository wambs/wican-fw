/*
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include "driver/twai.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "lwip/sockets.h"
#include "elm327.h"
#include "autopid.h"
#include "expression_parser.h"
#include "mqtt.h"
#include "cJSON.h"
#include "config_server.h"
#include "autopid.h"
#include <math.h>
#include "obd2_standard_pids.h"
#include "wc_timer.h"
#include <float.h>
#include "obd_logger.h"
#include "hw_config.h"
#include "dev_status.h"
#include "sleep_mode.h"
#include "https_client_mgr.h"
#include "cert_manager.h"
#include <time.h>
#include "ha_webhooks.h"
#include "autopid_config.h"
#include "autopid_pause.h"
#include "esp_heap_caps.h"
#include "autopid.h"
#include "imu.h"
#include "restart_tracker.h"

// #define TAG __func__
#define TAG "AUTO_PID"

#define TEMP_BUFFER_LENGTH 32
#define ECU_CONNECTED_BIT BIT0
#define AUTOPID_POLL_JITTER_MS 100

// Backoff tuning (milliseconds)
// How many consecutive failures before enabling backoff.
#ifndef AUTOPID_BACKOFF_RETRY_THRESHOLD
#define AUTOPID_BACKOFF_RETRY_THRESHOLD 15
#endif
// Minimum backoff delay once enabled (per destination type).
#ifndef AUTOPID_BACKOFF_MIN_CAP_HTTP_MS
#define AUTOPID_BACKOFF_MIN_CAP_HTTP_MS 10000
#endif
#ifndef AUTOPID_BACKOFF_MIN_CAP_ABRP_MS
#define AUTOPID_BACKOFF_MIN_CAP_ABRP_MS 10000
#endif
// Maximum backoff relative to the configured cycle (e.g. 2 = 2x cycle).
#ifndef AUTOPID_BACKOFF_MAX_MULTIPLIER
#define AUTOPID_BACKOFF_MAX_MULTIPLIER 2
#endif




/* [NEW] Task handle for the asynchronous processing engine */
static TaskHandle_t autopid_processing_task_handle = NULL;
static StackType_t *autopid_processing_task_stack = NULL;
static StaticTask_t autopid_processing_task_buffer;

static char *auto_pid_buf;
static QueueHandle_t autopidQueue;
static char *device_id;
static EventGroupHandle_t xautopid_event_group = NULL;
static StaticEventGroup_t xautopid_event_group_buffer;
static autopid_config_t *autopid_config = NULL;
static autopid_data_t autopid_data = {.json_str = NULL, .mutex = NULL};
static QueueHandle_t protocolnumberQueue = NULL;
static volatile bool supply_mode_active = false;
// Cached configuration JSON (built once after autopid_config is loaded)
static char *autopid_config_json = NULL;
static StaticTimer_t autopid_bit_set_timer_buffer;
static TimerHandle_t autopid_bit_set_timer_handle = NULL;

static const char* current_group_mqtt_topic = NULL;

#if HARDWARE_VER == WICAN_PRO
static char *elm327_autopid_cmd_buffer;
static uint32_t elm327_autopid_cmd_buffer_len = 0;
static int64_t elm327_autopid_last_cmd_time = 0;
#endif

static const uint8_t autopid_protocol_header_length[] = {
    0, // 0: Automatic
    3, // 1: SAE J1850 PWM (41.6 kbaud)
    3, // 2: SAE J1850 VPW (10.4 kbaud)
    3, // 3: ISO 9141-2 (5 baud init, 10.4 kbaud)
    3, // 4: ISO 14230-4 KWP (5 baud init, 10.4 kbaud)
    3, // 5: ISO 14230-4 KWP (fast init, 10.4 kbaud)
    0, // 6: ISO 15765-4 CAN (11 bit ID, 500 kbaud)
    9, // 7: ISO 15765-4 CAN (29 bit ID, 500 kbaud)
    0, // 8: ISO 15765-4 CAN (11 bit ID, 250 kbaud)
    9, // 9: ISO 15765-4 CAN (29 bit ID, 250 kbaud)
    9, // A: SAE J1939 CAN (29 bit ID, 250 kbaud)
    0, // B: USER1 CAN (11 bit ID, 125 kbaud)
    0  // C: USER2 CAN (29 bit ID, 50 kbaud)
};

// Forward Declarations
static void send_commands(char *commands, uint32_t delay_ms);
static void publish_parameter_mqtt(parameter_t *param);
static void autopid_data_update(autopid_config_t *pids);
static void autopid_processing_task(void *pvParameters);

static const char *autopid_imu_activity_state_str(void)
{
    switch (imu_get_activity_state())
    {
    case ACTIVITY_STATE_ACTIVE:
        return "active";
    case ACTIVITY_STATE_STATIONARY:
        return "stationary";
    case ACTIVITY_STATE_INVALID:
    default:
        return "invalid";
    }
}

static char *autopid_build_debug_payload(void)
{
    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        return NULL;
    }

    float imu_x = 0.0f;
    float imu_y = 0.0f;
    float imu_z = 0.0f;
    float battery_voltage = 0.0f;
    uint8_t imu_status2 = 0;
    char trigger[32] = {0};
    const char *activity = autopid_imu_activity_state_str();

    if (strcmp(activity, "active") == 0)
    {
        imu_status2 = imu_get_last_int_status2();
    }

    cJSON_AddStringToObject(root, "imu_activity", activity);
    cJSON_AddBoolToObject(root, "imu_active", strcmp(activity, "active") == 0);
    cJSON_AddNumberToObject(root, "imu_int_status2", imu_status2);
    cJSON_AddBoolToObject(root, "imu_trigger_smd", (imu_status2 & ICM42670_SMD_INT_BITS) != 0);
    cJSON_AddBoolToObject(root, "imu_trigger_x", (imu_status2 & ICM42670_WOM_X_INT_BITS) != 0);
    cJSON_AddBoolToObject(root, "imu_trigger_y", (imu_status2 & ICM42670_WOM_Y_INT_BITS) != 0);
    cJSON_AddBoolToObject(root, "imu_trigger_z", (imu_status2 & ICM42670_WOM_Z_INT_BITS) != 0);
    cJSON_AddNumberToObject(root, "imu_trigger_x_count", imu_get_wom_x_count());
    cJSON_AddNumberToObject(root, "imu_trigger_y_count", imu_get_wom_y_count());
    cJSON_AddNumberToObject(root, "imu_trigger_z_count", imu_get_wom_z_count());
    cJSON_AddNumberToObject(root, "imu_last_active_ms", imu_get_last_active_ms());

    if (imu_status2 & ICM42670_SMD_INT_BITS)
        strlcat(trigger, "smd", sizeof(trigger));
    if (imu_status2 & ICM42670_WOM_X_INT_BITS)
        strlcat(trigger, trigger[0] ? ",x" : "x", sizeof(trigger));
    if (imu_status2 & ICM42670_WOM_Y_INT_BITS)
        strlcat(trigger, trigger[0] ? ",y" : "y", sizeof(trigger));
    if (imu_status2 & ICM42670_WOM_Z_INT_BITS)
        strlcat(trigger, trigger[0] ? ",z" : "z", sizeof(trigger));
    cJSON_AddStringToObject(root, "imu_trigger", trigger[0] ? trigger : "none");

    if (imu_read_accel(&imu_x, &imu_y, &imu_z) == ESP_OK)
    {
        cJSON_AddNumberToObject(root, "imu_accel_x", imu_x);
        cJSON_AddNumberToObject(root, "imu_accel_y", imu_y);
        cJSON_AddNumberToObject(root, "imu_accel_z", imu_z);
    }

    if (sleep_mode_get_voltage(&battery_voltage) == ESP_OK)
    {
        cJSON_AddNumberToObject(root, "battery_voltage", battery_voltage);
    }

    autopid_pause_pid_polling_state_t polling_state = autopid_pause_get_pid_polling_state();
    cJSON_AddBoolToObject(root, "pid_polling_paused", polling_state.paused);
    cJSON_AddStringToObject(root, "pause_reason",
                            polling_state.reason ? polling_state.reason : "none");

    restart_tracker_state_t restart_state = {0};
    if (restart_tracker_get_state(&restart_state) == ESP_OK)
    {
        cJSON_AddNumberToObject(root, "restart_unexpected_reset_count",
                                restart_state.unexpected_reset_count);
    }
    else
    {
        cJSON_AddNumberToObject(root, "restart_unexpected_reset_count", 0);
    }

    if (config_server_get_mqtt_include_timestamp() == 1)
    {
        cJSON_AddNumberToObject(root, "timestamp", (double)time(NULL));
    }

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return payload;
}

// Wall-clock time of the last autopid_data snapshot refresh (0 = never).
static time_t autopid_data_updated_at = 0;

static volatile int32_t s_last_car_on_valid_stamp = -1;

void autopid_pid_update_hook(parameter_t *param, float value) {
    if (!param || !param->name) return;
    const char *gate_param = config_server_get_car_on_param();
    if (!gate_param || strlen(gate_param) == 0) return;

    if (strstr(param->name, gate_param) != NULL || strstr(gate_param, param->name) != NULL) {
        
        const char *op = config_server_get_car_on_operator();
        const char *val_str = config_server_get_car_on_value();
        if (!op || !val_str) return;

        float threshold = atof(val_str);
        bool condition_met = false;

        // Safely catch standard operators OR their HTML-encoded equivalents
        if ((strstr(op, ">") || strstr(op, "&gt;")) && value > threshold) condition_met = true;
        else if ((strstr(op, "<") || strstr(op, "&lt;")) && value < threshold) condition_met = true;
        else if ((strstr(op, "=") || strstr(op, "&eq;")) && value == threshold) condition_met = true;

        if (condition_met) {
            s_last_car_on_valid_stamp = (int32_t)(esp_timer_get_time() / 1000000ULL);
        }
    }
}

bool autopid_car_is_on(void) {
    int32_t t = s_last_car_on_valid_stamp;
    return t >= 0 && ((int32_t)(esp_timer_get_time() / 1000000ULL) - t) < 65;
}



// --- HELPER: Check for RPM to detect Engine Running (NEW) ---
static bool is_engine_running(void) {
    char *rpm_json = autopid_get_value_by_name("RPM");
    if (!rpm_json) rpm_json = autopid_get_value_by_name("Engine RPM"); 

    bool running = false;
    if (rpm_json) {
        cJSON *root = cJSON_Parse(rpm_json);
        if (root) {
            cJSON *child = root->child;
            while(child) {
                if (cJSON_IsNumber(child)) {
                    // Threshold: > 300 RPM usually means engine is running (cranking or idle)
                    if (child->valuedouble > 300.0) running = true;
                }
                child = child->next;
            }
            cJSON_Delete(root);
        }
        free(rpm_json);
    }
    return running;
}

// buffer fix - replaced function below, old use static for multiple thread calls
// Helper functions
//  Custom printer function to format numbers with 2 decimal places
char *formatNumberPrecision(double num) {
    static char buf[32]; // <--- THE ASSASSIN
    snprintf(buf, sizeof(buf), "%.2f", num);
    // ...
    return buf;
}

  
static double autopid_round_parameter_value(double value)
{
    return round(value * 100.0) / 100.0;
}

static cJSON *autopid_add_parameter_number_to_object(cJSON *object,
                                                     const char *name,
                                                     float value)
{
    return cJSON_AddNumberToObject(object, name, autopid_round_parameter_value((double)value));
}

static bool autopid_prepare_parameter_value(parameter_t *param,
                                            double raw_value,
                                            float *out_value,
                                            const char *source)
{
    double rounded_value;

    if (!param || !out_value)
        return false;

    if (!isfinite(raw_value))
    {
        ESP_LOGW(TAG, "%s parameter %s produced non-finite value - ignoring",
                 source ? source : "AUTO_PID",
                 param->name ? param->name : "(null)");
        return false;
    }

    rounded_value = autopid_round_parameter_value(raw_value);

    if (param->min != FLT_MAX && rounded_value < param->min)
    {
        ESP_LOGW(TAG, "%s parameter %s value %.2f below min %.2f - ignoring",
                 source ? source : "AUTO_PID",
                 param->name ? param->name : "(null)",
                 rounded_value,
                 param->min);
        return false;
    }

    if (param->max != FLT_MAX && rounded_value > param->max)
    {
        ESP_LOGW(TAG, "%s parameter %s value %.2f above max %.2f - ignoring",
                 source ? source : "AUTO_PID",
                 param->name ? param->name : "(null)",
                 rounded_value,
                 param->max);
        return false;
    }

    *out_value = (float)rounded_value;
    return true;
}

static bool autopid_supply_mode_compare_value(double actual, const char *op, double expected)
{
    if (!op)
        return false;

    if (strcmp(op, "=") == 0 || strcmp(op, "==") == 0)
        return fabs(actual - expected) < 0.00001;
    if (strcmp(op, "<") == 0)
        return actual < expected;
    if (strcmp(op, ">") == 0)
        return actual > expected;
    if (strcmp(op, ">=") == 0)
        return actual >= expected;
    if (strcmp(op, "<=") == 0)
        return actual <= expected;
    if (strcmp(op, "!=") == 0)
        return fabs(actual - expected) >= 0.00001;

    return false;
}

static void autopid_update_supply_mode_state_from_param(const parameter_t *param)
{
    if (!param ||
        !param->name ||
        !autopid_config ||
        !autopid_config->supply_mode_enabled ||
        !autopid_config->supply_mode_pid_name ||
        autopid_config->supply_mode_pid_name[0] == '\0' ||
        autopid_config->supply_mode_operator[0] == '\0' ||
        strcmp(param->name, autopid_config->supply_mode_pid_name) != 0 ||
        !isfinite((double)param->value))
    {
        return;
    }

    supply_mode_active = autopid_supply_mode_compare_value((double)param->value,
                                                           autopid_config->supply_mode_operator,
                                                           (double)autopid_config->supply_mode_value);
}

bool autopid_supply_mode_is_active(void)
{
    if (!autopid_config ||
        !autopid_config->supply_mode_enabled ||
        !autopid_config->supply_mode_pid_name ||
        autopid_config->supply_mode_pid_name[0] == '\0' ||
        autopid_config->supply_mode_operator[0] == '\0')
    {
        return false;
    }

    return supply_mode_active;
}

// strdup_psram
static char *strdup_psram(const char *s)
{
    if (!s)
        return NULL;

    size_t len = strlen(s) + 1;
    char *copy = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!copy)
        return NULL;

    memcpy(copy, s, len);
    return copy;
}

// buffer fix replaced function below
// Recursively limit decimal precision in the JSON structure
void limitJsonDecimalPrecision(cJSON *item)
{
    if (!item)
        return;

    // If current item is a number, modify its value
    if (cJSON_IsNumber(item))
    {
        // --- FIXED: Use the native thread-safe math rounder! ---
        item->valuedouble = autopid_round_parameter_value(item->valuedouble);
        item->valuestring = NULL; // Force cJSON to use valuedouble
    }

    // Process all children
    cJSON *child = item->child;
    while (child)
    {
        limitJsonDecimalPrecision(child);
        child = child->next;
    }
}

esp_err_t autopid_set_protocol_number(int32_t protocol_value)
{
    if (protocolnumberQueue == NULL)
    {
        ESP_LOGE(TAG, "Protocol queue not initialized");
        return ESP_FAIL;
    }

    // Clear any existing value in the queue
    xQueueReset(protocolnumberQueue);

    // Add the new protocol value
    if (xQueueSend(protocolnumberQueue, &protocol_value, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to set protocol number: %ld", protocol_value);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Protocol number set to: %ld", protocol_value);
    return ESP_OK;
}

esp_err_t autopid_get_protocol_number(int32_t *protocol_value)
{
    if (protocolnumberQueue == NULL)
    {
        ESP_LOGE(TAG, "Protocol queue not initialized");
        return ESP_FAIL;
    }

    if (protocol_value == NULL)
    {
        ESP_LOGE(TAG, "Invalid protocol_value pointer");
        return ESP_FAIL;
    }

    // Peek at the value without removing it from the queue
    if (xQueuePeek(protocolnumberQueue, protocol_value, 0) != pdPASS)
    {
        ESP_LOGW(TAG, "No protocol number available in queue");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Retrieved protocol number: %ld", *protocol_value);
    return ESP_OK;
}

bool autopid_lock(uint32_t timeout_ms)
{
    if (!autopid_config || !autopid_config->mutex)
        return false;

    TickType_t to = pdMS_TO_TICKS(timeout_ms);
    return (xSemaphoreTake(autopid_config->mutex, to) == pdTRUE);
}

void autopid_unlock(void)
{
    if (!autopid_config || !autopid_config->mutex)
        return;

    xSemaphoreGive(autopid_config->mutex);
}

const std_pid_t *get_pid_from_string(const char *pid_string)
{
    char pid_hex[3];
    uint8_t pid_value;

    // Extract first 2 characters (hex PID)
    strncpy(pid_hex, pid_string, 2);
    pid_hex[2] = '\0';

    // Convert hex string to integer
    pid_value = (uint8_t)strtol(pid_hex, NULL, 16);

    // Get the base PID info
    const std_pid_t *pid_info = get_pid(pid_value);
    if (!pid_info)
    {
        return NULL;
    }

    // If there's a parameter name specified after the dash
    if (strchr(pid_string, '-'))
    {
        const char *param_name = strchr(pid_string, '-') + 1;

        // For all PIDs, verify the parameter exists
        bool param_found = false;
        for (int i = 0; i < pid_info->num_params; i++)
        {
            if (strcmp(pid_info->params[i].name, param_name) == 0)
            {
                param_found = true;
                break;
            }
        }
        if (!param_found)
        {
            return NULL;
        }
    }

    return pid_info;
}

static esp_err_t extract_signal_value(const uint8_t *data,
                                      uint8_t data_length,
                                      const std_parameter_t *param,
                                      float *result)
{
    // Validate input parameters
    if (!data || !param || !result)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Calculate which bytes we need
    uint8_t start_byte = param->bit_start / 8;
    uint8_t bytes_needed = (param->bit_length + 7) / 8;

    // Validate data length
    if (start_byte + bytes_needed > data_length)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    // Extract raw value (Motorola format)
    uint32_t raw_value = 0;
    for (uint8_t i = 0; i < bytes_needed; i++)
    {
        raw_value = (raw_value << 8) | data[start_byte + i];
    }

    // Apply bit mask for the signal length
    uint32_t mask = (1ULL << param->bit_length) - 1;
    raw_value &= mask;

    // Calculate and limit physical value
    float physical_value = (float)raw_value * param->scale + param->offset;

    if (physical_value < param->min)
    {
        physical_value = param->min;
    }
    if (physical_value > param->max)
    {
        physical_value = param->max;
    }

    *result = physical_value;
    return ESP_OK;
}

static void merge_response_frames(uint8_t *data, uint32_t length, uint8_t *merged_frame)
{
    // Initialize merged frame with first 7 bytes
    for (int i = 0; i < 7; i++)
    {
        merged_frame[i] = data[i];
    }

    // Process subsequent frames
    for (int frame = 7; frame < length; frame += 7)
    {
        // Perform bitwise OR for each byte position
        for (int byte = 0; byte < 7 && (frame + byte) < length; byte++)
        {
            merged_frame[byte] |= data[frame + byte];
        }
    }
}

esp_err_t autopid_find_standard_pid(uint8_t protocol, char *available_pids, uint32_t available_pids_size)
{
    if (autopid_config == NULL || autopid_config->mutex == NULL)
    {
        ESP_LOGE(TAG, "autopid_config not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    response_t *response = NULL;
    uint32_t supported_pids = 0;
    uint8_t selected_protocol = 0;
    static const char *supported_protocols[] = {
        "ATTP0\r",               // Protocol 0
        "ATTP6\rATSH7DF\rATCRA\r",      // Protocol 6
        "ATTP7\rATSH18DB33F1\rATCRA\r", // Protocol 7
        "ATTP8\rATSH7DF\rATCRA\r",      // Protocol 8
        "ATTP9\rATSH18DB33F1\rATCRA\r", // Protocol 9
    };

    if (protocol >= 6 && protocol <= 9)
    {
        selected_protocol = protocol - 5; // supported_protocols index, 6-5 = 1, 7-5 = 2, 8-5 = 3, 9-5 = 4
    }
    else
    {
        selected_protocol = 0;
    }

    response = (response_t *)heap_caps_malloc(sizeof(response_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    memset(response, 0, sizeof(response_t));

    if (response == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for response");

        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(autopid_config->mutex, portMAX_DELAY);

    cJSON *root = cJSON_CreateObject();
    cJSON *pid_array = cJSON_CreateArray();

    if ((protocol >= 6 && protocol <= 9) || protocol == 0)
    {
        ESP_LOGI(TAG, "Setting protocol %d", protocol);

        static const char *elm327_config = "atws\ratm0\rate0\rath1\ratl0\rats1\ratst96\r";
        // elm327_process_cmd((uint8_t*)elm327_config, strlen(elm327_config), &frame, &autopidQueue);
        // while (xQueueReceive(autopidQueue, response, pdMS_TO_TICKS(1000)) == pdPASS);
        elm327_process_cmd((uint8_t *)elm327_config, strlen(elm327_config), &autopidQueue, elm327_autopid_cmd_buffer, &elm327_autopid_cmd_buffer_len, &elm327_autopid_last_cmd_time, NULL);
        const char *protocol_cmds = supported_protocols[selected_protocol];
        ESP_LOGI(TAG, "Sending protocol commands: %s", protocol_cmds);
        // elm327_process_cmd((uint8_t*)protocol_cmds, strlen(protocol_cmds), &frame, &autopidQueue);
        // while (xQueueReceive(autopidQueue, response, pdMS_TO_TICKS(1000)) == pdPASS);
        elm327_process_cmd((uint8_t *)protocol_cmds, strlen(protocol_cmds), &autopidQueue, elm327_autopid_cmd_buffer, &elm327_autopid_cmd_buffer_len, &elm327_autopid_last_cmd_time, NULL);
        ESP_LOGI(TAG, "Protocol %d set successfully", selected_protocol);
    }
    else
    {
        ESP_LOGE(TAG, "Invalid protocol number: %d", selected_protocol);
        // elm327_unlock();
        free(response);
        xSemaphoreGive(autopid_config->mutex);
        return ESP_FAIL;
    }

    xQueueReceive(autopidQueue, response, pdMS_TO_TICKS(100));

    const char *pid_support_cmds[] = {
        "0100\r", // PIDs 0x01-0x20
        "0120\r", // PIDs 0x21-0x40
        "0140\r", // PIDs 0x41-0x60
        "0160\r", // PIDs 0x61-0x80
        "0180\r", // PIDs 0x81-0xA0
        "01A0\r", // PIDs 0xA1-0xC0
    };

   while (xQueueReceive(autopidQueue, response, pdMS_TO_TICKS(100)) == pdPASS) {
        if (response->priority_data != NULL) {
            free(response->priority_data);
            response->priority_data = NULL;
        }
    }
    
    ESP_LOGI(TAG, "Starting PID support command processing");
    for (int i = 0; i < sizeof(pid_support_cmds) / sizeof(pid_support_cmds[0]); i++)
    {
        ESP_LOGI(TAG, "Processing PID support command: %s", pid_support_cmds[i]);

        if (elm327_process_cmd((uint8_t *)pid_support_cmds[i], strlen(pid_support_cmds[i]), &autopidQueue, elm327_autopid_cmd_buffer, &elm327_autopid_cmd_buffer_len, &elm327_autopid_last_cmd_time, autopid_parser) != 0)
        {
            ESP_LOGW(TAG, "Failed to process PID support command: %s", pid_support_cmds[i]);
            continue;
        }

        if (xQueueReceive(autopidQueue, response, pdMS_TO_TICKS(10000)) == pdPASS)
        {
            ESP_LOGI(TAG, "Raw response length: %lu", response->length);
            ESP_LOG_BUFFER_HEX(TAG, response->data, response->length);

            // Skip mode byte (0x41) and PID byte
            if ((strstr((char *)response->data, "error") == NULL) && response->length >= 6)
            {
                // Determine frame size by response length.
                // CAN responses: 7 bytes per frame [PCI, mode, PID, D1, D2, D3, D4].
                // ISO/KWP responses: 6 bytes per frame [PID, D1, D2, D3, D4, checksum].
                bool is_can_frame = (response->length >= 7 && response->length % 7 == 0);

                if (is_can_frame)
                {
                    // CAN format: merge 7-byte frames via OR, bitmap at offset 3
                    uint8_t merged_frame[7] = {0};
                    merge_response_frames(response->data, response->length, merged_frame);

                    supported_pids = (merged_frame[3] << 24) |
                                     (merged_frame[4] << 16) |
                                     (merged_frame[5] << 8) |
                                     merged_frame[6];
                }
                else
                {
                    // ISO/KWP format: merge 6-byte frames via OR, bitmap at offset 1
                    uint8_t merged[6] = {0};
                    for (uint32_t f = 0; f < response->length; f += 6)
                    {
                        for (uint8_t b = 0; b < 6 && (f + b) < response->length; b++)
                        {
                            merged[b] |= response->data[f + b];
                        }
                    }
                    supported_pids = (merged[1] << 24) |
                                     (merged[2] << 16) |
                                     (merged[3] << 8) |
                                     merged[4];
                }

                ESP_LOGI(TAG, "Merged frame bitmap: 0x%08lx", supported_pids);

                for (int bit = 0; bit < 32; bit++)
                {
                    if (supported_pids & (1 << (31 - bit)))
                    {
                        uint8_t pid = (i * 32) + bit + 1;
                        const std_pid_t *pid_info = get_pid(pid);

                        if (pid_info)
                        {
                            char pid_str[64];

                            // If the PID has multiple parameters
                            if (pid_info->num_params > 1 && pid_info->params)
                            {
                                ESP_LOGI(TAG, "Processing multi-parameter PID: %02X", pid);
                                // Add each parameter as a separate entry
                                for (int p = 0; p < pid_info->num_params; p++)
                                {
                                    if (pid_info->params[p].name)
                                    {
                                        snprintf(pid_str, sizeof(pid_str), "%02X-%s",
                                                 pid, pid_info->params[p].name);
                                        ESP_LOGI(TAG, "PID %02X parameter %d supported: %s",
                                                 pid, p + 1, pid_str);
                                        cJSON_AddItemToArray(pid_array, cJSON_CreateString(pid_str));
                                    }
                                    else
                                    {
                                        ESP_LOGW(TAG, "PID %02X parameter %d has NULL name", pid, p + 1);
                                    }
                                }
                            }
                            else if (pid_info->params && pid_info->params[0].name)
                            {
                                // Single parameter PID
                                snprintf(pid_str, sizeof(pid_str), "%02X-%s",
                                         pid, pid_info->params[0].name);
                                ESP_LOGI(TAG, "PID %02X supported: %s", pid, pid_str);
                                cJSON_AddItemToArray(pid_array, cJSON_CreateString(pid_str));
                            }
                            else
                            {
                                ESP_LOGW(TAG, "PID %02X has invalid or NULL parameters", pid);
                            }
                        }
                    }
                }
            }
            else
            {
                ESP_LOGW(TAG, "Response length too short: %lu", response->length);
            }
        }
        else
        {
            if (response->length >= 7)
            {
                ESP_LOGW(TAG, "No response received for PID support command: %s", pid_support_cmds[i]);
            }
            else
            {
                ESP_LOGW(TAG, "Recvied error response: %s", response->data);
            }
        }
	// buffer fix--- NEW FIX: PREVENT LOOP OVERWRITE LEAK ---
        if (response->priority_data != NULL) {
            free(response->priority_data);
            response->priority_data = NULL;
        }
        // buffer fix --------------------------------------------
    }

    ESP_LOGI(TAG, "Adding PIDs to JSON object");    
    cJSON_AddItemToObject(root, "std_pids", pid_array);

    // Convert to string and cleanup
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str)
    {
        ESP_LOGI(TAG, "JSON string created, length: %zu", strlen(json_str));
        if (strlen(json_str) < available_pids_size)
        {
            strcpy(available_pids, json_str);
            free(json_str);
            cJSON_Delete(root);

            // ESP_LOGI(TAG, "Restoring protocol settings");
            // elm327_process_cmd((uint8_t*)restore_cmd, strlen(restore_cmd), &frame, &autopidQueue);
            // while (xQueueReceive(autopidQueue, response, pdMS_TO_TICKS(1000)) == pdPASS);

            free(response);
            xSemaphoreGive(autopid_config->mutex);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "JSON string too long for buffer");
        free(json_str);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to create JSON string");
    }

    ESP_LOGI(TAG, "Restoring protocol settings");
    // elm327_process_cmd((uint8_t*)restore_cmd, strlen(restore_cmd), &frame, &autopidQueue);
    // while (xQueueReceive(autopidQueue, &response, pdMS_TO_TICKS(1000)) == pdPASS);

    cJSON_Delete(root);

    // --- NEW: CLEANUP BEFORE RETURN ---
    if (response->priority_data != NULL) {
        free(response->priority_data);
    }
    
    free(response);
    // elm327_unlock();
    xSemaphoreGive(autopid_config->mutex);
    return ESP_FAIL;
}

static void autopid_data_update(autopid_config_t *pids)
{
    if (!pids || !pids->mutex)
    {
        ESP_LOGE(TAG, "Invalid autopid_config or mutex");
        return;
    }

    // take autopid_data mutex
    if (xSemaphoreTake(autopid_data.mutex, portMAX_DELAY) == pdTRUE)
    {
        // free old data
        if (autopid_data.json_str != NULL)
        {
            free(autopid_data.json_str);
            autopid_data.json_str = NULL;
        }

        cJSON *root = cJSON_CreateObject();
        if (root)
        {

            // 1. PIDs (Group-Aware Logic)
            if (pids->use_groups) {
                for (uint32_t g = 0; g < pids->group_count; g++) {

		  pid_group_t *grp = &pids->groups[g];
                    if (!grp->enabled) continue;
                    for (uint32_t i = 0; i < grp->pid_count; i++) {
                        pid_data_t *curr_pid = grp->pids[i];
                        if (!curr_pid || !curr_pid->enabled) continue;
			
                        for (uint32_t j = 0; j < curr_pid->parameters_count; j++) {
                            parameter_t *param = &curr_pid->parameters[j];
                            if (!param->enabled) continue;

			    if (param->failed) continue;

                            // ---> SKIP NON-DEFAULT DESTINATIONS <---
                            if (param->destination_type != DEST_DEFAULT) continue;

                            if (param->name) {
                                if (param->raw_string_value != NULL) {
                                    cJSON_AddStringToObject(root, param->name, param->raw_string_value);
                                } else if (param->value != FLT_MAX) {
                                    if (param->sensor_type == BINARY_SENSOR) {
                                        cJSON_AddStringToObject(root, param->name, param->value > 0 ? "on" : "off");
                                    } else {
                                        cJSON_AddNumberToObject(root, param->name, param->value);
                                    }
                                }
                            }
                        }
                    }

               }

	    } 
            
            if (pids->pid_count > 0) {
                // Legacy Flat List Logic
                for (uint32_t i = 0; i < pids->pid_count; i++) {
                    pid_data_t *curr_pid = &pids->pids[i];

                    bool is_in_group = false;
                    if (pids->use_groups) {
                        for (uint32_t g = 0; g < pids->group_count; g++) {
                            for (uint32_t gp = 0; gp < pids->groups[g].pid_count; gp++) {
                                if (pids->groups[g].pids[gp] == curr_pid) {
                                    is_in_group = true; break;
                                }
                            }
                            if (is_in_group) break;
                        }
                    }
                    if (is_in_group) continue; // Let the Group logic handle this one!
                    // -------------------------------

                    if (!curr_pid->enabled) continue;
                    for (uint32_t j = 0; j < curr_pid->parameters_count; j++) {
                        parameter_t *param = &curr_pid->parameters[j];
                        if (!param->enabled) continue;

			// ---> ADD THIS: STRIP FAILED PIDS <---
                        if (param->failed) continue;

                        // ---> SKIP NON-DEFAULT DESTINATIONS <---
                        if (param->destination_type != DEST_DEFAULT) continue;

                        if (param->name) {
                            if (param->raw_string_value != NULL) {
                                cJSON_AddStringToObject(root, param->name, param->raw_string_value);
                            } else if (param->value != FLT_MAX) {
                                if (param->sensor_type == BINARY_SENSOR) {
                                    cJSON_AddStringToObject(root, param->name, param->value > 0 ? "on" : "off");
                                } else {
                                    cJSON_AddNumberToObject(root, param->name, param->value);
                                }
                            }
                        }		       
                    }
                }
            }

            // 2. Add CAN-filter (broadcast) parameters
            for (uint32_t fi = 0; fi < pids->can_filters_count; fi++)
            {
                can_filter_t *f = &pids->can_filters[fi];
                for (uint32_t pi = 0; pi < f->parameters_count; pi++)
                {
                    parameter_t *param = &f->parameters[pi];
                    if (!param->enabled) continue;

                    // ---> SKIP NON-DEFAULT DESTINATIONS <---
                    if (param->destination_type != DEST_DEFAULT) continue;

                    if (param->name) {
                        if (param->raw_string_value != NULL) {
                            cJSON_AddStringToObject(root, param->name, param->raw_string_value);
                        } else if (param->value != FLT_MAX) {
                            if (param->sensor_type == BINARY_SENSOR) {
                                cJSON_AddStringToObject(root, param->name, param->value > 0 ? "on" : "off");
                            } else {
                                cJSON_AddNumberToObject(root, param->name, param->value);
                            }
                        }
                    }
                }
            }
            if (root->child)
            {
                autopid_data_updated_at = time(NULL);
            }
            autopid_data.json_str = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);
        }
	  

        // release mutex
        xSemaphoreGive(autopid_data.mutex);
    }
}

char *autopid_data_read(void)
{
    // json_str must be freed by the caller
    char *json_str = NULL;

    if (!autopid_data.mutex)
    {
        ESP_LOGE(TAG, "Invalid mutex, autopid_data not initialized");
        return NULL;
    }

    if (xSemaphoreTake(autopid_data.mutex, portMAX_DELAY) == pdTRUE)
    {
        if (autopid_data.json_str != NULL)
        {
           json_str = (char *)heap_caps_malloc(strlen(autopid_data.json_str) + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (json_str != NULL)
            {
                strcpy(json_str, autopid_data.json_str);
            }
        }
        else
        {
            json_str = NULL;
        }
        xSemaphoreGive(autopid_data.mutex);
    }
    return json_str;
}

char *autopid_get_value_by_name(char *name)
{
    // json_str must be freed by the caller
    char *result_json = NULL;

    if (autopid_data.mutex == NULL)
    {
        ESP_LOGE(TAG, "Invalid mutex, autopid_data not initialized");
        return NULL;
    }
    else if (name == NULL || strlen(name) == 0)
    {
        ESP_LOGE(TAG, "Invalid name");
        return NULL;
    }

    if (xSemaphoreTake(autopid_data.mutex, portMAX_DELAY) == pdTRUE)
    {
        if (autopid_data.json_str != NULL)
        {
            char *json_copy = (char *)heap_caps_malloc(strlen(autopid_data.json_str) + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (json_copy != NULL)
            {
                strcpy(json_copy, autopid_data.json_str);

                cJSON *root = cJSON_Parse(json_copy);
                if (root)
                {
                    cJSON *item = cJSON_GetObjectItem(root, name);
                    if (item)
                    {
                        cJSON *result = cJSON_CreateObject();
                        cJSON *value_copy = cJSON_Duplicate(item, 1);
                        cJSON_AddItemToObject(result, name, value_copy);
                        result_json = cJSON_PrintUnformatted(result);
                        cJSON_Delete(result);
                    }
                    cJSON_Delete(root);
                }
                heap_caps_free(json_copy);
            }
        }
        xSemaphoreGive(autopid_data.mutex);
    }
    return result_json;
}

void autopid_data_publish(void)
{
    if (!autopid_config || !autopid_config->mutex)
    {
        ESP_LOGE(TAG, "Invalid autopid_config or mutex");
        return;
    }

    if (xSemaphoreTake(autopid_config->mutex, portMAX_DELAY) == pdTRUE)
    {
        cJSON *root = cJSON_CreateObject();
        if (root)
        {
            // 1. PIDs (Group-Aware Logic)
            if (autopid_config->use_groups) {
                for (uint32_t g = 0; g < autopid_config->group_count; g++) {
                    pid_group_t *grp = &autopid_config->groups[g];
                    if (!grp->enabled) continue;
                    for (uint32_t i = 0; i < grp->pid_count; i++) {
                        pid_data_t *curr_pid = grp->pids[i];
                        if (!curr_pid || !curr_pid->enabled) continue;
                        for (uint32_t j = 0; j < curr_pid->parameters_count; j++) {
                            parameter_t *param = &curr_pid->parameters[j];
                            if (!param->enabled) continue;

                            // ---> SKIP NON-DEFAULT DESTINATIONS <---
                            if (param->destination_type != DEST_DEFAULT) continue;

                            if (param->name) {
                                if (param->raw_string_value != NULL) {
                                    cJSON_AddStringToObject(root, param->name, param->raw_string_value);
                                } else if (param->value != FLT_MAX) {
                                    if (param->sensor_type == BINARY_SENSOR) {
                                        cJSON_AddStringToObject(root, param->name, param->value > 0 ? "on" : "off");
                                    } else {
                                        cJSON_AddNumberToObject(root, param->name, param->value);
                                    }
                                }
                            }
                        }
                    }
               }
            } 

            // Legacy Flat List Logic
            if (autopid_config->pid_count > 0) {
                for (uint32_t i = 0; i < autopid_config->pid_count; i++) {
                    pid_data_t *curr_pid = &autopid_config->pids[i];

                    // ---> ADD THIS FILTER BLOCK <---
                    bool is_in_group = false;
                    if (autopid_config->use_groups) {
                        for (uint32_t g = 0; g < autopid_config->group_count; g++) {
                            for (uint32_t gp = 0; gp < autopid_config->groups[g].pid_count; gp++) {
                                if (autopid_config->groups[g].pids[gp] == curr_pid) {
                                    is_in_group = true; break;
                                }
                            }
                            if (is_in_group) break;
                        }
                    }
                    if (is_in_group) continue; // Let the Group logic handle this one!
                    // -------------------------------
		    
                    if (!curr_pid->enabled) continue;
                    for (uint32_t j = 0; j < curr_pid->parameters_count; j++) {
                        parameter_t *param = &curr_pid->parameters[j];
                        if (!param->enabled) continue;

                        // ---> SKIP NON-DEFAULT DESTINATIONS <---
                        if (param->destination_type != DEST_DEFAULT) continue;

                          if (param->name) {
                            if (param->raw_string_value != NULL) {
                                cJSON_AddStringToObject(root, param->name, param->raw_string_value);
                            } else if (param->value != FLT_MAX) {
                                if (param->sensor_type == BINARY_SENSOR) {
                                    cJSON_AddStringToObject(root, param->name, param->value > 0 ? "on" : "off");
                                } else {
                                    autopid_add_parameter_number_to_object(root, param->name, param->value);
                                }
                            }
                        }
                    }
                }
            }

            // 2. Add CAN-filter (broadcast) parameters
            for (uint32_t fi = 0; fi < autopid_config->can_filters_count; fi++)
            {
                can_filter_t *f = &autopid_config->can_filters[fi];
                for (uint32_t pi = 0; pi < f->parameters_count; pi++)
                {
                    parameter_t *param = &f->parameters[pi];

                    if (!param->enabled) continue;

                    // ---> SKIP NON-DEFAULT DESTINATIONS <---
                    if (param->destination_type != DEST_DEFAULT) continue;

                    if (param->name) {
                        if (param->raw_string_value != NULL) {
                            cJSON_AddStringToObject(root, param->name, param->raw_string_value);
                        } else if (param->value != FLT_MAX) {
                            if (param->sensor_type == BINARY_SENSOR) {
                                cJSON_AddStringToObject(root, param->name, param->value > 0 ? "on" : "off");
                            } else {
                                autopid_add_parameter_number_to_object(root, param->name, param->value);
                            }
                        }
                    }
                }
            }

            if (root->child)
            {
                if (config_server_get_mqtt_include_timestamp() == 1) {
                    cJSON_AddNumberToObject(root, "timestamp", (double)time(NULL));
                }
                char *json_str = cJSON_PrintUnformatted(root);
		
                if (json_str)
                {
                    if (autopid_config->group_destination && strlen(autopid_config->group_destination) > 0)
                    {
                        mqtt_publish(autopid_config->group_destination, json_str, 0, 0, 1);
                        ESP_LOGI(TAG, "Published to %s", autopid_config->group_destination);
                    }
                    else
                    {
                        mqtt_publish(config_server_get_mqtt_rx_topic(), json_str, 0, 0, 1);
                    }
                    free(json_str);
                }
            }
            else
            {
                ESP_LOGW(TAG, "No valid parameters found to publish");
            }

            cJSON_Delete(root);
        }
        xSemaphoreGive(autopid_config->mutex);
    }
}

bool autopid_get_ecu_status(void)
{
    EventBits_t uxBits;
    if (xautopid_event_group != NULL)
    {
        uxBits = xEventGroupGetBits(xautopid_event_group);

        return (uxBits & ECU_CONNECTED_BIT);
    }
    else
        return false;
}

// Map destination_type_t to string for logging
static const char *dest_type_str(destination_type_t t)
{
    switch (t)
    {
    case DEST_MQTT_TOPIC:
        return "MQTT_Topic";
    case DEST_MQTT_WALLBOX:
        return "MQTT_WallBox";
    case DEST_MQTT_DEBUG:
        return "MQTT_Debug";
    case DEST_HTTP:
        return "HTTP";
    case DEST_HTTPS:
        return "HTTPS";
    case DEST_ABRP_API:
        return "ABRP_API";
    default:
        return "Default";
    }
}

// URL encode a string for form data transmission
static char *url_encode_string(const char *input)
{
    if (!input)
        return NULL;

    size_t input_len = strlen(input);
    size_t encoded_len = input_len * 3 + 1; // worst case for URL encoding
    char *encoded = heap_caps_malloc(encoded_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!encoded)
        return NULL;

    size_t j = 0;
    for (size_t k = 0; k < input_len && j < encoded_len - 1; k++)
    {
        char c = input[k];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
            c == '-' || c == '_' || c == '.' || c == '~')
        {
            encoded[j++] = c;
        }
        else
        {
            if (j < encoded_len - 3)
            {
                sprintf(&encoded[j], "%%%02X", (unsigned char)c);
                j += 3;
            }
        }
    }
    encoded[j] = '\0';

    return encoded;
}

// Helper to copy/map a value from src to tlm with type normalization for ABRP
static void abrp_add_mapped(cJSON *src, cJSON *tlm, const char *from_key, const char *to_key)
{
    if (!src || !tlm || !from_key || !to_key)
        return;
    cJSON *it = cJSON_GetObjectItemCaseSensitive(src, from_key);
    if (!it)
        return;
    if (cJSON_IsNumber(it))
    {
        cJSON_AddNumberToObject(tlm, to_key, it->valuedouble);
    }
    else if (cJSON_IsBool(it))
    {
        cJSON_AddNumberToObject(tlm, to_key, cJSON_IsTrue(it) ? 1 : 0);
    }
    else if (cJSON_IsString(it) && it->valuestring)
    {
        const char *s = it->valuestring;
        if (strcasecmp(s, "on") == 0)
        {
            cJSON_AddNumberToObject(tlm, to_key, 1);
        }
        else if (strcasecmp(s, "off") == 0)
        {
            cJSON_AddNumberToObject(tlm, to_key, 0);
        }
        else
        {
            char *endp = NULL;
            double val = strtod(s, &endp);
            if (endp && endp != s)
            {
                cJSON_AddNumberToObject(tlm, to_key, val);
            }
            else
            {
                cJSON_AddStringToObject(tlm, to_key, s);
            }
        }
    }
}

// Build ABRP telemetry data: creates flat telemetry object (not wrapped) for URL encoding
static char *build_abrp_payload(const char *raw_json, const char *car_model)
{
    if (!raw_json)
        return NULL;

    // Parse incoming telemetry snapshot; if parsing fails, return NULL
    cJSON *src = cJSON_Parse(raw_json);
    if (!src)
    {
        ESP_LOGE(TAG, "Failed to parse raw JSON for ABRP telemetry");
        return NULL;
    }

    // Mapping between internal names and ABRP keys
    typedef struct
    {
        const char *from;
        const char *to;
    } param_map_t;
    static const param_map_t param_map[] = {
        {"SOC", "soc"},
        {"HV_W", "power"},
        {"SPEED", "speed"},
        {"CHARGING", "is_charging"},
        {"CHARGING_DC", "is_dcfc"},
        {"PARK_BRAKE", "is_parked"},
        {"HV_CAPACITY_KWH", "capacity"},
        {"HV_CAPACITY_R", "soe"},
        {"SOH", "soh"},
        {"TMP_A", "ext_temp"},
        {"BATT_TEMP", "batt_temp"},
        {"HV_V", "voltage"},
        {"HV_A", "current"},
        {"ODOMETER", "odometer"},
        {"RANGE", "est_battery_range"},
        {"T_CAB", "cabin_temp"},
        {"TYRE_P_FL", "tire_pressure_fl"},
        {"TYRE_P_FR", "tire_pressure_fr"},
        {"TYRE_P_RL", "tire_pressure_rl"},
        {"TYRE_P_RR", "tire_pressure_rr"},
    };

    // Destination tlm object
    cJSON *tlm = cJSON_CreateObject();
    if (!tlm)
    {
        cJSON_Delete(src);
        return NULL;
    }

    // Apply mapping
    for (size_t i = 0; i < sizeof(param_map) / sizeof(param_map[0]); i++)
    {
        abrp_add_mapped(src, tlm, param_map[i].from, param_map[i].to);
    }

    // Pass-through commonly accepted ABRP extras if already present
    const char *passthrough_keys[] = {"lat", "lon", "elevation"};
    for (size_t i = 0; i < sizeof(passthrough_keys) / sizeof(passthrough_keys[0]); i++)
    {
        const char *k = passthrough_keys[i];
        if (!cJSON_GetObjectItemCaseSensitive(tlm, k))
        {
            cJSON *it = cJSON_GetObjectItemCaseSensitive(src, k);
            if (it)
            {
                if (cJSON_IsNumber(it))
                    cJSON_AddNumberToObject(tlm, k, it->valuedouble);
                else if (cJSON_IsString(it) && it->valuestring)
                {
                    char *endp = NULL;
                    double v = strtod(it->valuestring, &endp);
                    if (endp && endp != it->valuestring)
                        cJSON_AddNumberToObject(tlm, k, v);
                    else
                        cJSON_AddStringToObject(tlm, k, it->valuestring);
                }
            }
        }
    }

    // Ensure utc present (seconds since epoch)
    if (!cJSON_GetObjectItemCaseSensitive(tlm, "utc"))
    {
        time_t now = 0;
        time(&now);
        cJSON_AddNumberToObject(tlm, "utc", (double)now);
    }
    // Match ABRP example payloads: include car_model inside tlm when available.
    if (car_model && *car_model && !cJSON_GetObjectItemCaseSensitive(tlm, "car_model"))
    {
        cJSON_AddStringToObject(tlm, "car_model", car_model);
    }

    // Return tlm object directly (not wrapped) for URL encoding as tlm parameter
    char *printed = cJSON_PrintUnformatted(tlm);
    char *buf_psram = NULL;
    if (printed)
    {
        buf_psram = heap_caps_malloc(strlen(printed) + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (buf_psram)
        {
            strcpy(buf_psram, printed);
        }
        free(printed);
    }
    cJSON_Delete(tlm);
    cJSON_Delete(src);
    return buf_psram; // may be NULL if allocation failed
}

/// @brief
/// @param

void autopid_publish_all_destinations(bool is_event_trigger)
{
    if (!autopid_config)
    {
        ESP_LOGE(TAG, "autopid_publish_all_destinations: autopid_config NULL");
        return;
    }
    if (strcmp("enable", autopid_config->grouping) != 0)
    {
        return; // grouping disabled
    }

    // Get the regular AutoPID snapshot. MQTT Debug can still publish if no
    // regular PID snapshot is available.
    char *raw_json = autopid_data_read();
    if (!raw_json)
    {
        ESP_LOGW(TAG, "No regular autopid data to publish");
    }

    // Inject timestamp into snapshot JSON (only if enabled!)
    if (raw_json && config_server_get_mqtt_include_timestamp() == 1)
    {
        cJSON *ts_root = cJSON_Parse(raw_json);
        if (ts_root)
        {
            cJSON_AddNumberToObject(ts_root, "timestamp", (double)time(NULL));
            char *new_json = cJSON_PrintUnformatted(ts_root);
            cJSON_Delete(ts_root);
            
            if (new_json) {
                free(raw_json);
                raw_json = new_json;
            }
        }
    }

    char *debug_json = NULL;

    // --- FIXED: RESTORE BULK PUBLISHING FOR STANDARD MQTT USERS ---
    // Always attempt to publish the default payload if triggered by an event,
    // completely independent of the advanced destinations loop below.
    if (is_event_trigger) {
        bool can_publish = (config_server_mqtt_en_config() == 1) && (mqtt_connected() != 0);
        if (can_publish && raw_json) {
            mqtt_publish(config_server_get_mqtt_rx_topic(), raw_json, 0, 0, 1);
        }
    }

    // If there are no advanced destinations to process, clean up and exit.
    if (autopid_config->destinations_count == 0) {
        if (raw_json) free(raw_json);
        if (debug_json) free(debug_json);
        return;
    }

    for (uint32_t i = 0; i < autopid_config->destinations_count; i++)
    {
        group_destination_t *gd = &autopid_config->destinations[i];
        if (!gd->enabled)
            continue;

        // ---> RESTORED EVENT OR TIMER LOGIC <---
        // Respect the user's UI choice! 0 = Event-Driven, >0 = Timer-Buffered
        uint32_t base_cycle = gd->cycle; 
        uint32_t effective_cycle = base_cycle;
        
        if (gd->backoff_ms && gd->backoff_ms > base_cycle)
        {
            effective_cycle = gd->backoff_ms; // apply backoff delay
        }

        if (effective_cycle == 0) 
        {
            // Event-driven: Only publish if this function was called immediately after a group update
            if (!is_event_trigger) {
                continue; 
            }
        } 
        else 
        {
            // Timer-driven: Ignore event triggers, let the background polling task handle the schedule
            if (is_event_trigger) {
                continue;
            }
            if (gd->publish_timer != 0 && !wc_timer_is_expired(&gd->publish_timer))
            {
                continue; // Not yet time
            }
        }
        // ----------------------------------

        const char *dest = gd->destination ? gd->destination : "";

        if (gd->type != DEST_MQTT_DEBUG && !raw_json)
        {
            gd->fail_count++;
            ESP_LOGW(TAG, "Destination %u skipped: no regular AutoPID payload", (unsigned int)i);
            if (effective_cycle > 0)
            {
                wc_timer_set(&gd->publish_timer, effective_cycle);
            }
            else
            {
                gd->publish_timer = 0;
            }
            continue;
        }

        switch (gd->type)
        {
        case DEST_MQTT_DEBUG:
        {
            bool can_publish = (config_server_mqtt_en_config() == 1) && (mqtt_connected() != 0);
            const char *topic = (dest[0] != '\0') ? dest : config_server_get_mqtt_rx_topic();

            if (!debug_json)
            {
                debug_json = autopid_build_debug_payload();
            }

            if (can_publish && debug_json)
            {
                mqtt_publish(topic, debug_json, 0, 0, 1);
                gd->success_count++;
                ESP_LOGI(TAG, "Published MQTT Debug to %s", topic);
            }
            else
            {
                gd->fail_count++;
                ESP_LOGW(TAG, "MQTT Debug publish skipped (not connected/disabled or payload failed) to %s", topic);
            }
            break;
        }
        case DEST_MQTT_TOPIC:
        case DEST_DEFAULT: // treat as MQTT default topic
        {
            bool can_publish = (config_server_mqtt_en_config() == 1) && (mqtt_connected() != 0);
            const char *topic = (dest[0] != '\0') ? dest : config_server_get_mqtt_rx_topic();
            if (can_publish)
            {
                mqtt_publish(topic, raw_json, 0, 0, 1);
                gd->success_count++;
                ESP_LOGI(TAG, "Published MQTT (%s) to %s", dest_type_str(gd->type), topic);
            }
            else
            {
                gd->fail_count++;
                ESP_LOGW(TAG, "MQTT publish skipped (not connected/disabled) (%s) to %s", dest_type_str(gd->type), topic);
            }
            break;
        }
        case DEST_MQTT_WALLBOX:
        {
            // For now replicate same JSON publish (future: custom format)
            bool can_publish = (config_server_mqtt_en_config() == 1) && (mqtt_connected() != 0);
            const char *topic = (dest[0] != '\0') ? dest : config_server_get_mqtt_rx_topic();
            if (can_publish)
            {
                mqtt_publish(topic, raw_json, 0, 0, 1);
                gd->success_count++;
                ESP_LOGI(TAG, "Published MQTT WallBox to %s", topic);
            }
            else
            {
                gd->fail_count++;
                ESP_LOGW(TAG, "MQTT WallBox publish skipped (not connected/disabled) to %s", topic);
            }
            break;
        }
        case DEST_HTTP:
        case DEST_HTTPS:
        {
            // Build URL for standard HTTP/HTTPS endpoints
            char *url = dest[0] ? strdup_psram(dest) : NULL;

            if (!url)
            {
                ESP_LOGW(TAG, "Destination %u missing URL", i);
                gd->fail_count++;
                break;
            }

            char *body = NULL;
            char *current_autopid_data = NULL;
            char *current_config_data = NULL;
            char *current_status_data = NULL;

            if (gd->settings_sent == true)
            {
                // Wrap telemetry under "autopid_data" for standard HTTP/HTTPS after initial settings
                cJSON *root_obj = cJSON_CreateObject();
                if (root_obj)
                {
                    cJSON *auto_obj = NULL;
                    if (raw_json)
                    {
                        auto_obj = cJSON_Parse(raw_json);
                    }
                    if (!auto_obj)
                        auto_obj = cJSON_CreateObject();
                    cJSON_AddItemToObject(root_obj, "autopid_data", auto_obj);

                    char *printed = cJSON_PrintUnformatted(root_obj);
                    if (printed)
                    {
                        body = strdup_psram(printed);
                        free(printed);
                    }
                    cJSON_Delete(root_obj);
                }
            }
            else
            {
                // Use settings JSON body for standard HTTP/HTTPS
                current_status_data = config_server_get_status_json(true);
                current_config_data = autopid_get_config();
                current_autopid_data = strdup_psram(raw_json);

                cJSON *root_obj = cJSON_CreateObject();
                if (root_obj)
                {
                    cJSON *cfg_obj = NULL;
                    cJSON *sts_obj = NULL;
                    cJSON *auto_obj = NULL;

                    if (current_config_data)
                    {
                        cfg_obj = cJSON_Parse(current_config_data);
                    }
                    if (!cfg_obj)
                        cfg_obj = cJSON_CreateObject();

                    if (current_status_data)
                    {
                        sts_obj = cJSON_Parse(current_status_data);
                    }
                    if (!sts_obj)
                        sts_obj = cJSON_CreateObject();

                    if (current_autopid_data)
                    {
                        auto_obj = cJSON_Parse(current_autopid_data);
                    }
                    if (!auto_obj)
                        auto_obj = cJSON_CreateObject();

                    cJSON_AddItemToObject(root_obj, "config", cfg_obj);
                    cJSON_AddItemToObject(root_obj, "status", sts_obj);
                    cJSON_AddItemToObject(root_obj, "autopid_data", auto_obj);

                    char *printed = cJSON_PrintUnformatted(root_obj);
                    if (printed)
                    {
                        body = strdup_psram(printed);
                        free(printed);
                    }
                    cJSON_Delete(root_obj);
                }

                if (current_autopid_data)
                {
                    free(current_autopid_data);
                    current_autopid_data = NULL;
                }
                if (current_status_data)
                {
                    free(current_status_data);
                    current_status_data = NULL;
                }
            }

            if (!body)
            {
                ESP_LOGE(TAG, "Failed to allocate body");
                free(url);
                gd->fail_count++;
                break;
            }

            https_client_mgr_config_t cfg = {0};
            cfg.url = url;
            // Startup network + DNS + TLS can be slow; use a slightly larger timeout.
            cfg.timeout_ms = 5000;

            // Detect scheme from URL
            bool is_https_url = (strncasecmp(url, "https://", 8) == 0);
            if (is_https_url)
            {
                // HTTPS: use cert set if provided and not default, otherwise use built-in bundle
                if (gd->cert_set && strcmp(gd->cert_set, "default") != 0)
                {
                    size_t ca_len = 0, cli_len = 0, key_len = 0;
                    const char *ca = cert_manager_get_set_ca_ptr(gd->cert_set, &ca_len);
                    const char *cli = cert_manager_get_set_client_cert_ptr(gd->cert_set, &cli_len);
                    const char *key = cert_manager_get_set_client_key_ptr(gd->cert_set, &key_len);
                    if (ca)
                    {
                        cfg.cert_pem = ca;
                        cfg.cert_len = ca_len;
                        ESP_LOGI(TAG, "Using CA cert from set '%s' for HTTPS", gd->cert_set);
                    }
                    else
                    {
                        cfg.use_crt_bundle = true;
                        ESP_LOGW(TAG, "No CA cert in set '%s', using built-in bundle for HTTPS", gd->cert_set);
                    }
                    if (cli && key)
                    {
                        cfg.client_cert_pem = cli;
                        cfg.client_cert_len = cli_len;
                        cfg.client_key_pem = key;
                        cfg.client_key_len = key_len;
                        ESP_LOGI(TAG, "Using client cert+key from set '%s' for HTTPS", gd->cert_set);
                    }
                }
                else
                {
                    cfg.use_crt_bundle = true; // use built-in bundle for default/no cert set
                    ESP_LOGI(TAG, "Using built-in certificate bundle for HTTPS");
                }
            }

            // Build auth + content type for standard HTTP/HTTPS (JSON content)
            https_client_mgr_auth_t auth = {0};
            // Map extended auth types
            switch (gd->auth.type)
            {
            case DEST_AUTH_BEARER:
                if (gd->auth.bearer && strlen(gd->auth.bearer) > 0)
                {
                    auth.bearer_token = gd->auth.bearer;
                }
                break;
            case DEST_AUTH_API_KEY_HEADER:
                if (gd->auth.api_key && strlen(gd->auth.api_key) > 0)
                {
                    auth.api_key = gd->auth.api_key;
                    auth.api_key_header_name = gd->auth.api_key_header_name; // may be NULL -> defaults to x-api-key
                }
                break;
            case DEST_AUTH_API_KEY_QUERY:
                if (gd->auth.api_key && strlen(gd->auth.api_key) > 0 && gd->auth.api_key_query_name && strlen(gd->auth.api_key_query_name) > 0)
                {
                    auth.api_key = gd->auth.api_key;
                    auth.api_key_query_name = gd->auth.api_key_query_name;
                }
                break;
            case DEST_AUTH_BASIC:
                auth.basic_username = gd->auth.basic_username;
                auth.basic_password = gd->auth.basic_password;
                break;
            case DEST_AUTH_NONE:
            default:
                break;
            }
            // Legacy fallback: if no explicit auth configured but api_token exists, use as Bearer
            if (!auth.bearer_token && gd->api_token && strlen(gd->api_token) > 0 && gd->auth.type == DEST_AUTH_NONE)
            {
                auth.bearer_token = gd->api_token;
            }
            // Append any extra query params configured in destination
            if (gd->query_params && gd->query_params_count > 0)
            {
                // Build a temporary array compatible with https_client_mgr.
                // DO NOT use alloca(): user config can make this huge and corrupt memory.
                size_t qp_count = gd->query_params_count;
                if (qp_count > AUTOPID_MAX_DEST_QUERY_PARAMS)
                {
                    qp_count = AUTOPID_MAX_DEST_QUERY_PARAMS;
                }

                https_client_mgr_query_kv_t *qp = heap_caps_calloc(qp_count, sizeof(https_client_mgr_query_kv_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
                if (qp)
                {
                    for (size_t qi = 0; qi < qp_count; ++qi)
                    {
                        qp[qi].key = gd->query_params[qi].key ? gd->query_params[qi].key : "";
                        qp[qi].value = gd->query_params[qi].value ? gd->query_params[qi].value : "";
                    }
                    auth.extra_query = qp;
                    auth.extra_query_count = qp_count;
                }
                else
                {
                    ESP_LOGW(TAG, "Failed to allocate query params array (count=%u)", (unsigned)qp_count);
                }
            }

            // For HTTPS, if host is raw IPv4 address, skip CN verification to allow self-signed IP certs
            if (url)
            {
                const char *host_start = strstr(url, "://");
                host_start = host_start ? host_start + 3 : url;
                // host ends at '/' or end
                char host_buf[64] = {0};
                size_t hi = 0;
                while (host_start[hi] && host_start[hi] != '/' && host_start[hi] != ':' && hi < sizeof(host_buf) - 1)
                {
                    host_buf[hi] = host_start[hi];
                    hi++;
                }
                bool is_ip = true;
                for (size_t k = 0; k < hi; k++)
                {
                    if ((host_buf[k] < '0' || host_buf[k] > '9') && host_buf[k] != '.')
                    {
                        is_ip = false;
                        break;
                    }
                }
                if (is_ip)
                {
                    cfg.skip_common_name = true;
                }
            }

            https_client_mgr_response_t resp = {0};
            esp_err_t err = https_client_mgr_request_with_auth(&cfg, HTTPS_METHOD_POST,
                                                               body, strlen(body),
                                                               "application/json",
                                                               &auth,
                                                               NULL,
                                                               &resp);

            bool ok = (err == ESP_OK && resp.is_success);
            if (ok)
            {
                ESP_LOGI(TAG, "HTTP(S) dest %u status %d success", i, resp.status_code);
                gd->consec_failures = 0;
                gd->backoff_ms = 0;
                // After initial successful settings push for this destination, switch to telemetry-only
                gd->settings_sent = true;
                gd->success_count++;
            }
            else
            {
                ESP_LOGE(TAG, "HTTP(S) dest %u request failed: %s", i, esp_err_to_name(err));
                gd->consec_failures++;
                gd->fail_count++;
                // Apply backoff logic (same as ABRP)
                if (gd->consec_failures >= AUTOPID_BACKOFF_RETRY_THRESHOLD)
                {
                    uint32_t min_cap = AUTOPID_BACKOFF_MIN_CAP_HTTP_MS;
                    uint32_t next = (gd->backoff_ms ? gd->backoff_ms : base_cycle);
                    if (next < min_cap)
                        next = next * 2;
                    if (next < min_cap)
                        next = min_cap;
                    uint32_t max_backoff = base_cycle * AUTOPID_BACKOFF_MAX_MULTIPLIER;
                    if (max_backoff < min_cap)
                        max_backoff = min_cap;
                    if (next > max_backoff)
                        next = max_backoff;
                    gd->backoff_ms = next;
                }
            }

            https_client_mgr_free_response(&resp);
            if (auth.extra_query)
            {
                free((void *)auth.extra_query);
                auth.extra_query = NULL;
                auth.extra_query_count = 0;
            }
            if (body) free(body);
            if (url) free(url);
            break;
        }
        case DEST_ABRP_API:
        {
            // Build URL (ABRP uses form data in body, not URL parameters)
            char *url = dest[0] ? strdup_psram(dest) : NULL;
            if (!url)
            {
                ESP_LOGW(TAG, "Destination %u missing URL", i);
                gd->fail_count++;
                break;
            }

            // Build body: ABRP uses URL-encoded form data, others use JSON
            char *body = NULL;
            if (gd->type == DEST_ABRP_API)
            {
                if (!gd->api_token || gd->api_token[0] == '\0')
                {
                    ESP_LOGW(TAG, "ABRP destination %u missing user token (api_token)", i);
                    free(url);
                    gd->fail_count++;
                    break;
                }

                char *tlm_data = build_abrp_payload(raw_json, autopid_config->vehicle_model);
                if (!tlm_data)
                {
                    ESP_LOGE(TAG, "Failed to build ABRP telemetry data");
                    free(url);
                    gd->fail_count++;
                    break;
                }

                // URL encode the telemetry JSON for form data
                char *encoded_tlm = url_encode_string(tlm_data);
                if (!encoded_tlm)
                {
                    ESP_LOGE(TAG, "Failed to URL encode telemetry data");
                    free(tlm_data);
                    free(url);
                    gd->fail_count++;
                    break;
                }

                // URL encode token
                char *encoded_token = url_encode_string(gd->api_token);
                if (!encoded_token)
                {
                    ESP_LOGE(TAG, "Failed to URL encode ABRP token");
                    free(encoded_tlm);
                    free(tlm_data);
                    free(url);
                    gd->fail_count++;
                    break;
                }

                // Build form data body: token=xxx&tlm=encoded_json
                // (car_model is embedded in tlm JSON when available)
                size_t body_len = strlen(encoded_token) + strlen(encoded_tlm) + 64;
                body = heap_caps_malloc(body_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (body)
                {
                    snprintf(body, body_len, "token=%s&tlm=%s", encoded_token, encoded_tlm);
                }

                if (encoded_token) free(encoded_token);
                if (encoded_tlm) free(encoded_tlm);
                if (tlm_data) free(tlm_data);
            }
            else
            {
                body = strdup_psram(raw_json);
            }
            if (!body)
            {
                ESP_LOGE(TAG, "Failed to allocate body");
                free(url);
                gd->fail_count++;
                break;
            }

            https_client_mgr_config_t cfg = {0};
            cfg.url = url;
            // Log destination URL plus current system time (epoch + ISO8601 UTC) for TLS timing diagnostics
            time_t _now = 0;
            time(&_now);
            struct tm _utc;
            gmtime_r(&_now, &_utc);
            char _tbuf[32];
            strftime(_tbuf, sizeof(_tbuf), "%Y-%m-%dT%H:%M:%SZ", &_utc);
            ESP_LOGI(TAG, "HTTP(S) dest %u URL: %s (epoch=%ld utc=%s)", i, url, (long)_now, _tbuf);
            // ABRP requests can involve ARP/DNS/route setup on first send; use a slightly larger timeout.
            cfg.timeout_ms = 5000;
            // ABRP API is always HTTPS - configure certificates appropriately
            cfg.use_crt_bundle = true; // ABRP always uses HTTPS with cert bundle by default
            // Build content-type; ABRP expects x-www-form-urlencoded and token in body, not Authorization header
            const char *content_type = (gd->type == DEST_ABRP_API)
                                           ? "application/x-www-form-urlencoded"
                                           : "application/json";

            // Build auth for ABRP: api_key can be passed as query param (?api_key=...) or
            // Authorization header with value "APIKEY <key>" (per Iternio Telemetry API).
            https_client_mgr_auth_t auth = {0};
            if (gd->type == DEST_ABRP_API)
            {
                switch (gd->auth.type)
                {
                case DEST_AUTH_API_KEY_QUERY:
                    if (gd->auth.api_key && gd->auth.api_key[0] && gd->auth.api_key_query_name && gd->auth.api_key_query_name[0])
                    {
                        auth.api_key = gd->auth.api_key;
                        auth.api_key_query_name = gd->auth.api_key_query_name;
                    }
                    break;
                case DEST_AUTH_API_KEY_HEADER:
                    if (gd->auth.api_key && gd->auth.api_key[0])
                    {
                        // If user sets header_name="Authorization" and api_key="<rawkey>", prefix with "APIKEY ".
                        const char *hn = gd->auth.api_key_header_name;
                        bool is_authz = (!hn) || (strcasecmp(hn, "Authorization") == 0);
                        if (is_authz)
                        {
                            const char *v = gd->auth.api_key;
                            bool already_prefixed = (strncasecmp(v, "APIKEY ", 7) == 0);
                            if (already_prefixed)
                            {
                                auth.api_key_header_name = hn ? hn : "Authorization";
                                auth.api_key = gd->auth.api_key;
                            }
                            else
                            {
                                // Build "APIKEY <key>" in PSRAM and keep it alive for request.
                                size_t alen = strlen(gd->auth.api_key) + 8;
                                char *aval = heap_caps_malloc(alen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                                if (aval)
                                {
                                    snprintf(aval, alen, "APIKEY %s", gd->auth.api_key);
                                    auth.api_key_header_name = hn ? hn : "Authorization";
                                    auth.api_key = aval;
                                }
                                else
                                {
                                    auth.api_key_header_name = hn ? hn : "Authorization";
                                    auth.api_key = gd->auth.api_key;
                                }
                            }
                        }
                        else
                        {
                            // Non-Authorization header name; send as-is.
                            auth.api_key_header_name = gd->auth.api_key_header_name;
                            auth.api_key = gd->auth.api_key;
                        }
                    }
                    break;
                case DEST_AUTH_NONE:
                default:
                    break;
                }

                // Append extra query params if configured
                if (gd->query_params && gd->query_params_count > 0)
                {
                    size_t qp_count = gd->query_params_count;
                    if (qp_count > AUTOPID_MAX_DEST_QUERY_PARAMS)
                    {
                        qp_count = AUTOPID_MAX_DEST_QUERY_PARAMS;
                    }
                    https_client_mgr_query_kv_t *qp = heap_caps_calloc(qp_count, sizeof(https_client_mgr_query_kv_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
                    if (qp)
                    {
                        for (size_t qi = 0; qi < qp_count; ++qi)
                        {
                            qp[qi].key = gd->query_params[qi].key ? gd->query_params[qi].key : "";
                            qp[qi].value = gd->query_params[qi].value ? gd->query_params[qi].value : "";
                        }
                        auth.extra_query = qp;
                        auth.extra_query_count = qp_count;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Failed to allocate query params array (count=%u)", (unsigned)qp_count);
                    }
                }

                // Helpful warning if no api_key configured anywhere
                if (!auth.api_key && (!strstr(url, "api_key=") && !strstr(url, "apiKey=") && !strstr(url, "apikey=")))
                {
                    ESP_LOGW(TAG, "ABRP destination %u has no api_key auth configured (expected query api_key=... or Authorization: APIKEY ...)", i);
                }
            }

            https_client_mgr_response_t resp = {0};
            // For ABRP (always HTTPS), if host is raw IPv4 address, skip CN verification to allow self-signed IP certs
            if (url)
            {
                const char *host_start = strstr(url, "://");
                host_start = host_start ? host_start + 3 : url;
                // host ends at '/' or end
                char host_buf[64] = {0};
                size_t hi = 0;
                while (host_start[hi] && host_start[hi] != '/' && host_start[hi] != ':' && hi < sizeof(host_buf) - 1)
                {
                    host_buf[hi] = host_start[hi];
                    hi++;
                }
                bool is_ip = true;
                for (size_t k = 0; k < hi; k++)
                {
                    if ((host_buf[k] < '0' || host_buf[k] > '9') && host_buf[k] != '.')
                    {
                        is_ip = false;
                        break;
                    }
                }
                if (is_ip)
                {
                    cfg.skip_common_name = true;
                }
            }

            // No auth struct here for ABRP unless configured elsewhere; token is already included in body
            esp_err_t err = https_client_mgr_request_with_auth(&cfg, HTTPS_METHOD_POST,
                                                               body, strlen(body),
                                                               content_type,
                                                               (gd->type == DEST_ABRP_API) ? &auth : NULL,
                                                               NULL,
                                                               &resp);
            bool ok = false;
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "HTTP(S) dest %u status %d success=%d", i, resp.status_code, resp.is_success);
                if (gd->type == DEST_ABRP_API)
                {
                    // ABRP sometimes returns HTTP 200 even on logical errors.
                    // Default to HTTP success, but if JSON includes "status", require it to be "ok".
                    ok = resp.is_success;
                    if (resp.data && resp.data_len > 0)
                    {
                        cJSON *jr = cJSON_Parse(resp.data);
                        if (jr)
                        {
                            cJSON *st = cJSON_GetObjectItemCaseSensitive(jr, "status");
                            const char *st_str = cJSON_IsString(st) ? st->valuestring : NULL;
                            if (st_str)
                            {
                                ok = (strcmp(st_str, "ok") == 0);
                            }
                            ESP_LOGI(TAG, "ABRP status: %s", st_str ? st_str : "<none>");
                            cJSON_Delete(jr);
                        }
                        else
                        {
                            ESP_LOGW(TAG, "ABRP response not JSON parseable; treating HTTP status as outcome");
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "ABRP empty response body; treating HTTP status as outcome");
                    }
                }
                else
                {
                    ok = resp.is_success;
                }
            }
            else
            {
                ESP_LOGE(TAG, "HTTP(S) dest %u request failed: %s", i, esp_err_to_name(err));
                ok = false;
            }

            if (ok)
            {
                gd->consec_failures = 0;
                gd->backoff_ms = 0;
                gd->success_count++;
            }
            else
            {
                gd->consec_failures++;
                gd->fail_count++;
            }
            // Backoff logic: after N failures, increase delay up to Mx base_cycle (or min cap)
            if (gd->consec_failures >= AUTOPID_BACKOFF_RETRY_THRESHOLD)
            {
                uint32_t min_cap = AUTOPID_BACKOFF_MIN_CAP_ABRP_MS;
                uint32_t next = (gd->backoff_ms ? gd->backoff_ms : base_cycle);
                if (next < min_cap)
                    next = next * 2; // exponential until min_cap
                if (next < min_cap)
                    next = min_cap; // ensure floor
                // limit to M * base_cycle if that is bigger than min_cap
                uint32_t max_backoff = base_cycle * AUTOPID_BACKOFF_MAX_MULTIPLIER;
                if (max_backoff < min_cap)
                    max_backoff = min_cap; // keep at least min_cap
                if (next > max_backoff)
                    next = max_backoff;
                gd->backoff_ms = next;
            }
            https_client_mgr_free_response(&resp);
            if (auth.extra_query)
            {
                free((void *)auth.extra_query);
                auth.extra_query = NULL;
                auth.extra_query_count = 0;
            }
            // If we allocated an "APIKEY <key>" string above, free it here.
            if (gd->type == DEST_ABRP_API && auth.api_key && gd->auth.api_key && auth.api_key != gd->auth.api_key)
            {
                free((void *)auth.api_key);
                auth.api_key = NULL;
            }
            if (body) free(body);
            if (url) free(url);
            break;
        }
        default:
            break;
        }
        // Reschedule timer if periodic
        if (effective_cycle > 0)
        {
            wc_timer_set(&gd->publish_timer, effective_cycle); // schedule next expiry
        }
        else
        {
            gd->publish_timer = 0; // event-based; remains immediate until triggered differently
        }
        // Yield a bit after network operations
        vTaskDelay(pdMS_TO_TICKS(5));
    }

#if defined(CONFIG_HEAP_POISONING_LIGHT) || defined(CONFIG_HEAP_POISONING_COMPREHENSIVE)
    // If heap corruption is happening during destination publishing, catch it here
    // (closer to the root cause) rather than later in unrelated code (e.g., WiFi RX).
    heap_caps_check_integrity_all(true);
#endif

    free(debug_json);
    free(raw_json);
}


// Helper to strip spaces/special chars for safe MQTT Object IDs
static void sanitize_for_id(char *dest, const char *src, size_t max_len) {
    size_t i = 0, j = 0;
    while (src[i] && j < max_len - 1) {
        if (isalnum((unsigned char)src[i])) {
            dest[j++] = tolower((unsigned char)src[i]);
        } else if (src[i] == ' ' || src[i] == '-' || src[i] == '_') {
            dest[j++] = '_';
        }
        i++;
    }
    dest[j] = '\0';
}

// Translates WiCAN units into Home Assistant device classes
static const char* get_ha_device_class(const char* unit) {
    if (!unit || strlen(unit) == 0) return "";
    
    // Voltage
    if (strcmp(unit, "V") == 0 || strcmp(unit, "mV") == 0) return "voltage";
    
    // Current
    if (strcmp(unit, "A") == 0 || strcmp(unit, "mA") == 0) return "current";
    
    // Temperature
    if (strcmp(unit, "°C") == 0 || strcmp(unit, "C") == 0 || 
        strcmp(unit, "°F") == 0 || strcmp(unit, "F") == 0 || 
        strcmp(unit, "K") == 0) return "temperature";
    
    // Pressure
    if (strcasecmp(unit, "kPa") == 0 || strcasecmp(unit, "psi") == 0 || 
        strcasecmp(unit, "bar") == 0 || strcasecmp(unit, "mbar") == 0 || 
        strcasecmp(unit, "Pa") == 0 || strcasecmp(unit, "hPa") == 0 || 
        strcasecmp(unit, "inHg") == 0 || strcasecmp(unit, "mmHg") == 0) return "pressure";
    
    // Speed
    if (strcasecmp(unit, "km/h") == 0 || strcasecmp(unit, "mph") == 0 || 
        strcasecmp(unit, "m/s") == 0 || strcasecmp(unit, "knots") == 0) return "speed";
    
    // Power
    if (strcmp(unit, "kW") == 0 || strcmp(unit, "W") == 0) return "power";
    
    // Energy
    if (strcmp(unit, "kWh") == 0 || strcmp(unit, "Wh") == 0) return "energy";
    
    // Distance
    if (strcmp(unit, "km") == 0 || strcmp(unit, "mi") == 0 || 
        strcmp(unit, "m") == 0 || strcmp(unit, "cm") == 0 || 
        strcmp(unit, "mm") == 0 || strcmp(unit, "ft") == 0 || 
        strcmp(unit, "in") == 0 || strcmp(unit, "yd") == 0) return "distance";

    // Volume
    if (strcmp(unit, "L") == 0 || strcmp(unit, "mL") == 0 || 
        strcasecmp(unit, "gal") == 0) return "volume";

    // Weight / Mass
    if (strcmp(unit, "kg") == 0 || strcmp(unit, "g") == 0 || 
        strcmp(unit, "mg") == 0 || strcmp(unit, "lb") == 0 || 
        strcmp(unit, "lbs") == 0 || strcmp(unit, "oz") == 0) return "weight";

    // Duration / Time
    if (strcmp(unit, "d") == 0 || strcmp(unit, "h") == 0 || 
        strcmp(unit, "hr") == 0 || strcmp(unit, "min") == 0 || 
        strcmp(unit, "s") == 0 || strcmp(unit, "ms") == 0) return "duration";

    // Frequency
    if (strcasecmp(unit, "Hz") == 0 || strcasecmp(unit, "kHz") == 0 || 
        strcasecmp(unit, "MHz") == 0) return "frequency";
    
    // Fallback: HA handles generic numbers perfectly without a specific device class.
    // NOTE: We intentionally leave "%" blank because it could represent Battery (SoC), 
    // Engine Load, Throttle Position, or Fuel Level. Users must manually set "battery"
    // in the class field if they want the HA battery icon!
    return ""; 
}

// Forward declaration to let the compiler know this returns a string pointer
const char *config_server_get_mqtt_en(void);

static void autopid_publish_discovery_task(void *pvParameters)
{
    // Ensure MQTT is active before proceeding
    if (strcmp(config_server_get_mqtt_en(), "enable") != 0) {
        return;
    }

    // --- NEW: THE GATEKEEPER ---
    // Stall this task indefinitely until the underlying MQTT client 
    // (and the VPN tunnel it relies on) actually successfully connects to the broker.
    ESP_LOGI(TAG, "Auto Discovery waiting for MQTT/VPN connection...");
    while (mqtt_connected() == 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "MQTT Connected! Publishing Auto Discovery payloads...");
    // ---------------------------
    
    // --- Pull Custom Settings from Web GUI ---
    const char *base_path = config_server_get_mqtt_disc_path();
    const char *disc_id   = config_server_get_mqtt_disc_id();
    const char *disc_name = config_server_get_mqtt_disc_name();
    const char *disc_model = config_server_get_mqtt_disc_model();
    const char *disc_mfg   = config_server_get_mqtt_disc_mfg();
    const char *disc_area  = config_server_get_mqtt_disc_area();

    // --- Build The Global Device Blueprint ---
    cJSON *device_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(device_obj, "name", disc_name);
    cJSON_AddStringToObject(device_obj, "model", disc_model);
    cJSON_AddStringToObject(device_obj, "manufacturer", disc_mfg);
    cJSON_AddStringToObject(device_obj, "suggested_area", disc_area);
    cJSON_AddStringToObject(device_obj, "sw_version", "WiCAN Pro Firmware");

    // Identifiers MUST match exactly across all sensors to group them
    cJSON *identifiers_arr = cJSON_CreateArray();
    cJSON_AddItemToArray(identifiers_arr, cJSON_CreateString(disc_id));
    cJSON_AddItemToObject(device_obj, "identifiers", identifiers_arr);

    // Helper macro to generate and publish a sensor
    #define PUBLISH_HA_SENSOR(sensor_name, state_topic, val_template, unit, dev_class) do { \
        cJSON *payload = cJSON_CreateObject(); \
        cJSON_AddStringToObject(payload, "name", sensor_name); \
        cJSON_AddStringToObject(payload, "state_topic", state_topic); \
        cJSON_AddStringToObject(payload, "value_template", val_template); \
        if (dev_class && strlen(dev_class) > 0) cJSON_AddStringToObject(payload, "device_class", dev_class); \
        if (unit && strlen(unit) > 0) { \
            cJSON_AddStringToObject(payload, "unit_of_measurement", unit); \
            cJSON_AddStringToObject(payload, "state_class", "measurement"); \
	    cJSON_AddNumberToObject(payload, "suggested_display_precision", 1); \
        } \
        \
        char safe_name[64]; \
        sanitize_for_id(safe_name, sensor_name, sizeof(safe_name)); \
        char unique_id[128]; \
        snprintf(unique_id, sizeof(unique_id), "%s_%s", disc_id, safe_name); \
        cJSON_AddStringToObject(payload, "unique_id", unique_id); \
        \
        cJSON_AddItemToObject(payload, "device", cJSON_Duplicate(device_obj, true)); \
        \
        char *json_str = cJSON_PrintUnformatted(payload); \
        if (json_str) { \
            char topic[256]; \
            snprintf(topic, sizeof(topic), "%s/sensor/%s/%s/config", base_path, disc_id, safe_name); \
            mqtt_publish(topic, json_str, strlen(json_str), 0, 1); \
            free(json_str); \
        } \
        cJSON_Delete(payload); \
        vTaskDelay(pdMS_TO_TICKS(50)); \
    } while(0)

    // 2. Publish WiCAN Status Fields (if enabled)
    if (strcmp(config_server_get_mqtt_disc_status_en(), "enable") == 0) {
        char *status_topic = config_server_get_mqtt_status_topic();
        
        // --- Power & System ---
        PUBLISH_HA_SENSOR("Battery Voltage", status_topic, "{{ value_json.batt_voltage | replace('V', '') | float | round(1) }}", "V", "voltage");
        PUBLISH_HA_SENSOR("Uptime", status_topic, "{{ value_json.uptime }}", "", "");
        PUBLISH_HA_SENSOR("Firmware Version", status_topic, "{{ value_json.fw_version }}", "", "");
        PUBLISH_HA_SENSOR("Hardware Version", status_topic, "{{ value_json.hw_version }}", "", "");
        PUBLISH_HA_SENSOR("Boot Count", status_topic, "{{ value_json.restart_boot_count }}", "", "");
        PUBLISH_HA_SENSOR("Unexpected Resets", status_topic, "{{ value_json.restart_unexpected_reset_count }}", "", "");

        // --- NEW RESTART DIAGNOSTIC SENSORS ---
        PUBLISH_HA_SENSOR("Planned Restart", status_topic, "{{ value_json.restart_last_planned_reason }}", "", "");
        PUBLISH_HA_SENSOR("Restart Source", status_topic, "{{ value_json.restart_last_source }}", "", "");
	
        // --- Network ---
        PUBLISH_HA_SENSOR("WiFi Mode", status_topic, "{{ value_json.wifi_mode }}", "", "");
        PUBLISH_HA_SENSOR("WiFi SSID", status_topic, "{{ value_json.sta_ssid }}", "", "");
        PUBLISH_HA_SENSOR("IP Address", status_topic, "{{ value_json.sta_ip }}", "", "");
        PUBLISH_HA_SENSOR("MAC Address", status_topic, "{{ value_json.mac_address }}", "", "");
        PUBLISH_HA_SENSOR("AP Auto Disable", status_topic, "{{ value_json.ap_auto_disable }}", "", "");
        PUBLISH_HA_SENSOR("STA IP Type", status_topic, "{{ value_json.sta_ip_type }}", "", "");
        PUBLISH_HA_SENSOR("Home Priority", status_topic, "{{ value_json.sta_home_priority }}", "", "");
        
        // --- Hardware & Protocols ---
        PUBLISH_HA_SENSOR("BLE Status", status_topic, "{{ value_json.ble_status }}", "", "");
        PUBLISH_HA_SENSOR("CAN Mode", status_topic, "{{ value_json.can_mode }}", "", "");
        PUBLISH_HA_SENSOR("CAN Datarate", status_topic, "{{ value_json.can_datarate }}", "", "");
        PUBLISH_HA_SENSOR("Protocol", status_topic, "{{ value_json.protocol }}", "", "");
        
        // --- Sleep & Wakeup Settings ---
        PUBLISH_HA_SENSOR("Sleep Status", status_topic, "{{ value_json.sleep_status }}", "", "");
        PUBLISH_HA_SENSOR("Sleep Voltage", status_topic, "{{ value_json.sleep_volt | float | round(1) }}", "V", "voltage");
        PUBLISH_HA_SENSOR("Sleep Time", status_topic, "{{ value_json.sleep_time }}", "min", "");
        PUBLISH_HA_SENSOR("Wakeup Voltage", status_topic, "{{ value_json.wakeup_volt | float | round(1) }}", "V", "voltage");
        PUBLISH_HA_SENSOR("Wakeup Mode", status_topic, "{{ value_json.wakeup_mode }}", "", "");
        PUBLISH_HA_SENSOR("Wakeup Interval", status_topic, "{{ value_json.wakeup_interval }}", "min", "");
        
        // --- Diagnostics & Status ---
        PUBLISH_HA_SENSOR("ECU Status", status_topic, "{{ value_json.ecu_status }}", "", "");
        PUBLISH_HA_SENSOR("OBD Chip Status", status_topic, "{{ value_json.obd_chip_status }}", "", "");
        PUBLISH_HA_SENSOR("VPN Status", status_topic, "{{ value_json.vpn_status }}", "", "");
        PUBLISH_HA_SENSOR("Time Synced", status_topic, "{{ value_json.time_synced }}", "", "");
	PUBLISH_HA_SENSOR("VPN IP", status_topic, "{{ value_json.vpn_ip }}", "", "");
    }

    // 3. Publish OBD2/CAN PIDs (if enabled)
    if (strcmp(config_server_get_mqtt_disc_pids_en(), "enable") == 0) {
        if (autopid_lock(1000)) {
            if (autopid_config) {
                // --- A. Process Grouped PIDs ---
                if (autopid_config->use_groups) {
                    for (uint32_t g = 0; g < autopid_config->group_count; g++) {
                        pid_group_t *grp = &autopid_config->groups[g];
                        if (!grp->enabled) continue;
                        
                        for (uint32_t i = 0; i < grp->pid_count; i++) {
                            pid_data_t *curr_pid = grp->pids[i];
                            if (!curr_pid || !curr_pid->enabled) continue;
                            
                            for (uint32_t j = 0; j < curr_pid->parameters_count; j++) {
                                parameter_t *param = &curr_pid->parameters[j];
                                if (!param->enabled) continue;

                                // STRICTLY ONLY DISCOVER DEFAULT DESTINATIONS
                                if (param->destination_type != DEST_DEFAULT) continue;

                                  char val_template[256];
                                  snprintf(val_template, sizeof(val_template), "{{ value_json['%s'] if '%s' in value_json else this.state }}", param->name, param->name);

                                const char *target_topic = config_server_get_mqtt_rx_topic();

                                // Apply User Class OR fallback to Auto-Mapping
                                const char *final_class = (param->class && strlen(param->class) > 0 && strcmp(param->class, "none") != 0) 
                                    ? param->class 
                                    : get_ha_device_class(param->unit);

                                PUBLISH_HA_SENSOR(param->name, target_topic, val_template, param->unit, final_class);
                            }
                        }
                    }
                }

                // --- B. Process Flat List PIDs ---
                if (autopid_config->pid_count > 0) {
                    for (uint32_t i = 0; i < autopid_config->pid_count; i++) {
                        pid_data_t *curr_pid = &autopid_config->pids[i];

                        // Skip if it's already handled by a group above
                        bool is_in_group = false;
                        if (autopid_config->use_groups) {
                            for (uint32_t g = 0; g < autopid_config->group_count; g++) {
                                for (uint32_t gp = 0; gp < autopid_config->groups[g].pid_count; gp++) {
                                    if (autopid_config->groups[g].pids[gp] == curr_pid) {
                                        is_in_group = true; break;
                                    }
                                }
                                if (is_in_group) break;
                            }
                        }
                        if (is_in_group) continue;

                        if (!curr_pid->enabled) continue;
                        
                        for (uint32_t j = 0; j < curr_pid->parameters_count; j++) {
                            parameter_t *param = &curr_pid->parameters[j];
                            if (!param->enabled) continue;
                            
                            // STRICTLY ONLY DISCOVER DEFAULT DESTINATIONS
                            if (param->destination_type != DEST_DEFAULT) continue;

                            char val_template[256];
                            snprintf(val_template, sizeof(val_template), "{{ value_json['%s'] if '%s' in value_json else this.state }}", param->name, param->name);

                            const char *target_topic = config_server_get_mqtt_rx_topic();

                            // Apply User Class OR fallback to Auto-Mapping
                            const char *final_class = (param->class && strlen(param->class) > 0) 
                                ? param->class 
                                : get_ha_device_class(param->unit);

                            PUBLISH_HA_SENSOR(param->name, target_topic, val_template, param->unit, final_class);
                        }
                    }
                }
            }
            autopid_unlock();
        }
    }

    cJSON_Delete(device_obj);
    ESP_LOGI(TAG, "Auto Discovery Publishing Complete.");

    vTaskDelete(NULL);
}


void autopid_publish_discovery(void)
{
    // Spawn an independent task so the MQTT event loop is never blocked
    xTaskCreate(autopid_publish_discovery_task, "ha_disc", 6144, NULL, 5, NULL);
}


char *autopid_get_destinations_stats_json(void)
{
    if (!autopid_config || !autopid_config->mutex)
    {
        // Keep response shape stable even if not ready
        char *fallback = strdup("{\"destinations\":[]}");
        return fallback;
    }

    char *printed = NULL;
    if (xSemaphoreTake(autopid_config->mutex, pdMS_TO_TICKS(2000)) == pdTRUE)
    {
        cJSON *root = cJSON_CreateObject();
        cJSON *arr = cJSON_CreateArray();
        if (root && arr)
        {
            for (uint32_t i = 0; i < autopid_config->destinations_count; i++)
            {
                group_destination_t *gd = &autopid_config->destinations[i];
                cJSON *o = cJSON_CreateObject();
                if (!o)
                    continue;
                cJSON_AddNumberToObject(o, "success", (double)gd->success_count);
                cJSON_AddNumberToObject(o, "fail", (double)gd->fail_count);
                cJSON_AddItemToArray(arr, o);
            }
            cJSON_AddItemToObject(root, "destinations", arr);

            /* ---> NEW CODE START <--- */
            if (autopid_config->use_groups) 
            {
                cJSON *grp_arr = cJSON_CreateArray();
                if (grp_arr) 
                {
                    for (uint32_t g = 0; g < autopid_config->group_count; g++) 
                    {
                        // Safely add the boolean flag for each group to the JSON array
                        cJSON_AddItemToArray(grp_arr, cJSON_CreateBool(autopid_config->groups[g].mqtt_active_flag));
                    }
                    cJSON_AddItemToObject(root, "active_groups", grp_arr);
                }
            }
            /* ---> NEW CODE END <--- */

            printed = cJSON_PrintUnformatted(root);
        }
        else
        {
            if (arr)
                cJSON_Delete(arr);
        }
        if (root)
            cJSON_Delete(root);
        xSemaphoreGive(autopid_config->mutex);
    }

    if (!printed)
    {
        printed = strdup("{\"destinations\":[]}");
    }
    return printed; // caller must free()
}

 // -------------------------------------------------------------------------
// [UPDATED] Config Exporter (Supports Groups + Destination Types)
// -------------------------------------------------------------------------
char *autopid_get_config(void)
{
    if (autopid_config_json == NULL)
    {
        if (!autopid_config || !autopid_config->mutex)
        {
            ESP_LOGE(TAG, "autopid_get_config: autopid_config not ready");
            return NULL;
        }

        if (xSemaphoreTake(autopid_config->mutex, portMAX_DELAY) == pdTRUE)
        {
            cJSON *root = cJSON_CreateObject();
            if (!root)
            {
                ESP_LOGE(TAG, "Failed to create JSON object");
                xSemaphoreGive(autopid_config->mutex);
                return NULL;
            }

            cJSON_AddStringToObject(root, "supply_mode_enabled",
                                    autopid_config->supply_mode_enabled ? "enable" : "disable");
            cJSON_AddNumberToObject(root, "boot_pid_polling_keep_alive_seconds",
                                    autopid_config->boot_pid_polling_keep_alive_seconds);
            cJSON_AddStringToObject(root, "voltage_rise_wakeup",
                                    autopid_config->voltage_rise_wakeup_enabled ? "enable" : "disable");
            cJSON_AddNumberToObject(root, "voltage_rise_threshold", autopid_config->voltage_rise_threshold);
            cJSON_AddNumberToObject(root, "voltage_rise_time_seconds", autopid_config->voltage_rise_time_seconds);
            cJSON_AddStringToObject(root, "imu_voltage_override",
                                    autopid_config->imu_voltage_override_enabled ? "enable" : "disable");
            if (autopid_config->supply_mode_pid_name && autopid_config->supply_mode_pid_name[0] != '\0')
            {
                cJSON_AddStringToObject(root, "supply_mode_pid_name", autopid_config->supply_mode_pid_name);
            }
            if (autopid_config->supply_mode_operator[0] != '\0')
            {
                cJSON_AddStringToObject(root, "supply_mode_operator", autopid_config->supply_mode_operator);
            }
            if (autopid_config->supply_mode_enabled)
            {
                cJSON_AddNumberToObject(root, "supply_mode_value", autopid_config->supply_mode_value);
            }

            // 1. Export Groups (Primary Data Source for New UI)
            if (autopid_config->use_groups && autopid_config->group_count > 0)
            {
                cJSON *groups_array = cJSON_CreateArray();
                for (int g = 0; g < autopid_config->group_count; g++)
                {
                    pid_group_t *grp = &autopid_config->groups[g];
                    cJSON *g_obj = cJSON_CreateObject();
                    
                    cJSON_AddStringToObject(g_obj, "group_name", grp->name ? grp->name : "Group");
		    cJSON_AddBoolToObject(g_obj, "enabled", grp->enabled);
		    if (grp->mqtt_topic) cJSON_AddStringToObject(g_obj, "mqtt_topic", grp->mqtt_topic);
                    if (grp->init) cJSON_AddStringToObject(g_obj, "init", grp->init);
                    cJSON_AddNumberToObject(g_obj, "period", grp->period);
                    
                    // Condition mapping
                    const char *cond_str = "always";
                    if (grp->detection_method == DETECTION_VOLTAGE) cond_str = "voltage";
                    else if (grp->detection_method == DETECTION_ADAPTIVE_RPM) cond_str = "engine_running";
                    else if (grp->detection_method == DETECTION_MQTT) cond_str = "mqtt_on_demand";
                    else if (grp->detection_method == DETECTION_CUSTOM_PID) cond_str = "custom_pid"; // <--- ADD THIS
                    cJSON_AddStringToObject(g_obj, "condition", cond_str);

                    // --- NEW: Gatekeeper Export ---
                    cJSON_AddNumberToObject(g_obj, "wake_voltage", grp->wake_voltage);
                    if (grp->custom_pid_init) cJSON_AddStringToObject(g_obj, "custom_pid_init", grp->custom_pid_init);
                    if (grp->custom_pid_mode) cJSON_AddStringToObject(g_obj, "custom_pid_mode", grp->custom_pid_mode);
                    if (grp->custom_pid_expr) cJSON_AddStringToObject(g_obj, "custom_pid_expr", grp->custom_pid_expr);
                    if (grp->custom_pid_operator) cJSON_AddStringToObject(g_obj, "custom_pid_operator", grp->custom_pid_operator);
                    cJSON_AddNumberToObject(g_obj, "custom_pid_value", grp->custom_pid_value);
                    // ------------------------------

                    // Add PIDs for this group
                    cJSON *pids_array = cJSON_CreateArray();
                    for (int i = 0; i < grp->pid_count; i++)
                    {
                        pid_data_t *curr_pid = grp->pids[i];
                        if (!curr_pid) continue;

                        cJSON *p_obj = cJSON_CreateObject();
                        if (curr_pid->cmd) {
                            char *clean = strdup_psram(curr_pid->cmd);
                            if (clean) {
                                char *cr = strchr(clean, '\r');
                                if (cr) *cr = 0;
                                cJSON_AddStringToObject(p_obj, "pid", clean);
                                free(clean);
                            }
                        }
                        cJSON_AddBoolToObject(p_obj, "enabled", curr_pid->enabled);
                        if (curr_pid->period > 0) cJSON_AddNumberToObject(p_obj, "period", curr_pid->period);

                        // Add Parameters (Extractors)
                        cJSON *params_array = cJSON_CreateArray();
                        for (int j = 0; j < curr_pid->parameters_count; j++)
                        {
                            parameter_t *param = &curr_pid->parameters[j];
                            cJSON *param_details = cJSON_CreateObject();
                            
                            cJSON_AddStringToObject(param_details, "name", param->name);
                            if (param->expression) cJSON_AddStringToObject(param_details, "expression", param->expression);
                            if (param->unit) cJSON_AddStringToObject(param_details, "unit", param->unit);
                            if (param->class) cJSON_AddStringToObject(param_details, "class", param->class);
                            cJSON_AddNumberToObject(param_details, "min", param->min);
                            cJSON_AddNumberToObject(param_details, "max", param->max);
                            cJSON_AddBoolToObject(param_details, "enabled", param->enabled);
                            cJSON_AddBoolToObject(param_details, "onchange", param->onchange);

                            // Destination Type Enum -> String
                            const char *dtype_str = "Default";
                            switch(param->destination_type) {
                                case DEST_MQTT_TOPIC: dtype_str = "MQTT_Topic"; break;
                                case DEST_MQTT_WALLBOX: dtype_str = "MQTT_WallBox"; break;
                                case DEST_HTTP: dtype_str = "HTTP"; break;
                                case DEST_HTTPS: dtype_str = "HTTPS"; break;
                                case DEST_ABRP_API: dtype_str = "ABRP_API"; break;
	 			  //case DEST_BLE: dtype_str = "BLE"; break;
				  //case DEST_SCREEN: dtype_str = "Screen"; break;
				  //case DEST_LOGGER: dtype_str = "Logger"; break;
                                default: dtype_str = "Default"; break;
                            }
                            cJSON_AddStringToObject(param_details, "destination_type", dtype_str);
                            
                            if (param->destination) cJSON_AddStringToObject(param_details, "send_to", param->destination);

                            cJSON_AddItemToArray(params_array, param_details);
                        }
                        cJSON_AddItemToObject(p_obj, "parameters", params_array);
                        cJSON_AddItemToArray(pids_array, p_obj);
                    }
                    cJSON_AddItemToObject(g_obj, "pids", pids_array);
                    cJSON_AddItemToArray(groups_array, g_obj);
                }
                cJSON_AddItemToObject(root, "pid_groups", groups_array);
            }

            // 2. Export CAN Filters
            if (autopid_config->can_filters_count > 0)
            {
                cJSON *filters_array = cJSON_CreateArray();
                for (uint32_t fi = 0; fi < autopid_config->can_filters_count; fi++)
                {
                    can_filter_t *f = &autopid_config->can_filters[fi];
                    cJSON *f_obj = cJSON_CreateObject();
                    cJSON_AddNumberToObject(f_obj, "frame_id", f->frame_id);
                    
                    cJSON *f_params = cJSON_CreateArray();
                    for (uint32_t pi = 0; pi < f->parameters_count; pi++)
                    {
                        parameter_t *p = &f->parameters[pi];
                        cJSON *p_obj = cJSON_CreateObject();
                        cJSON_AddStringToObject(p_obj, "name", p->name);
                        cJSON_AddStringToObject(p_obj, "expression", p->expression);
                        cJSON_AddBoolToObject(p_obj, "enabled", p->enabled);
                        cJSON_AddItemToArray(f_params, p_obj);
                    }
                    cJSON_AddItemToObject(f_obj, "parameters", f_params);
                    cJSON_AddItemToArray(filters_array, f_obj);
                }
                cJSON_AddItemToObject(root, "can_filters", filters_array);
            }

            char *json_tmp = cJSON_PrintUnformatted(root);
            if (json_tmp)
            {
                autopid_config_json = strdup_psram(json_tmp);
                free(json_tmp);
            }
            cJSON_Delete(root);
            xSemaphoreGive(autopid_config->mutex);
        }
    }
    return autopid_config_json;
}
 
// Forward declarations (used by CAN-filter helpers below)
static void send_commands(char *commands, uint32_t delay_ms);
static void publish_parameter_mqtt(parameter_t *param);

static void send_can_filter_cmd(uint32_t frame_id)
{
    // Ensure current CAN protocol matches frame-id width before setting ATCRA.
    // For ISO15765 CAN:
    // - 6: 11-bit 500k, 7: 29-bit 500k
    // - 8: 11-bit 250k, 9: 29-bit 250k
    // If we're on a CAN protocol family, keep it consistent with the filter ID.
    int32_t current_protocol_number = -1;
    (void)autopid_get_protocol_number(&current_protocol_number);
    bool is_extended = (frame_id > 0x7FF);

    int32_t desired_protocol_number = -1;
    if (current_protocol_number == 6 || current_protocol_number == 7)
    {
        desired_protocol_number = is_extended ? 7 : 6;
    }
    else if (current_protocol_number == 8 || current_protocol_number == 9)
    {
        desired_protocol_number = is_extended ? 9 : 8;
    }

    if (desired_protocol_number != -1 && desired_protocol_number != current_protocol_number)
    {
        char proto_cmd[10];
        snprintf(proto_cmd, sizeof(proto_cmd), "ATTP%01lX\r", (unsigned long)desired_protocol_number);
        send_commands(proto_cmd, 2);
    }

    // ATCRA expects 3 hex digits for 11-bit, 8 hex digits for 29-bit.
    char cmd[20];
    if (frame_id <= 0x7FF)
    {
        snprintf(cmd, sizeof(cmd), "ATCRA%03lX\r", (unsigned long)frame_id);
    }
    else
    {
        snprintf(cmd, sizeof(cmd), "ATCRA%08lX\r", (unsigned long)frame_id);
    }
    send_commands(cmd, 2);
}

static void bytes_to_hex_str(char *out, size_t out_size, const uint8_t *data, size_t data_len)
{
    if (!out || out_size == 0)
        return;
    out[0] = '\0';
    if (!data || data_len == 0)
        return;

    size_t w = 0;
    for (size_t i = 0; i < data_len; i++)
    {
        if (w + 3 >= out_size)
            break;
        int n = snprintf(out + w, out_size - w, "%02X", (unsigned)data[i]);
        if (n <= 0)
            break;
        w += (size_t)n;
        if (i + 1 < data_len)
        {
            if (w + 2 >= out_size)
                break;
            out[w++] = ' ';
            out[w] = '\0';
        }
    }
}

static void process_can_filter_frame(can_filter_t *f, const response_t *rsp)
{
    if (!f || !rsp)
        return;
    if (rsp->length == 0)
        return;

    // Debug log raw response bytes for this filter (enable DEBUG log level to see)
    {
        const uint32_t max_log_bytes = 32;
        uint32_t n = rsp->length;
        if (n > max_log_bytes)
            n = max_log_bytes;
        char hex[3 * max_log_bytes + 1];
        bytes_to_hex_str(hex, sizeof(hex), rsp->data, n);
        ESP_LOGI(TAG, "CANFLT 0x%lX rsp_len=%lu data[%lu]=%s", (unsigned long)f->frame_id,
                 (unsigned long)rsp->length, (unsigned long)n, hex);
    }

    for (uint32_t pi = 0; pi < f->parameters_count; pi++)
    {
        parameter_t *param = &f->parameters[pi];
        if (!param->enabled)
            continue;
        if (!param || !param->expression)
            continue;
        if (!wc_timer_is_expired(&param->timer))
            continue;

        // Reset timer with parameter period
        wc_timer_set(&param->timer, (param->period == 0) ? 5000 : param->period);

        double result = 0;
        if (evaluate_expression((uint8_t *)param->expression, (uint8_t *)rsp->data, 0, &result))
        {
            if (autopid_prepare_parameter_value(param, result, &param->value, "CANFLT"))
            {
                param->failed = false;
                ESP_LOGI(TAG, "CANFLT 0x%lX param=%s result=%.2f", (unsigned long)f->frame_id,
                         param->name ? param->name : "(null)", (double)param->value);
		autopid_pid_update_hook(param, param->value);
                autopid_update_supply_mode_state_from_param(param);
                publish_parameter_mqtt(param);
            }
        }
        else
        {
            param->failed = true;
            ESP_LOGE(TAG, "CANFLT 0x%lX param=%s eval_failed", (unsigned long)f->frame_id,
                     param->name ? param->name : "(null)");
        }
    }
}

void parse_elm327_response(char *buffer, response_t *response)
{
    if (buffer == NULL || response == NULL)
    {
        ESP_LOGE(TAG, "Invalid buffer or response pointer");
        return;
    }

    ESP_LOGI(TAG, "Starting to parse ELM327 response. Input buffer: %s", buffer);

    // --- FIXED: Allocate the backup buffer safely on the HEAP, not the stack! ---
    size_t buf_len = strlen(buffer);
    char *original_buffer = heap_caps_malloc(buf_len + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (original_buffer != NULL) {
        strcpy(original_buffer, buffer);
    } else {
        ESP_LOGE(TAG, "Failed to allocate original_buffer on heap");
    }
    // ----------------------------------------------------------------------------
    
    int k = 0;
    int frame_count = 0;
    char *frame;
    char *data_start;
    uint32_t lowest_header = UINT32_MAX; 
    uint32_t highest_header = 0;         
    uint32_t first_header = 0;
    bool all_headers_same = true;
    uint8_t *lowest_header_data = NULL; 
    uint8_t lowest_header_length = 0;

    // --- ISO-TP TRACKING VARIABLES ---
    uint32_t multiframe_header = 0;
    bool is_iso_tp_multiframe = false;
    uint8_t expected_seq = 1;
    bool sequence_error = false;
    // ---------------------------------
    
    frame = strtok(buffer, "\r\n");
    ESP_LOGI(TAG, "First frame: %s", frame ? frame : "NULL");

    int32_t current_protocol_number = -1;

    if (autopid_get_protocol_number(&current_protocol_number) == ESP_OK)
    {
        ESP_LOGI(TAG, "Current protocol number: %ld", current_protocol_number);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to get protocol number");
    }

    while (frame != NULL)
    {
        ESP_LOGD(TAG, "Processing frame %d: %s", frame_count + 1, frame);
        frame_count++;

        // Remove trailing '>' if present
        size_t len = strlen(frame);
        if (len > 0 && frame[len - 1] == '>')
        {
            frame[len - 1] = '\0';
            ESP_LOGV(TAG, "Removed trailing '>' from frame");
        }

        data_start = strchr(frame, ' ');
        if (data_start != NULL)
        {
            int header_length = data_start - frame;
            if (header_length > 8) {
                header_length = 8;
            }
            char header_str[9] = {0};
            strncpy(header_str, frame, header_length);
            uint32_t current_header = strtoul(header_str, NULL, 16);
            ESP_LOGD(TAG, "Frame %d header: 0x%lX (length: %d)", frame_count, current_header, header_length);

            if (current_header > highest_header)
            {
                highest_header = current_header;
            }

            if (frame_count == 1)
            {
                first_header = current_header;
            }
            else if (current_header != first_header)
            {
                all_headers_same = false;
            }

            data_start++;
            if (current_protocol_number == -1 || (current_protocol_number > sizeof(autopid_protocol_header_length)))
            {
                switch (header_length)
                {
                case 2:
                    data_start += 9;
                    break;
                case 3:
                case 8:
                    break;
                default:
                    frame = strtok(NULL, "\r\n");
                    continue;
                }
            }
            else
            {
                data_start += (uint8_t)autopid_protocol_header_length[current_protocol_number];
            }

            char *current_data_start = data_start;
            int current_length = 0;

            // --- ISO-TP SEQUENCE VALIDATOR ---
            char *pci_start = current_data_start;
            while (*pci_start == ' ') pci_start++; 
            
            if (strlen(pci_start) >= 2) {
                char pci_byte_str[3] = {pci_start[0], pci_start[1], 0};
                uint8_t pci_byte = (uint8_t)strtol(pci_byte_str, NULL, 16);
                
                if (is_iso_tp_multiframe && current_header == multiframe_header) {
                    if ((pci_byte & 0xF0) == 0x20) {
                        uint8_t seq = pci_byte & 0x0F;
                        if (seq != expected_seq) {
                            sequence_error = true;
                            break; 
                        }
                        expected_seq = (expected_seq + 1) & 0x0F;
                    }
                } 
                else if (!is_iso_tp_multiframe && (pci_byte & 0xF0) == 0x10) {
                    is_iso_tp_multiframe = true;
                    multiframe_header = current_header;
                    expected_seq = 1;
                }
            }
            // ---------------------------------
            
            char *temp_data = data_start;
            while (*temp_data != '\0')
            {
                if (*temp_data == ' ')
                {
                    temp_data++;
                    continue;
                }
                if (strlen(temp_data) < 2)
                    break;
                current_length++;
                temp_data += 2;
            }

            if (current_header < lowest_header)
            {
                lowest_header = current_header;
                lowest_header_length = current_length;

                if (lowest_header_data == NULL)
                {
                    lowest_header_data = (uint8_t *)heap_caps_malloc(current_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                }
                else
                {
                    lowest_header_data = (uint8_t *)heap_caps_realloc(lowest_header_data, current_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                }

                int idx = 0;
                while (*current_data_start != '\0')
                {
                    if (*current_data_start == ' ')
                    {
                        current_data_start++;
                        continue;
                    }
                    if (strlen(current_data_start) < 2)
                        break;

                    char byte_str[3] = {current_data_start[0], current_data_start[1], 0};
                    lowest_header_data[idx++] = (unsigned char)strtol(byte_str, NULL, 16);
                    current_data_start += 2;
                }
            }

            while (*data_start != '\0')
            {
                if (*data_start == ' ')
                {
                    data_start++;
                    continue;
                }
                if (strlen(data_start) < 2)
                {
                    break;
                }

                char byte_str[3] = {data_start[0], data_start[1], 0};
                response->data[k] = (unsigned char)strtol(byte_str, NULL, 16);
                k++;
                data_start += 2;
            }
        }
        else
        {
            ESP_LOGW(TAG, "No space delimiter found in frame: %s", frame);
        }
        frame = strtok(NULL, "\r\n");
    }

    // --- ABORT, REJECT, AND BUNDLE HEX FOR UI ---
    if (sequence_error) {
        if (original_buffer != NULL) {
            // Flatten the original buffer (replace newlines with spaces)
            for(int i = 0; original_buffer[i] != '\0'; i++) {
                if(original_buffer[i] == '\r' || original_buffer[i] == '\n') {
                    original_buffer[i] = ' ';
                }
            }
            snprintf((char *)response->data, AUTOPID_BUFFER_SIZE, "error: missing frames [%.1024s]", original_buffer);
        } else {
            snprintf((char *)response->data, AUTOPID_BUFFER_SIZE, "error: missing frames");
        }
        
        response->length = strlen((char *)response->data);
        
        if (lowest_header_data != NULL) {
            free(lowest_header_data);
            lowest_header_data = NULL;
        }
        response->priority_data = NULL;
        response->priority_data_len = 0;
        
        // FREE THE HEAP POINTER BEFORE RETURNING
        if (original_buffer != NULL) {
            free(original_buffer);
        }
        return;
    }
    // --------------------------------------------

    // Clean up heap pointer if parsing succeeded
    if (original_buffer != NULL) {
        free(original_buffer);
    }

    response->length = k;

    if (frame_count <= 2 || all_headers_same)
    {
        response->priority_data = NULL;
        response->priority_data_len = 0;
        if (lowest_header_data != NULL)
        {
            free(lowest_header_data);
            lowest_header_data = NULL;
        }
    }
    else
    {
        response->priority_data = lowest_header_data;
        response->priority_data_len = lowest_header_length;
    }
}
  

static void append_to_buffer(char *buffer, const char *new_data)
{
    if (strlen(buffer) + strlen(new_data) < AUTOPID_BUFFER_SIZE)
    {
        strcat(buffer, new_data);
    }
    else
    {
        ESP_LOGE(TAG, "Failed add data to buffer");
    }
}

static bool autopid_buf_is_only_stopped(const char *buf)
{
    if (!buf)
        return true;

    const char *p = buf;
    while (*p)
    {
        // Skip whitespace and separators
        // Also skip ELM prompt characters '>' that can be appended into the same buffer.
        while (*p == '\r' || *p == '\n' || *p == ' ' || *p == '\t' || *p == '>')
            p++;
        if (*p == '\0')
            break;

        // Find end of token/line
        const char *start = p;
        while (*p && *p != '\r' && *p != '\n')
            p++;
        size_t len = (size_t)(p - start);

        // Trim trailing spaces
        while (len > 0 && (start[len - 1] == ' ' || start[len - 1] == '\t' || start[len - 1] == '>'))
            len--;

        if (len == 0)
            continue;

        // Ignore bare prompt token
        if (len == 1 && start[0] == '>')
            continue;

        // ELM/STN chips often emit a lone "STOPPED" line when ATMA is terminated.
        if (!((len == 7 && strncmp(start, "STOPPED", 7) == 0) ||
              (len == 7 && strncmp(start, "stopped", 7) == 0)))
        {
            return false;
        }
    }

    return true;
}

// -----------------------
// ATMA (monitor-all) parsing
// -----------------------

static uint32_t autopid_atma_expected_frame_id = 0;
static bool autopid_atma_filter_enabled = false;

static void autopid_atma_set_expected_frame_id(uint32_t frame_id)
{
    autopid_atma_expected_frame_id = frame_id;
    autopid_atma_filter_enabled = (frame_id != 0);
}

static bool autopid_is_hex_n(const char *s, size_t n)
{
    if (!s || n == 0)
        return false;
    for (size_t i = 0; i < n; i++)
    {
        if (!isxdigit((unsigned char)s[i]))
            return false;
    }
    return true;
}

static bool autopid_hex_byte(const char *s, uint8_t *out)
{
    if (!s || !out)
        return false;
    if (!autopid_is_hex_n(s, 2))
        return false;
    char tmp[3] = {s[0], s[1], 0};
    char *endptr = NULL;
    long v = strtol(tmp, &endptr, 16);
    if (endptr == tmp)
        return false;
    if (v < 0 || v > 255)
        return false;
    *out = (uint8_t)v;
    return true;
}

static bool autopid_atma_parse_line(const char *line, size_t line_len, uint32_t *out_header, response_t *out_rsp)
{
    if (!line || line_len == 0 || !out_header || !out_rsp)
        return false;

    // Trim leading/trailing whitespace
    while (line_len > 0 && (line[0] == ' ' || line[0] == '\t'))
    {
        line++;
        line_len--;
    }
    while (line_len > 0 && (line[line_len - 1] == ' ' || line[line_len - 1] == '\t'))
        line_len--;
    if (line_len == 0)
        return false;

    // Ignore prompt / terminator lines
    if (line_len == 1 && line[0] == '>')
        return false;
    if (line_len >= 7 && (strncasecmp(line, "STOPPED", 7) == 0))
        return false;
    if (line_len >= 5 && (strncasecmp(line, "ERROR", 5) == 0))
        return false;
    if (line_len >= 7 && (strncasecmp(line, "NO DATA", 7) == 0))
        return false;

    // Tokenize by whitespace (in-place not allowed, so we scan)
    const char *p = line;
    const char *end = line + line_len;

    // Read first token
    while (p < end && (*p == ' ' || *p == '\t'))
        p++;
    const char *t1 = p;
    while (p < end && *p != ' ' && *p != '\t')
        p++;
    size_t t1_len = (size_t)(p - t1);
    if (t1_len == 0)
        return false;

    uint32_t header = 0;
    bool header_ok = false;

    // Case 1: header is contiguous hex: "123" (11-bit) or "000008C0" (29-bit)
    if ((t1_len == 3 || t1_len == 8) && autopid_is_hex_n(t1, t1_len))
    {
        char tmp[9] = {0};
        memcpy(tmp, t1, t1_len);
        header = (uint32_t)strtoul(tmp, NULL, 16);
        header_ok = true;
    }
    else if (t1_len == 2 && autopid_is_hex_n(t1, 2))
    {
        // Case 2: header is split into bytes: "00 00 08 C0" (extended)
        // or (less common) "01 23" (standard with leading zero).
        uint8_t b1 = 0, b2 = 0, b3 = 0, b4 = 0;
        const char *t2 = NULL, *t3 = NULL, *t4 = NULL;
        size_t t2_len = 0, t3_len = 0, t4_len = 0;

        // token 2
        while (p < end && (*p == ' ' || *p == '\t'))
            p++;
        t2 = p;
        while (p < end && *p != ' ' && *p != '\t')
            p++;
        t2_len = (size_t)(p - t2);

        // token 3
        while (p < end && (*p == ' ' || *p == '\t'))
            p++;
        t3 = p;
        while (p < end && *p != ' ' && *p != '\t')
            p++;
        t3_len = (size_t)(p - t3);

        // token 4
        while (p < end && (*p == ' ' || *p == '\t'))
            p++;
        t4 = p;
        while (p < end && *p != ' ' && *p != '\t')
            p++;
        t4_len = (size_t)(p - t4);

        // Try 4-byte extended header first (only accept if it matches expected filter when enabled)
        if (t2_len == 2 && t3_len == 2 && t4_len == 2 &&
            autopid_hex_byte(t1, &b1) && autopid_hex_byte(t2, &b2) &&
            autopid_hex_byte(t3, &b3) && autopid_hex_byte(t4, &b4))
        {
            uint32_t hdr32 = ((uint32_t)b1 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 8) | (uint32_t)b4;
            if (!autopid_atma_filter_enabled || hdr32 == autopid_atma_expected_frame_id)
            {
                header = hdr32;
                header_ok = true;
            }
        }

        // If not a matching 4-byte header, try 2-byte standard header (only if it matches expected)
        if (!header_ok && t2_len == 2)
        {
            if (autopid_hex_byte(t1, &b1) && autopid_hex_byte(t2, &b2))
            {
                uint32_t hdr16 = ((uint32_t)b1 << 8) | (uint32_t)b2;
                if ((!autopid_atma_filter_enabled && hdr16 <= 0x7FF) ||
                    (autopid_atma_filter_enabled && hdr16 == autopid_atma_expected_frame_id))
                {
                    header = hdr16;
                    header_ok = true;
                }
            }
        }
    }

    if (!header_ok)
        return false;

    // Filter out unrelated frames (important when frames are buffered before ATCRA takes effect)
    if (autopid_atma_filter_enabled && header != autopid_atma_expected_frame_id)
        return false;

    // Parse remaining tokens as data bytes
    memset(out_rsp, 0, sizeof(*out_rsp));
    out_rsp->priority_data = NULL;
    out_rsp->priority_data_len = 0;

    uint32_t out_idx = 0;
    while (p < end)
    {
        while (p < end && (*p == ' ' || *p == '\t'))
            p++;
        const char *bt = p;
        while (p < end && *p != ' ' && *p != '\t')
            p++;
        size_t bt_len = (size_t)(p - bt);
        if (bt_len == 0)
            continue;
        if (bt_len == 1 && bt[0] == '>')
            break;
        if (bt_len != 2 || !autopid_is_hex_n(bt, 2))
            continue;
        if (out_idx >= sizeof(out_rsp->data))
            break;
        uint8_t b = 0;
        if (!autopid_hex_byte(bt, &b))
            continue;
        out_rsp->data[out_idx++] = b;
    }

    out_rsp->length = out_idx;
    if (out_rsp->length == 0)
        return false;

    *out_header = header;
    return true;
}

static char autopid_atma_line_buf[192];
static size_t autopid_atma_line_pos = 0;

static void autopid_atma_parser_reset(void)
{
    autopid_atma_line_pos = 0;
    autopid_atma_line_buf[0] = '\0';
}

void autopid_atma_parser(char *str, uint32_t len, QueueHandle_t *q, char *cmd_str)
{
    (void)cmd_str;
    if (!str || len == 0 || !q)
        return;

    for (uint32_t i = 0; i < len; i++)
    {
        char c = str[i];
        if (c == '\r' || c == '\n')
        {
            if (autopid_atma_line_pos > 0)
            {
                // Process one complete line
                uint32_t header = 0;

		// --- FIXED: Use static heap allocation instead of stack ---
                static response_t *rsp = NULL;
                if (!rsp) rsp = heap_caps_malloc(sizeof(response_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		
                if (rsp && autopid_atma_parse_line(autopid_atma_line_buf, autopid_atma_line_pos, &header, rsp))
                {
                    if (xQueueSend(*q, rsp, pdMS_TO_TICKS(20)) != pdPASS)
                    {
                        // Drop if queue is full; monitoring is time-sliced.
                    }
                }
                autopid_atma_line_pos = 0;
            }
        }
        else
        {
            if (autopid_atma_line_pos + 1 < sizeof(autopid_atma_line_buf))
            {
                autopid_atma_line_buf[autopid_atma_line_pos++] = c;
                autopid_atma_line_buf[autopid_atma_line_pos] = '\0';
            }
            else
            {
                // Line too long; reset to avoid runaway.
                autopid_atma_line_pos = 0;
            }
        }
    }
}

void autopid_parser(char *str, uint32_t len, QueueHandle_t *q, char *cmd_str)
{
    static response_t *response = NULL;

    if (response == NULL)
    {
        response = (response_t *)heap_caps_malloc(sizeof(response_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }

    if (response == NULL)
        return;

    if (str != NULL && strlen(str) != 0)
    {
        ESP_LOGI(TAG, "%s", str);

        append_to_buffer(auto_pid_buf, str);

        if (strchr(str, '>') != NULL)
        {
            // If we only got the ATMA terminator, ignore it.
            if (autopid_buf_is_only_stopped(auto_pid_buf))
            {
                auto_pid_buf[0] = '\0';
                return;
            }

            // --- NEW: Wipe stale pointers before we parse anything! ---
            response->priority_data = NULL;
            response->priority_data_len = 0;
            response->length = 0;
            memset(response->data, 0, AUTOPID_BUFFER_SIZE);
            // ----------------------------------------------------------

            if (strstr(str, "NO DATA") == NULL && strstr(str, "ERROR") == NULL)
            {
                // Parse the accumulated buffer
                parse_elm327_response(auto_pid_buf, response);

                // If parsing produced no bytes (e.g. STOPPED, SEARCHING..., echoed command),
                // mark as error so the caller won't try to decode it as a valid PID payload.
                if (response->length == 0)
                {
                    strcpy((char *)response->data, "error");
                    response->length = strlen((char *)response->data);
                }

                if (xQueueSend(autopidQueue, response, pdMS_TO_TICKS(1000)) != pdPASS)
                {
                    ESP_LOGE(TAG, "Failed to send to queue");
		    // --- NEW: FREE IF QUEUE FULL ---
                    if (response->priority_data != NULL) {
                        free(response->priority_data);
                        response->priority_data = NULL;
                    }
                }
            }
            else
            {
                sprintf((char *)response->data, "error");
                response->length = strlen((char *)response->data);
                ESP_LOGE(TAG, "Error response: %s", auto_pid_buf);
                // kww HARD FIX: Ensure string buffer is cleared on error frames as well
                auto_pid_buf[0] = '\0';
		
                if (xQueueSend(autopidQueue, response, pdMS_TO_TICKS(1000)) != pdPASS)
                {
                    ESP_LOGE(TAG, "Failed to send to queue");
		    // --- NEW: FREE IF QUEUE FULL ---
                    if (response->priority_data != NULL) {
                        free(response->priority_data);
                        response->priority_data = NULL;
                    }
                }
            }
            // Clear the buffer after parsing
            auto_pid_buf[0] = '\0';
        }
    }
}

static bool try_parse_protocol_cmd(const char *cmd, int32_t *out_protocol)
{
    if (!cmd || !out_protocol)
        return false;

    const unsigned char *p = (const unsigned char *)cmd;
    while (*p && isspace(*p))
        p++;

    if (tolower(p[0]) != 'a' || tolower(p[1]) != 't')
        return false;

    p += 2;
    while (*p && isspace(*p))
        p++;

    if (tolower(*p) == 's')
    {
        p++;
        while (*p && isspace(*p))
            p++;
        if (tolower(*p) != 'p')
            return false;
        p++;
    }
    else if (tolower(*p) == 't')
    {
        // ATTP: accept both "ATTP" and whitespace-separated forms like "AT TP".
        // After the first 'T', we allow either immediate/space-separated 'P' (TP)
        // or another 'T' then 'P' (TTP).
        p++;
        while (*p && isspace(*p))
            p++;
        if (tolower(*p) == 'p')
        {
            p++;
        }
        else
        {
            if (tolower(*p) != 't')
                return false;
            p++;
            while (*p && isspace(*p))
                p++;
            if (tolower(*p) != 'p')
                return false;
            p++;
        }
    }
    else
    {
        return false;
    }

    while (*p && isspace(*p))
        p++;

    if (!isxdigit(*p))
        return false;

    char hex_buf[3] = {0};
    int hi = 0;
    while (*p && isxdigit(*p) && hi < 2)
    {
        hex_buf[hi++] = (char)*p;
        p++;
    }

    char *endptr = NULL;
    long v = strtol(hex_buf, &endptr, 16);
    if (endptr == hex_buf)
        return false;

    *out_protocol = (int32_t)v;
    return true;
}

static void send_commands(char *commands, uint32_t delay_ms)
{
    if (!commands) return;

    char *cmd_start = commands;
    
    // Loop until we reach the end of the string
    while (*cmd_start != '\0')
    {
        // Skip leading spaces, \r, \n, or semicolons
        while (*cmd_start == ' ' || *cmd_start == '\r' || *cmd_start == '\n' || *cmd_start == ';') {
            cmd_start++;
        }
        if (*cmd_start == '\0') break;

        // Find the end of this command (marked by \r, \n, ;, or end of string)
        char *cmd_end = cmd_start;
        while (*cmd_end != '\0' && *cmd_end != '\r' && *cmd_end != '\n' && *cmd_end != ';') {
            cmd_end++;
        }

        size_t cmd_len = cmd_end - cmd_start;
        if (cmd_len == 0) {
            cmd_start = cmd_end;
            continue;
        }

        // Create a temporary buffer for the command and explicitly append \r
        char str_send[cmd_len + 2]; 
        strncpy(str_send, cmd_start, cmd_len);
        str_send[cmd_len] = '\r';   // ELM327 requires \r
        str_send[cmd_len + 1] = '\0'; 

        // Keep protocol tracking in sync if init strings switch protocol.
        int32_t new_protocol_number = -1;
        bool skip_send = false;
        if (try_parse_protocol_cmd(str_send, &new_protocol_number))
        {
            int32_t current_protocol_number = -1;
            if (autopid_get_protocol_number(&current_protocol_number) == ESP_OK &&
                current_protocol_number == new_protocol_number)
            {
                skip_send = false;
            }
            else
            {
                autopid_set_protocol_number(new_protocol_number);
            }
        }

        if (!skip_send &&
            (strstr(str_send, "ath0") == NULL && strstr(str_send, "ATH0") == NULL && strstr(str_send, "at h0") == NULL && strstr(str_send, "AT H0") == NULL) &&
            (strstr(str_send, "ats0") == NULL && strstr(str_send, "ATS0") == NULL && strstr(str_send, "at s0") == NULL && strstr(str_send, "AT s0") == NULL) &&
            (strstr(str_send, "ate1") == NULL && strstr(str_send, "ATE1") == NULL && strstr(str_send, "at e1") == NULL && strstr(str_send, "AT E1") == NULL))
        {
#if HARDWARE_VER == WICAN_PRO
            // RESTORED: Use the stable elm327_process_cmd. 
            // elm327_run_command causes FreeRTOS spinlock panics due to pointer mismatch.
            elm327_process_cmd((uint8_t *)str_send, cmd_len + 1, &autopidQueue, elm327_autopid_cmd_buffer, &elm327_autopid_cmd_buffer_len, &elm327_autopid_last_cmd_time, autopid_parser);
#else
            twai_message_t tx_msg;
            elm327_process_cmd((uint8_t *)str_send, cmd_len + 1, &tx_msg, &autopidQueue);
#endif
        }

        cmd_start = cmd_end; 
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}



char* autopid_process_raw_expression(uint8_t *data, uint32_t data_len, bool is_dtc) {

    if (!data || data_len == 0) return NULL;

    // --- NEW: SMUGGLE OUR CUSTOM ERROR STRING THROUGH ---
    if (strstr((char *)data, "error: missing frames") != NULL) {
        char *error_str = heap_caps_malloc(data_len + 1, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
        if (error_str) {
            strncpy(error_str, (char *)data, data_len);
            error_str[data_len] = '\0';
            return error_str;
        }
    }
    // ----------------------------------------------------
    
    // 1. If user explicitly requested DTC_RAW, run the formatting
    if (is_dtc) {
        uint8_t total_dtcs = 0;
        uint8_t dtc_buffer[128]; 
        int dtc_idx = 0;

        for (uint32_t i = 0; i < data_len; i++) {
            if (data[i] == 0x43 && (i + 1) < data_len) {
                uint8_t count = data[i+1];
                total_dtcs += count;
                int dtc_bytes = count * 2; 
                
                if ((i + 1 + dtc_bytes < data_len) && (dtc_idx + dtc_bytes < sizeof(dtc_buffer))) {
                    for (int j = 0; j < dtc_bytes; j++) {
                        dtc_buffer[dtc_idx++] = data[i + 2 + j];
                    }
                    i += 1 + dtc_bytes; 
                }
            }
        }

        char *master_string = heap_caps_malloc(4 + (dtc_idx * 2) + 1, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
        if (master_string) {
            sprintf(master_string, "43%02X", total_dtcs);
            for (int b = 0; b < dtc_idx; b++) {
                sprintf(&master_string[4 + (b * 2)], "%02X", dtc_buffer[b]);
            }
            return master_string;
        }
    }
    
    // 3. UNIVERSAL RAW DUMP (Mode 22, ISO-TP, etc.)
    else {
        // --- THE ISO-TP CLEANER ---
        if (data_len > 8 && data[0] == 0x10) {
            uint8_t clean_data[256]; // Ensures buffer is large enough
            uint32_t clean_len = 0;
            uint32_t i = 2; // Skip byte 0 (0x10 First Frame) and byte 1 (Length)
            
            // Read the data payload of the first frame (up to 6 bytes)
            while (i < 8 && i < data_len && clean_len < sizeof(clean_data)) {
                clean_data[clean_len++] = data[i++];
            }
            
            // Read the consecutive frames
            while (i < data_len && clean_len < sizeof(clean_data)) {
                // Check if the current byte is a sequence byte (0x21, 0x22, etc.)
                if ((data[i] & 0xF0) == 0x20) { 
                    i++; // Skip the sequence byte!
                    
                    // The next 7 bytes are pure data
                    uint32_t chunk_end = i + 7; 
                    while (i < chunk_end && i < data_len && clean_len < sizeof(clean_data)) {
                        clean_data[clean_len++] = data[i++];
                    }
                } else {
                    // Fallback just in case formatting is misaligned
                    clean_data[clean_len++] = data[i++];
                }
            }
            
            // Convert the beautifully cleaned array into a Hex String
            char *master_string = heap_caps_malloc((clean_len * 2) + 1, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
            if (master_string) {
                for (uint32_t j = 0; j < clean_len; j++) {
                    sprintf(&master_string[j * 2], "%02X", clean_data[j]);
                }
                return master_string;
            }
        } 
        // --- STANDARD SINGLE-FRAME DUMP ---
        else {
            char *master_string = heap_caps_malloc((data_len * 2) + 1, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
            if (master_string) {
                for (uint32_t i = 0; i < data_len; i++) {
                    sprintf(&master_string[i * 2], "%02X", data[i]);
                }
                return master_string;
            }
        }
    }
    
    return NULL; // Return NULL if allocation or parsing fails
}


static void execute_pid(pid_data_t *curr_pid, bool check_timers) {
    if (!curr_pid || curr_pid->cmd == NULL || strlen(curr_pid->cmd) == 0) return;

    // --- FIXED: Allocate response on PSRAM Heap, not Stack ---
    response_t *elm327_response = heap_caps_malloc(sizeof(response_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!elm327_response) return;

    // Flush Queue using our heap pointer
    while (xQueueReceive(autopidQueue, elm327_response, 0) == pdPASS) {
        if (elm327_response->priority_data != NULL) {
            free(elm327_response->priority_data);
        }
    }

    // HARD FIX: Flush the residual string accumulation buffers to prevent cross-talk
    if (auto_pid_buf != NULL) {
        auto_pid_buf[0] = '\0';
    } 
    #if HARDWARE_VER == WICAN_PRO
    if (elm327_autopid_cmd_buffer != NULL) {
        elm327_autopid_cmd_buffer[0] = '\0';
    }
    #endif

    ESP_LOGI(TAG, "Executing command: %s", curr_pid->cmd);

    #if HARDWARE_VER == WICAN_PRO
    if (elm327_process_cmd((uint8_t *)curr_pid->cmd, strlen(curr_pid->cmd), &autopidQueue, elm327_autopid_cmd_buffer, &elm327_autopid_cmd_buffer_len, &elm327_autopid_last_cmd_time, autopid_parser) == ESP_OK)
    #else
    twai_message_t tx_msg;
    if (elm327_process_cmd((uint8_t *)curr_pid->cmd, strlen(curr_pid->cmd), &tx_msg, &autopidQueue) == ESP_OK)
    #endif
    {
        memset(elm327_response, 0, sizeof(response_t));

        xSemaphoreGive(autopid_config->mutex);
        BaseType_t got_response = xQueueReceive(autopidQueue, elm327_response, pdMS_TO_TICKS(1000));
        xSemaphoreTake(autopid_config->mutex, portMAX_DELAY);

        if (got_response == pdPASS) {

            if (strstr((char *)elm327_response->data, "error") == NULL &&
                strstr((char *)elm327_response->data, "SEARCHING") == NULL &&
                strstr((char *)elm327_response->data, "UNABLE TO CONNECT") == NULL) {


                    // ====================================================================
                    // NEW: UNIVERSAL STRUCTURAL DID VALIDATION & ALIGNMENT
                    // ====================================================================
                    if (strncmp(curr_pid->cmd, "22", 2) == 0 && strlen(curr_pid->cmd) >= 6) {
                        
                        // Extract expected DID safely
                        char hex1[3] = {curr_pid->cmd[2], curr_pid->cmd[3], '\0'};
                        char hex2[3] = {curr_pid->cmd[4], curr_pid->cmd[5], '\0'};
                        
                        uint8_t exp_b1 = (uint8_t)strtol(hex1, NULL, 16);
                        uint8_t exp_b2 = (uint8_t)strtol(hex2, NULL, 16);

                        int payload_start = -1;

                        // Scan the first 6 bytes to bypass ISO-TP headers (10 XX) or concatenated NRC stall frames (7F 22 78)
                        for (int b = 0; b < 6 && b <= elm327_response->length - 3; b++) {
                            if (elm327_response->data[b] == 0x62 && 
                                elm327_response->data[b+1] == exp_b1 && 
                                elm327_response->data[b+2] == exp_b2) {
                                
                                // Found the DID! Now look backwards to find the true start of the CAN frame
                                if (b == 0) {
                                    payload_start = 0; 
                                } 
                                else if (b == 1 && elm327_response->data[0] <= 0x07) {
                                    payload_start = 0; // Single frame PCI byte is at index 0
                                }
                                else if (b >= 1 && elm327_response->data[b-1] <= 0x07) {
                                    payload_start = b - 1; // Single frame PCI byte
                                }
                                else if (b >= 2 && elm327_response->data[b-2] >= 0x10 && elm327_response->data[b-2] <= 0x1F) {
                                    payload_start = b - 2; // ISO-TP First Frame (10 XX)
                                } 
                                else {
                                    payload_start = b; // Fallback alignment
                                }
                                break;
                            }
                        }

                        if (payload_start == -1) {
                            ESP_LOGE(TAG, "CROSSTALK REJECTED: DID %02X%02X not found. Dropping.", exp_b1, exp_b2);
                            if (elm327_response->priority_data != NULL) free(elm327_response->priority_data);
                            free(elm327_response);
                            return; // Drop bad frame silently
                        }

                        // Shift the array to erase concatenated 7F 22 78 frames, restoring pure payload structure
                        if (payload_start > 0) {
                            elm327_response->length -= payload_start;
                            memmove(elm327_response->data, &elm327_response->data[payload_start], elm327_response->length);
                            memset(&elm327_response->data[elm327_response->length], 0, AUTOPID_BUFFER_SIZE - elm327_response->length);
                        }
                    }
                    // ====================================================================

	      
                xEventGroupSetBits(xautopid_event_group, ECU_CONNECTED_BIT);
                autopid_config->last_successful_pid_time = time(NULL);
		
                // ---> LOOP THROUGH PARAMETERS USING THE SHARED RESPONSE <---
                for (uint32_t p = 0; p < curr_pid->parameters_count; p++) {
                    parameter_t *param = &curr_pid->parameters[p];
                    if (!param->enabled) continue;

                    // If we are in Legacy mode, respect the individual parameter timers
                    if (check_timers) {
                        if (!wc_timer_is_expired(&param->timer)) continue;
                        wc_timer_set(&param->timer, param->period);
                    }

                    param->failed = false;

                    // 1. Custom / Specific PID Logic
                    if (curr_pid->pid_type == PID_CUSTOM || curr_pid->pid_type == PID_SPECIFIC) {

                        // Apply the RAW / DTC_RAW fix we just mapped out!
                        if (param->expression && (strcasecmp(param->expression, "RAW") == 0 || strcasecmp(param->expression, "DTC_RAW") == 0)) {
                            if (param->raw_string_value) {
                                free(param->raw_string_value);
                                param->raw_string_value = NULL;
                            }
                            
                            bool is_dtc = (strcasecmp(param->expression, "DTC_RAW") == 0);
                            param->raw_string_value = autopid_process_raw_expression(elm327_response->data, elm327_response->length, is_dtc);
                            
                            if (param->raw_string_value) publish_parameter_mqtt(param);
                            else param->failed = true;
                            
                        } else {
                            // Standard Math Evaluation
                            double result;
                            if (evaluate_expression((uint8_t *)param->expression, (uint8_t *)elm327_response->data, 0, &result)) {
                                if (param->min != FLT_MAX && result < param->min) { } 
                                else if (param->max != FLT_MAX && result > param->max) { } 
                                else {
                                    param->value = (float)(round(result * 100.0) / 100.0);
				    autopid_pid_update_hook(param, param->value);
                                    autopid_update_supply_mode_state_from_param(param);
                                    publish_parameter_mqtt(param);
                                }
                            } else { param->failed = true; }
                        }
                    }
                    // 2. Standard PID Logic
                    else if (curr_pid->pid_type == PID_STD) {
                        const std_pid_t *pid_info = get_pid_from_string(param->name);
                        if (pid_info) {
                            for (int pi = 0; pi < pid_info->num_params; pi++) {
                                const char *param_name = strchr(param->name, '-');
                                if (param_name && strcmp(param_name + 1, pid_info->params[pi].name) == 0) {
                                    esp_err_t err;
                                   if (elm327_response->priority_data != NULL) {
                                      err = extract_signal_value(elm327_response->priority_data, elm327_response->priority_data_len, &pid_info->params[pi], &param->value);
                                   } else {
                                      err = extract_signal_value(elm327_response->data, elm327_response->length, &pid_info->params[pi], &param->value);
                                   }
                                    if (err == ESP_OK) {
                                        param->value = roundf(param->value * 100.0) / 100.0;
					autopid_pid_update_hook(param, param->value);
                                        autopid_update_supply_mode_state_from_param(param);
                                        publish_parameter_mqtt(param);
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
            } else {
                // ECU Responded with ERROR (Mark due parameters as failed)
                for (uint32_t p = 0; p < curr_pid->parameters_count; p++) {
                    parameter_t *param = &curr_pid->parameters[p];
                    if (param->enabled && (!check_timers || wc_timer_is_expired(&param->timer))) {
                        param->failed = true;

                        // buffer fix --- RESTORED FIX: Clear Ghost RPMs and Stale Strings ---
                        param->value = 0; 
                        if (param->raw_string_value != NULL) {
                            free(param->raw_string_value);
                            param->raw_string_value = NULL;
                        }
                        // buffer fix --------------------------------------------------------
			
                        if (check_timers) wc_timer_set(&param->timer, param->period);
                    }
                }
                ESP_LOGE(TAG, "ELM Response Error for %s", curr_pid->cmd);
            }
        } else {
            // Queue Timeout (Mark due parameters as failed)
            for (uint32_t p = 0; p < curr_pid->parameters_count; p++) {
                parameter_t *param = &curr_pid->parameters[p];
                if (param->enabled && (!check_timers || wc_timer_is_expired(&param->timer))) {
                    param->failed = true;

                    // buffer fix --- RESTORED FIX: Clear Ghost RPMs and Stale Strings ---
                    param->value = 0; 
                    if (param->raw_string_value != NULL) {
                        free(param->raw_string_value);
                        param->raw_string_value = NULL;
                    }
                    // buffer fix --------------------------------------------------------
		    
                    if (check_timers) wc_timer_set(&param->timer, param->period);
                }
            }
            ESP_LOGE(TAG, "Queue Timeout for %s", curr_pid->cmd);
        }
        // --- NEW: PLUG THE MEMORY LEAK ---
        if (elm327_response->priority_data != NULL) {
          free(elm327_response->priority_data);
        }
    } else {
        ESP_LOGE(TAG, "Process Cmd Failed: %s", curr_pid->cmd);
    }
    
    // FREE THE HEAP POINTER!
    free(elm327_response);
}
		     
static bool all_parameters_failed(autopid_config_t *autopid_config)
{
    if (!autopid_config)
        return true;

    bool any_success = false;

    xSemaphoreTake(autopid_config->mutex, portMAX_DELAY);
    for (uint32_t i = 0; i < autopid_config->pid_count; i++)
    {
        pid_data_t *curr_pid = &autopid_config->pids[i];
        for (uint32_t p = 0; p < curr_pid->parameters_count; p++)
        {
            if (!curr_pid->parameters[p].failed)
            {
                any_success = true;
                break;
            }
        }
        if (any_success)
            break;
    }
    xSemaphoreGive(autopid_config->mutex);

    return !any_success;
}

static void publish_parameter_mqtt(parameter_t *param)
{
    if (!param)
        return;

    // ---> BYPASS FOR BULK GROUPS <---
    // MQTT_Grp parameters are aggregated and published at the end of the group loop!
    if (param->destination_type == DEST_MQTT_GRP) {
        return;
    }

    // ---> BYPASS FOR BULK DEFAULT <---
    // If JSON Data Grouping is enabled, default parameters are sent in one giant payload!
    if (param->destination_type == DEST_DEFAULT && autopid_config && autopid_config->grouping && strcmp("enable", autopid_config->grouping) == 0) {
        return;
    }

    // Nothing can be delivered while MQTT is offline. Return before touching the
    // "last sent" history, so on-change values are not marked as sent and lost.
    // The latest values are published when MQTT reconnects (see
    // autopid_publish_last_known).
    if (config_server_mqtt_en_config() != 1 || !mqtt_connected()) {
        return;
    }

    // Filter: If "onchange" is set, only proceed if value changed
    if (param->onchange)
    {
        // ---> ADD STRING ONCHANGE LOGIC <---
        if (param->raw_string_value != NULL) {
            if (param->last_sent_raw_string && strcmp(param->raw_string_value, param->last_sent_raw_string) == 0) {
                return; // SKIP PUBLISH
            }
        } else {
            if (param->value == param->last_sent_value) {
                return; // SKIP PUBLISH
            }
        }
    }
    
    // Update history only if we are about to send
    param->last_sent_value = param->value;
    
    // ---> ADD STRING HISTORY UPDATE <---
    if (param->raw_string_value != NULL) {
        if (param->last_sent_raw_string) free(param->last_sent_raw_string);
        param->last_sent_raw_string = strdup_psram(param->raw_string_value);
    }
    
    char *payload = NULL;
    
    switch (param->destination_type)
    {
     case DEST_DEFAULT:
     case DEST_MQTT_GRP:
     case DEST_MQTT_TOPIC:
        // JSON format
        {
            cJSON *param_json = cJSON_CreateObject();
            if (param_json)
            {
                // ---> ADD STRING PAYLOAD LOGIC <---
                if (param->raw_string_value != NULL) {
                    cJSON_AddStringToObject(param_json, param->name, param->raw_string_value);
                }
                else if (param->sensor_type == BINARY_SENSOR) {
                    cJSON_AddStringToObject(param_json, param->name, param->value > 0 ? "on" : "off");
                } else {
                    cJSON_AddNumberToObject(param_json, param->name, param->value);
                }
                
                limitJsonDecimalPrecision(param_json);
                
                // Add the timestamp BEFORE we print and delete
                if (config_server_get_mqtt_include_timestamp() == 1) {
                    cJSON_AddNumberToObject(param_json, "timestamp", (double)time(NULL));
                }
                
                // Print to payload
                payload = cJSON_PrintUnformatted(param_json);
                
                // Delete the JSON object EXACTLY ONCE
                cJSON_Delete(param_json);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to create JSON object for MQTT payload");
            }
        }
        break;

    case DEST_MQTT_WALLBOX:
        // ---> ADD WALLBOX STRING LOGIC <---
        if (param->raw_string_value != NULL) {
            asprintf(&payload, "%s", param->raw_string_value);
        } else {
            asprintf(&payload, "%.2f", param->value);
        }
        break;

    default:
        break;
    }

    if (payload)
    {
        // ---> NEW GROUP ROUTING LOGIC <---
        if (param->destination_type == DEST_MQTT_GRP) {
            if (current_group_mqtt_topic && strlen(current_group_mqtt_topic) > 0) {
                mqtt_publish(current_group_mqtt_topic, payload, 0, 0, 1);
                ESP_LOGI(TAG, "Published to group topic %s", current_group_mqtt_topic);
            } else {
                ESP_LOGW(TAG, "MQTT_Grp selected but no group topic defined");
            }
        }
        // ---> EXISTING ROUTING <---
        else if (param->destination && strlen(param->destination) > 0)
        {
            mqtt_publish(param->destination, payload, 0, 0, 1);
            ESP_LOGI(TAG, "Published to %s", param->destination);
        }
        else
        {
            mqtt_publish(config_server_get_mqtt_rx_topic(), payload, 0, 0, 1);
        }
        free(payload);
    }
}

// ==========================================================================================
// PUBLISH LAST-KNOWN DATA ON MQTT (RE)CONNECT
// ------------------------------------------------------------------------------------------
// While the device has no network (e.g. driving away from home Wi-Fi) MQTT publishes are
// dropped. As soon as MQTT connects again, publish the most recent value of every
// MQTT-bound parameter so the broker (and Home Assistant) gets current data straight away,
// even if the car is already off and nothing new can be polled.
// ==========================================================================================

static bool param_has_value(const parameter_t *param)
{
    return param && param->enabled && param->name &&
           (param->raw_string_value != NULL || param->value != FLT_MAX);
}

static void add_param_value_to_json(cJSON *obj, const parameter_t *param)
{
    if (param->raw_string_value != NULL) {
        cJSON_AddStringToObject(obj, param->name, param->raw_string_value);
    } else if (param->sensor_type == BINARY_SENSOR) {
        cJSON_AddStringToObject(obj, param->name, param->value > 0 ? "on" : "off");
    } else {
        cJSON_AddNumberToObject(obj, param->name, param->value);
    }
}

static double last_known_timestamp(void)
{
    // Prefer the time the data was actually captured; fall back to now if the
    // clock was not synced at capture time (pre-2020).
    if (autopid_data_updated_at > 1577836800) {
        return (double)autopid_data_updated_at;
    }
    return (double)time(NULL);
}

static void mark_param_sent(parameter_t *param)
{
    param->last_sent_value = param->value;
    if (param->raw_string_value != NULL) {
        if (param->last_sent_raw_string) free(param->last_sent_raw_string);
        param->last_sent_raw_string = strdup_psram(param->raw_string_value);
    }
}

// Publishes and frees obj. Returns true if something was handed to MQTT.
static bool publish_last_known_json(const char *topic, cJSON *obj)
{
    bool sent = false;
    if (obj && obj->child && topic && topic[0] != '\0') {
        limitJsonDecimalPrecision(obj);
        if (config_server_get_mqtt_include_timestamp() == 1) {
            cJSON_AddNumberToObject(obj, "timestamp", last_known_timestamp());
        }
        char *payload = cJSON_PrintUnformatted(obj);
        if (payload) {
            mqtt_publish(topic, payload, 0, 0, 1);
            free(payload);
            sent = true;
        }
    }
    if (obj) cJSON_Delete(obj);
    return sent;
}

static bool grouping_enabled(void)
{
    return autopid_config->grouping && strcmp("enable", autopid_config->grouping) == 0;
}

// Individual-topic parameters (MQTT_Topic / WallBox, and Default when JSON grouping is off).
// Groups' MQTT_Grp params are handled by the caller; with grouping on, Default params are
// covered by the snapshot.
static uint32_t publish_last_known_param(parameter_t *param)
{
    if (!param_has_value(param)) return 0;
    if (param->destination_type == DEST_DEFAULT && grouping_enabled()) return 0;

    const char *topic = (param->destination && param->destination[0] != '\0')
                            ? param->destination
                            : config_server_get_mqtt_rx_topic();

    if (param->destination_type == DEST_MQTT_TOPIC || param->destination_type == DEST_DEFAULT) {
        cJSON *obj = cJSON_CreateObject();
        if (!obj) return 0;
        add_param_value_to_json(obj, param);
        if (publish_last_known_json(topic, obj)) {
            mark_param_sent(param);
            return 1;
        }
    } else if (param->destination_type == DEST_MQTT_WALLBOX) {
        char *payload = NULL;
        if (param->raw_string_value != NULL) {
            asprintf(&payload, "%s", param->raw_string_value);
        } else {
            asprintf(&payload, "%.2f", param->value);
        }
        if (payload) {
            mqtt_publish(topic, payload, 0, 0, 1);
            free(payload);
            mark_param_sent(param);
            return 1;
        }
    }
    return 0;
}

static bool pid_is_in_any_group(const pid_data_t *pid)
{
    if (!autopid_config->use_groups) return false;
    for (uint32_t g = 0; g < autopid_config->group_count; g++) {
        for (uint32_t gp = 0; gp < autopid_config->groups[g].pid_count; gp++) {
            if (autopid_config->groups[g].pids[gp] == pid) return true;
        }
    }
    return false;
}

void autopid_publish_last_known(void)
{
    if (!autopid_config || !autopid_config->mutex) return;
    if (config_server_mqtt_en_config() != 1 || !mqtt_connected()) return;

    uint32_t published = 0;

    // 1. Snapshot of DEST_DEFAULT params -> enabled MQTT group destinations.
    if (grouping_enabled()) {
        char *raw_json = autopid_data_read();
        if (raw_json) {
            for (uint32_t i = 0; i < autopid_config->destinations_count; i++) {
                group_destination_t *gd = &autopid_config->destinations[i];
                if (!gd->enabled) continue;
                if (gd->type != DEST_MQTT_TOPIC && gd->type != DEST_DEFAULT && gd->type != DEST_MQTT_WALLBOX) continue;
                const char *topic = (gd->destination && gd->destination[0] != '\0')
                                        ? gd->destination
                                        : config_server_get_mqtt_rx_topic();
                cJSON *snap = cJSON_Parse(raw_json);
                if (publish_last_known_json(topic, snap)) published++;
            }
            heap_caps_free(raw_json);
        }
    }

    // 2. Per-parameter topics and group bulk topics. Needs the config lock; the poller can
    // hold it for a full cycle, so wait a while instead of giving up immediately.
    if (xSemaphoreTake(autopid_config->mutex, pdMS_TO_TICKS(30000)) != pdTRUE) {
        ESP_LOGW(TAG, "Publish-on-connect: config busy, sent snapshot only (%lu msgs)", (unsigned long)published);
        return;
    }

    if (autopid_config->use_groups) {
        for (uint32_t g = 0; g < autopid_config->group_count; g++) {
            pid_group_t *group = &autopid_config->groups[g];
            if (!group->enabled) continue;
            bool has_group_topic = group->mqtt_topic && group->mqtt_topic[0] != '\0';
            cJSON *grp_json = has_group_topic ? cJSON_CreateObject() : NULL;

            for (uint32_t i = 0; i < group->pid_count; i++) {
                pid_data_t *pid = group->pids[i];
                if (!pid || !pid->enabled) continue;
                for (uint32_t p = 0; p < pid->parameters_count; p++) {
                    parameter_t *param = &pid->parameters[p];
                    if (param->destination_type == DEST_MQTT_GRP) {
                        if (grp_json && param_has_value(param)) {
                            add_param_value_to_json(grp_json, param);
                            mark_param_sent(param);
                        }
                    } else {
                        published += publish_last_known_param(param);
                    }
                }
            }
            if (publish_last_known_json(has_group_topic ? group->mqtt_topic : NULL, grp_json)) published++;
        }
    }

    for (uint32_t i = 0; i < autopid_config->pid_count; i++) {
        pid_data_t *pid = &autopid_config->pids[i];
        if (!pid->enabled || pid_is_in_any_group(pid)) continue;
        for (uint32_t p = 0; p < pid->parameters_count; p++) {
            published += publish_last_known_param(&pid->parameters[p]);
        }
    }

    for (uint32_t fi = 0; fi < autopid_config->can_filters_count; fi++) {
        can_filter_t *f = &autopid_config->can_filters[fi];
        for (uint32_t p = 0; p < f->parameters_count; p++) {
            published += publish_last_known_param(&f->parameters[p]);
        }
    }

    xSemaphoreGive(autopid_config->mutex);
    ESP_LOGI(TAG, "Publish-on-connect: sent %lu last-known message(s)", (unsigned long)published);
}

// Watches for MQTT connecting (rising edge) and publishes the last-known data once.
static void autopid_mqtt_connect_publish_task(void *pvParameters)
{
    for (;;) {
        dev_status_wait_for_bits(DEV_MQTT_CONNECTED_BIT, portMAX_DELAY);

        // Let the MQTT client finish its own on-connect work (status, subscriptions).
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (dev_status_is_mqtt_connected()) {
            ESP_LOGI(TAG, "MQTT connected, publishing last-known data");
            autopid_publish_last_known();
        }

        // Wait for the disconnect before arming again.
        while (dev_status_is_mqtt_connected()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void autopid_publish_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Autopid Publish Task Started");
    wc_timer_t skip_log_timer = 0;

    vTaskDelay(pdMS_TO_TICKS(5000)); // Initial delay to allow system stabilization
    for (;;)
    {
        if(!dev_status_is_sta_connected())
        {
            dev_status_wait_for_bits(DEV_STA_CONNECTED_BIT, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        // Only publish when autopid is enabled and STA is connected
        if (dev_status_is_autopid_enabled() && dev_status_is_sta_connected())
        {
            // Grouping must remain enabled at runtime
            if (autopid_config && autopid_config->grouping && strcmp("enable", autopid_config->grouping) == 0)
            {
                autopid_publish_all_destinations(false);
            }
            else
            {
                // Helpful visibility: if grouping toggles off at runtime, publish task becomes a no-op.
                if (skip_log_timer == 0 || wc_timer_is_expired(&skip_log_timer))
                {
                    wc_timer_set(&skip_log_timer, 5000);
                    ESP_LOGW(TAG, "Publish skipped: grouping disabled or config missing");
                }
            }
        }
        else
        {
            // Rate-limited visibility into the most common reasons publishing appears to "stop".
            if (skip_log_timer == 0 || wc_timer_is_expired(&skip_log_timer))
            {
                wc_timer_set(&skip_log_timer, 5000);
                ESP_LOGW(TAG, "Publish skipped: autopid_enabled=%d sta_connected=%d",
                         dev_status_is_autopid_enabled() ? 1 : 0,
                         dev_status_is_sta_connected() ? 1 : 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// -----------------------
// Webhook runtime stats helpers
// -----------------------

static void webhook_format_utc(char out[32])
{
    if (!out)
        return;
    out[0] = '\0';
    time_t now;
    time(&now);
    struct tm utc;
    gmtime_r(&now, &utc);
    strftime(out, 32, "%Y-%m-%dT%H:%M:%SZ", &utc);
}

static void webhook_sanitize_snippet(char *s)
{
    if (!s)
        return;
    for (char *p = s; *p; p++)
    {
        unsigned char c = (unsigned char)*p;
        if (c == '\r' || c == '\n' || c == '\t')
            *p = ' ';
        else if (c < 0x20)
            *p = ' ';
    }
}

static void autopid_webhook_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Autopid Webhook Task Started");


    uint32_t last_post_time = 0;
    static char *prev_autopid_snapshot = NULL; // previous snapshot for diffing when sending only changes
    static char *prev_config_snapshot = NULL;  // previous config for diffing
    static char *prev_status_snapshot = NULL;  // previous status for diffing

    vTaskDelay(pdMS_TO_TICKS(5000)); // Initial delay to allow system stabilization
    for (;;)
    {
        if(!dev_status_is_sta_connected())
        {
            dev_status_wait_for_bits(DEV_STA_CONNECTED_BIT, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        // Only post when autopid is enabled and STA is connected
        if (dev_status_is_autopid_enabled() && dev_status_is_sta_connected())
        {
            // Read webhook_data_mode setting from config
            bool send_full_data = false;
            if (autopid_config && autopid_config->webhook_data_mode)
            {
                send_full_data = (strcmp(autopid_config->webhook_data_mode, "full") == 0);
            }
            ha_webhook_config_t webhook_cfg = {0};
            esp_err_t err = ha_webhooks_get_config(&webhook_cfg);

            if (err == ESP_OK && webhook_cfg.enabled && webhook_cfg.url[0] != '\0')
            {
                uint32_t current_time = xTaskGetTickCount() / configTICK_RATE_HZ;
                uint32_t interval_sec = webhook_cfg.interval > 0 ? webhook_cfg.interval : 60;

                // Check if it's time to post
                if ((current_time - last_post_time) >= interval_sec)
                {
                    // IMPORTANT: update the schedule on attempt, not only on success.
                    // Otherwise if the endpoint is down we will retry every loop (1s) forever,
                    // which can starve other network traffic.
                    last_post_time = current_time;

                    // Get current autopid data
                    char *raw_json = autopid_data_read();
                    if (raw_json)
                    {
                        char *body = NULL;
                        char *current_config_data = NULL;
                        char *current_status_data = NULL;
                        char *current_autopid_data = NULL;

                        // Always send settings-style JSON: {config, status, autopid_data}
                        current_status_data = config_server_get_status_json(true);
                        current_config_data = autopid_get_config();
                        current_autopid_data = strdup_psram(raw_json);

                        cJSON *root_obj = cJSON_CreateObject();
                        if (root_obj)
                        {
                            cJSON *cfg_obj = NULL;
                            cJSON *sts_obj = NULL;
                            cJSON *auto_obj = NULL;

                                // Build config (full or diff)
                                cJSON *curr_cfg_src = NULL;
                                if (current_config_data)
                                {
                                    curr_cfg_src = cJSON_Parse(current_config_data);
                                }
                                if (!curr_cfg_src)
                                    curr_cfg_src = cJSON_CreateObject();
                                if (send_full_data)
                                {
                                    cfg_obj = cJSON_Duplicate(curr_cfg_src, true);
                                }
                                else
                                {
                                    cJSON *cfg_diff = cJSON_CreateObject();
                                    if (!cfg_diff)
                                        cfg_diff = cJSON_CreateObject();
                                    cJSON *prev_cfg_src = NULL;
                                    if (prev_config_snapshot)
                                    {
                                        prev_cfg_src = cJSON_Parse(prev_config_snapshot);
                                    }
                                    cJSON *it = NULL;
                                    cJSON_ArrayForEach(it, curr_cfg_src)
                                    {
                                        const char *key = it->string;
                                        if (!key)
                                            continue;
                                        bool changed = true;
                                        if (prev_cfg_src)
                                        {
                                            cJSON *prev_it = cJSON_GetObjectItemCaseSensitive(prev_cfg_src, key);
                                            if (prev_it)
                                            {
                                                if (cJSON_IsString(it) && cJSON_IsString(prev_it))
                                                {
                                                    const char *cs = it->valuestring ? it->valuestring : "";
                                                    const char *ps = prev_it->valuestring ? prev_it->valuestring : "";
                                                    changed = (strcmp(cs, ps) != 0);
                                                }
                                                else if (cJSON_IsNumber(it) && cJSON_IsNumber(prev_it))
                                                {
                                                    changed = (it->valuedouble != prev_it->valuedouble);
                                                }
                                                else if (cJSON_IsBool(it) && cJSON_IsBool(prev_it))
                                                {
                                                    int cv = cJSON_IsTrue(it) ? 1 : 0;
                                                    int pv = cJSON_IsTrue(prev_it) ? 1 : 0;
                                                    changed = (cv != pv);
                                                }
                                                else
                                                {
                                                    // Different types -> consider changed
                                                    changed = true;
                                                }
                                            }
                                        }
                                        if (changed)
                                        {
                                            if (cJSON_IsString(it) && it->valuestring)
                                                cJSON_AddStringToObject(cfg_diff, key, it->valuestring);
                                            else if (cJSON_IsNumber(it))
                                                cJSON_AddNumberToObject(cfg_diff, key, it->valuedouble);
                                            else if (cJSON_IsBool(it))
                                                cJSON_AddBoolToObject(cfg_diff, key, cJSON_IsTrue(it));
                                        }
                                    }
                                    cfg_obj = cfg_diff;
                                    if (prev_cfg_src)
                                        cJSON_Delete(prev_cfg_src);
                                    if (prev_config_snapshot)
                                    {
                                        free(prev_config_snapshot);
                                        prev_config_snapshot = NULL;
                                    }
                                    prev_config_snapshot = strdup_psram(current_config_data);
                                }
                                if (curr_cfg_src)
                                    cJSON_Delete(curr_cfg_src);

                                // Build status (full or diff)
                                cJSON *curr_sts_src = NULL;
                                if (current_status_data)
                                {
                                    curr_sts_src = cJSON_Parse(current_status_data);
                                }
                                if (!curr_sts_src)
                                    curr_sts_src = cJSON_CreateObject();
                                if (send_full_data)
                                {
                                    sts_obj = cJSON_Duplicate(curr_sts_src, true);
                                }
                                else
                                {
                                    cJSON *sts_diff = cJSON_CreateObject();
                                    if (!sts_diff)
                                        sts_diff = cJSON_CreateObject();
                                    cJSON *prev_sts_src = NULL;
                                    if (prev_status_snapshot)
                                    {
                                        prev_sts_src = cJSON_Parse(prev_status_snapshot);
                                    }
                                    cJSON *it2 = NULL;
                                    cJSON_ArrayForEach(it2, curr_sts_src)
                                    {
                                        const char *key = it2->string;
                                        if (!key)
                                            continue;
                                        bool changed = true;
                                        if (prev_sts_src)
                                        {
                                            cJSON *prev_it = cJSON_GetObjectItemCaseSensitive(prev_sts_src, key);
                                            if (prev_it)
                                            {
                                                if (cJSON_IsString(it2) && cJSON_IsString(prev_it))
                                                {
                                                    const char *cs = it2->valuestring ? it2->valuestring : "";
                                                    const char *ps = prev_it->valuestring ? prev_it->valuestring : "";
                                                    changed = (strcmp(cs, ps) != 0);
                                                }
                                                else if (cJSON_IsNumber(it2) && cJSON_IsNumber(prev_it))
                                                {
                                                    changed = (it2->valuedouble != prev_it->valuedouble);
                                                }
                                                else if (cJSON_IsBool(it2) && cJSON_IsBool(prev_it))
                                                {
                                                    int cv = cJSON_IsTrue(it2) ? 1 : 0;
                                                    int pv = cJSON_IsTrue(prev_it) ? 1 : 0;
                                                    changed = (cv != pv);
                                                }
                                                else
                                                {
                                                    changed = true;
                                                }
                                            }
                                        }
                                        if (changed)
                                        {
                                            if (cJSON_IsString(it2) && it2->valuestring)
                                                cJSON_AddStringToObject(sts_diff, key, it2->valuestring);
                                            else if (cJSON_IsNumber(it2))
                                                cJSON_AddNumberToObject(sts_diff, key, it2->valuedouble);
                                            else if (cJSON_IsBool(it2))
                                                cJSON_AddBoolToObject(sts_diff, key, cJSON_IsTrue(it2));
                                        }
                                    }
                                    sts_obj = sts_diff;
                                    if (prev_sts_src)
                                        cJSON_Delete(prev_sts_src);
                                    if (prev_status_snapshot)
                                    {
                                        free(prev_status_snapshot);
                                        prev_status_snapshot = NULL;
                                    }
                                    prev_status_snapshot = strdup_psram(current_status_data);
                                }
                                if (curr_sts_src)
                                    cJSON_Delete(curr_sts_src);

                                // Build autopid_data (full or only changed depending on __placeholder_flag__)
                                cJSON *curr_auto_src = NULL;
                                if (current_autopid_data)
                                {
                                    curr_auto_src = cJSON_Parse(current_autopid_data);
                                }
                                if (!curr_auto_src)
                                    curr_auto_src = cJSON_CreateObject();

                                if (send_full_data)
                                {
                                    // Send full autopid_data
                                    auto_obj = cJSON_Duplicate(curr_auto_src, true);
                                }
                                else
                                {
                                    // Send only changed values compared to previous snapshot
                                    cJSON *diff_obj = cJSON_CreateObject();
                                    if (!diff_obj)
                                        diff_obj = cJSON_CreateObject();

                                    cJSON *prev_src = NULL;
                                    if (prev_autopid_snapshot)
                                    {
                                        prev_src = cJSON_Parse(prev_autopid_snapshot);
                                    }
                                    // Iterate current keys and compare to previous
                                    cJSON *it = NULL;
                                    cJSON_ArrayForEach(it, curr_auto_src)
                                    {
                                        const char *key = it->string;
                                        if (!key)
                                            continue;
                                        double curr_val = 0;
                                        bool has_curr = false;
                                        if (cJSON_IsNumber(it))
                                        {
                                            curr_val = it->valuedouble;
                                            has_curr = true;
                                        }
                                        else if (cJSON_IsString(it) && it->valuestring)
                                        {
                                            has_curr = true;
                                        }
                                        else if (cJSON_IsBool(it))
                                        {
                                            curr_val = cJSON_IsTrue(it) ? 1.0 : 0.0;
                                            has_curr = true;
                                        }
                                        if (!has_curr)
                                            continue;

                                        bool changed = true;
                                        if (prev_src)
                                        {
                                            cJSON *prev_it = cJSON_GetObjectItemCaseSensitive(prev_src, key);
                                            if (prev_it)
                                            {
                                                if (cJSON_IsNumber(it) && cJSON_IsNumber(prev_it))
                                                {
                                                    double prev_val = prev_it->valuedouble;
                                                    changed = (curr_val != prev_val);
                                                }
                                                else if (cJSON_IsString(it) && cJSON_IsString(prev_it))
                                                {
                                                    const char *cs = it->valuestring ? it->valuestring : "";
                                                    const char *ps = prev_it->valuestring ? prev_it->valuestring : "";
                                                    changed = (strcmp(cs, ps) != 0);
                                                }
                                                else if (cJSON_IsBool(it) && cJSON_IsBool(prev_it))
                                                {
                                                    int cv = cJSON_IsTrue(it) ? 1 : 0;
                                                    int pv = cJSON_IsTrue(prev_it) ? 1 : 0;
                                                    changed = (cv != pv);
                                                }
                                            }
                                        }

                                        if (changed)
                                        {
                                            if (cJSON_IsNumber(it))
                                            {
                                                cJSON_AddNumberToObject(diff_obj, key, it->valuedouble);
                                            }
                                            else if (cJSON_IsString(it) && it->valuestring)
                                            {
                                                cJSON_AddStringToObject(diff_obj, key, it->valuestring);
                                            }
                                            else if (cJSON_IsBool(it))
                                            {
                                                cJSON_AddBoolToObject(diff_obj, key, cJSON_IsTrue(it));
                                            }
                                        }
                                    }

                                    auto_obj = diff_obj;
                                    if (prev_src)
                                        cJSON_Delete(prev_src);
                                    if (prev_autopid_snapshot)
                                    {
                                        free(prev_autopid_snapshot);
                                        prev_autopid_snapshot = NULL;
                                    }
                                    prev_autopid_snapshot = strdup_psram(current_autopid_data);
                                }
                                if (curr_auto_src)
                                    cJSON_Delete(curr_auto_src);

                                // Only add sections that have data (non-empty objects)
                                if (cfg_obj && cJSON_GetArraySize(cfg_obj) > 0)
                                {
                                    cJSON_AddItemToObject(root_obj, "config", cfg_obj);
                                }
                                else if (cfg_obj)
                                {
                                    cJSON_Delete(cfg_obj);
                                }

                                if (sts_obj && cJSON_GetArraySize(sts_obj) > 0)
                                {
                                    cJSON_AddItemToObject(root_obj, "status", sts_obj);
                                }
                                else if (sts_obj)
                                {
                                    cJSON_Delete(sts_obj);
                                }

                                if (auto_obj && cJSON_GetArraySize(auto_obj) > 0)
                                {
                                    cJSON_AddItemToObject(root_obj, "autopid_data", auto_obj);
                                }
                                else if (auto_obj)
                                {
                                    cJSON_Delete(auto_obj);
                                }

                                // Add mock GPS data
                                // cJSON *gps = cJSON_CreateObject();
                                // if (gps)
                                // {
                                //     cJSON_AddNumberToObject(gps, "latitude", 37.7749);
                                //     cJSON_AddNumberToObject(gps, "longitude", -122.4194);
                                //     cJSON_AddNumberToObject(gps, "accuracy", 10);
                                //     cJSON_AddNumberToObject(gps, "altitude", 25.5);
                                //     cJSON_AddNumberToObject(gps, "speed", 15.3);
                                //     cJSON_AddNumberToObject(gps, "heading", 180);
                                //     cJSON_AddItemToObject(root_obj, "gps", gps);
                                // }
                                char *printed = cJSON_PrintUnformatted(root_obj);
                                if (printed)
                                {
                                    body = strdup_psram(printed);
                                    free(printed);
                                }
                                cJSON_Delete(root_obj);
                            }

                            // Free only buffers we allocated locally.
                            // current_autopid_data is a strdup_psram of raw_json  safe to free.
                            if (current_autopid_data)
                            {
                                free(current_autopid_data);
                                current_autopid_data = NULL;
                            }
                            // current_status_data is returned heap buffer by config_server_get_status_json(true)  safe to free.
                            if (current_status_data)
                            {
                                free(current_status_data);
                                current_status_data = NULL;
                            }
                            // current_config_data comes from autopid_get_config() and is a cached global  DO NOT free here.

                        if (body)
                        {
                                https_client_mgr_response_t last_resp = {0};
                                esp_err_t post_err = ESP_FAIL;
                                bool ok = false;
                                const char *successful_url = NULL;

#if HA_WEBHOOK_MAX_URLS > 0
                                size_t webhook_url_count = webhook_cfg.url_count > 0 ? webhook_cfg.url_count : 1;
#else
                                size_t webhook_url_count = 1;
#endif

                                for (size_t url_index = 0; url_index < webhook_url_count; ++url_index)
                                {
#if HA_WEBHOOK_MAX_URLS > 0
                                    const char *target_url = webhook_cfg.url_count > 0 ? webhook_cfg.urls[url_index] : webhook_cfg.url;
#else
                                    const char *target_url = webhook_cfg.url;
#endif
                                    if (!target_url || target_url[0] == '\0')
                                        continue;

                                    https_client_mgr_config_t cfg = {0};
                                    cfg.url = target_url;
                                    cfg.timeout_ms = 5000;

                                    // Detect scheme from URL
                                    bool is_https_url = (strncasecmp(target_url, "https://", 8) == 0);
                                    if (is_https_url)
                                    {
                                        // Use built-in certificate bundle for HTTPS
                                        cfg.use_crt_bundle = true;
                                        ESP_LOGI(TAG, "Using built-in certificate bundle for webhook HTTPS");

                                        // For HTTPS, if host is raw IPv4 address, skip CN verification
                                        const char *host_start = strstr(target_url, "://");
                                        host_start = host_start ? host_start + 3 : target_url;
                                        char host_buf[64] = {0};
                                        size_t hi = 0;
                                        while (host_start[hi] && host_start[hi] != '/' && host_start[hi] != ':' && hi < sizeof(host_buf) - 1)
                                        {
                                            host_buf[hi] = host_start[hi];
                                            hi++;
                                        }
                                        bool is_ip = true;
                                        for (size_t k = 0; k < hi; k++)
                                        {
                                            if ((host_buf[k] < '0' || host_buf[k] > '9') && host_buf[k] != '.')
                                            {
                                                is_ip = false;
                                                break;
                                            }
                                        }
                                        if (is_ip)
                                        {
                                            cfg.skip_common_name = true;
                                        }
                                    }

                                    https_client_mgr_response_t resp = {0};
                                    post_err = https_client_mgr_request_with_auth(&cfg, HTTPS_METHOD_POST,
                                                                                  body, strlen(body),
                                                                                  "application/json",
                                                                                  NULL,
                                                                                  NULL,
                                                                                  &resp);

                                    ok = (post_err == ESP_OK && resp.is_success);
                                    if (ok)
                                    {
                                        successful_url = target_url;
                                        last_resp = resp;
                                        ESP_LOGI(TAG, "Webhook POST success via %s, status %d", target_url, resp.status_code);
                                        break;
                                    }

                                    ESP_LOGE(TAG, "Webhook POST failed via %s: %s", target_url, esp_err_to_name(post_err));
                                    https_client_mgr_free_response(&last_resp);
                                    last_resp = resp;
                                }

                                // Update runtime webhook stats in cache (no filesystem write)
                                // NOTE: This task uses a PSRAM stack, so it must not touch file I/O.
                                ha_webhook_config_t upd = webhook_cfg;
                                if (ok)
                                {
                                    upd.success_count++;
                                    upd.retries = 0;
                                    strlcpy(upd.status, "ok", sizeof(upd.status));
                                    webhook_format_utc(upd.last_post);
                                    upd.last_error[0] = '\0';
                                    upd.last_error_time[0] = '\0';
                                    if (successful_url)
                                    {
                                        ESP_LOGI(TAG, "Webhook delivery completed using %s", successful_url);
                                    }
                                }
                                else
                                {
                                    upd.fail_count++;
                                    upd.retries++;
                                    strlcpy(upd.status, "failed", sizeof(upd.status));
                                    webhook_format_utc(upd.last_error_time);

                                    char snippet[96] = {0};
                                    if (last_resp.data && last_resp.data_len > 0)
                                    {
                                        size_t n = (size_t)last_resp.data_len;
                                        if (n > (sizeof(snippet) - 1))
                                            n = (sizeof(snippet) - 1);
                                        memcpy(snippet, last_resp.data, n);
                                        snippet[n] = '\0';
                                        webhook_sanitize_snippet(snippet);
                                    }

                                    if (post_err != ESP_OK)
                                    {
                                        if (snippet[0])
                                            snprintf(upd.last_error, sizeof(upd.last_error), "esp_err=%s; resp=%s", esp_err_to_name(post_err), snippet);
                                        else
                                            snprintf(upd.last_error, sizeof(upd.last_error), "esp_err=%s", esp_err_to_name(post_err));
                                    }
                                    else
                                    {
                                        if (snippet[0])
                                            snprintf(upd.last_error, sizeof(upd.last_error), "http=%d; resp=%s", last_resp.status_code, snippet);
                                        else
                                            snprintf(upd.last_error, sizeof(upd.last_error), "http=%d", last_resp.status_code);
                                    }
                                }
                                (void)ha_webhooks_update_cache(&upd);

                                https_client_mgr_free_response(&last_resp);
                            free(body);
                        }
                        free(raw_json);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}


static void autopid_publish_status_mqtt(void) {
    // Throttle this check to only run once per second so we don't spam the CPU
    static wc_timer_t check_timer = 0;
    if (check_timer != 0 && !wc_timer_is_expired(&check_timer)) {
        return;
    }
    wc_timer_set(&check_timer, 1000);

    // Verify MQTT and Status publishing are both enabled
    if (config_server_mqtt_en_config() != 1 || !mqtt_connected()) return;
    if (strcmp(config_server_get_mqtt_disc_status_en(), "enable") != 0) return;

    static wc_timer_t status_timer = 0;
    static float last_voltage = -1.0;
    
    bool mode_periodic = (strcmp(config_server_get_mqtt_disc_status_mode(), "periodic") == 0);
    bool should_publish = false;

    if (mode_periodic) {
        uint32_t period_sec = atoi(config_server_get_mqtt_disc_status_period());
        if (period_sec < 1) period_sec = 60; // Fallback to 60s if invalid
        
        if (status_timer == 0 || wc_timer_is_expired(&status_timer)) {
            should_publish = true;
            wc_timer_set(&status_timer, period_sec * 1000);
        }
    } else {
        // "On Change" Mode: Check if the battery voltage has drifted by 0.1V or more
        float current_v = 0;
        if (sleep_mode_get_voltage(&current_v) == ESP_OK) {
            if (fabs(current_v - last_voltage) >= 0.1) {
                should_publish = true;
                last_voltage = current_v;
            }
        }
    }

    // Publish the payload
    if (should_publish) {
        char *status_json = config_server_get_status_json(false);
        if (status_json) {
            const char *topic = config_server_get_mqtt_status_topic();
            // Publish with QoS 0, Retain 0
            mqtt_publish(topic, status_json, strlen(status_json), 0, 0); 
            free(status_json);
        }
    }
}

static void autopid_task(void *pvParameters)
{
    static char default_init[] = "ati\rate0\rath1\ratl0\rats1\ratm0\ratst96\r";
    wc_timer_t ecu_check_timer;
    
    // Change this: response_t monitor_rsp;
    response_t *monitor_rsp = heap_caps_malloc(sizeof(response_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Autopid Task Started");

    if (!autopid_config) {
        ESP_LOGE(TAG, "autopid_config is NULL");
        return;
    }

    // Protocol Setup
    char early_proto_cmd[16] = {0};
    bool have_early_proto_cmd = false;

    if (autopid_config->std_ecu_protocol && strcmp(autopid_config->std_ecu_protocol, "0") == 0) {
        ESP_LOGI(TAG, "Protocol is Auto");
        uint8_t auto_protocol_number = 0;
        if (elm327_get_protocol_number(&auto_protocol_number) == ESP_OK) {
            autopid_set_protocol_number(auto_protocol_number);
            if (autopid_config->standard_init) free(autopid_config->standard_init);
            char protocol_str[16];
            snprintf(protocol_str, sizeof(protocol_str), "ATTP%01X\r", auto_protocol_number);
            autopid_config->standard_init = strdup_psram(protocol_str);
            snprintf(early_proto_cmd, sizeof(early_proto_cmd), "ATTP%01X\r", auto_protocol_number);
            have_early_proto_cmd = true;
        } else {
            ESP_LOGE(TAG, "Failed to get protocol number");
        }
        ESP_LOGI(TAG, "Protocol number: %u", auto_protocol_number);
    } else if (autopid_config->std_ecu_protocol && autopid_config->std_ecu_protocol[0] != '\0') {
        char *endptr = NULL;
        long v = strtol(autopid_config->std_ecu_protocol, &endptr, 16);
        if (endptr != autopid_config->std_ecu_protocol && v >= 0 && v <= 0xFF) {
            if (v <= 0xF) snprintf(early_proto_cmd, sizeof(early_proto_cmd), "ATTP%01lX\r", v);
            else snprintf(early_proto_cmd, sizeof(early_proto_cmd), "ATTP%02lX\r", v);
            have_early_proto_cmd = true;
            ESP_LOGI(TAG, "Protocol is fixed: %s (early cmd: %s)", autopid_config->std_ecu_protocol, early_proto_cmd);
        } else {
            ESP_LOGW(TAG, "Invalid std_ecu_protocol: %s", autopid_config->std_ecu_protocol);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(150));
    send_commands(default_init, 50);
    if (have_early_proto_cmd) {
        send_commands(early_proto_cmd, 2);
    }
    ESP_LOGI(TAG, "Autopid Start loop");
    ESP_LOGI(TAG, "Total PIDs: %lu", autopid_config->pid_count);
    ESP_LOGI(TAG, "Total CAN Filters: %lu", autopid_config->can_filters_count);
    wc_timer_set(&ecu_check_timer, 2000);
    
    while (1)
    {
        static pid_type_t previous_pid_type = PID_MAX;
        static bool pid_polling_paused_prev = false;

        dev_status_wait_for_bits(DEV_AUTOPID_ELM327_APP_BIT, portMAX_DELAY);
        
        if (dev_status_is_sleeping()) {
            ESP_LOGI(TAG, "Device is sleeping, waiting for wakeup");
            obd_logger_disable();
            dev_status_set_bits(DEV_AUTOPID_IDLE_BIT);
            dev_status_wait_for_bits(DEV_AWAKE_BIT, portMAX_DELAY);
            dev_status_clear_bits(DEV_AUTOPID_IDLE_BIT);
            ESP_LOGI(TAG, "Device awake, resuming autopid task");
            obd_logger_enable();
        }

	// --- NEW: BYPASS VOLTAGE PAUSE IF USING PID SLEEP MODE ---
        if (config_server_get_sleep_config() != 2) {
	  
	  if (autopid_config->disable_on_sleep_voltage && !dev_status_is_wake_voltage_ok()) {
            ESP_LOGI(TAG, "Voltage below sleep threshold, pausing autopid until voltage recovers");
            obd_logger_disable();
            dev_status_set_bits(DEV_AUTOPID_IDLE_BIT);
            dev_status_wait_for_bits(DEV_WAKE_VOLTAGE_OK_BIT, portMAX_DELAY);
            dev_status_clear_bits(DEV_AUTOPID_IDLE_BIT);
            ESP_LOGI(TAG, "Voltage OK, resuming autopid task");
            obd_logger_enable();
	  }
	}

        if (!dev_status_is_autopid_enabled()) {
            ESP_LOGI(TAG, "Autopid is disabled, waiting for enable");
            obd_logger_disable();
            dev_status_set_bits(DEV_AUTOPID_IDLE_BIT);
            dev_status_wait_for_bits(DEV_AUTOPID_ENABLED_BIT, portMAX_DELAY);
            dev_status_clear_bits(DEV_AUTOPID_IDLE_BIT);
            ESP_LOGI(TAG, "Autopid enabled, resuming autopid task");
            obd_logger_enable();
            send_commands(default_init, 50);
            vTaskDelay(pdMS_TO_TICKS(100));
        }


         // Optional low-voltage behaviors that only pause PID polling...
        float batt_v = NAN;
        const char *pause_reason = NULL;
        bool pid_polling_paused = false;
        
        // --- NEW: BYPASS PAUSE HERE AS WELL ---
        if (config_server_get_sleep_config() != 2) {
            pid_polling_paused = autopid_pause_should_pause_pid_polling(autopid_config, &batt_v, &pause_reason);
        }

	
        if (pid_polling_paused != pid_polling_paused_prev)
        {
            if (pid_polling_paused)
            {
                if (pause_reason && strcmp(pause_reason, "automate_threshold") == 0 && !isnan(batt_v))
                {
                    ESP_LOGI(TAG, "PID polling paused (automate_threshold): %.2fV < %.2fV", batt_v, autopid_config->pid_polling_min_voltage);
                }
                else if (pause_reason && strcmp(pause_reason, "sleep_voltage") == 0)
                {
                    ESP_LOGI(TAG, "PID polling paused (sleep_voltage): below configured sleep voltage");
                }
                else
                {
                    ESP_LOGI(TAG, "PID polling paused");
                }
            }
            else
            {
                ESP_LOGI(TAG, "PID polling resumed");
                // Force init resend on next due PID to avoid stale ELM state.
                previous_pid_type = PID_MAX;
                // not used current_active_init = NULL;
            }
            pid_polling_paused_prev = pid_polling_paused;
        }

        // elm327_lock();
        if (!pid_polling_paused)
        {
            xSemaphoreTake(autopid_config->mutex, portMAX_DELAY);

            bool pids_polled = false;
            // ==========================================================================================
            // [NEW] GROUP MODE LOGIC (SYNCHRONIZED)
            // ==========================================================================================
  
            if (autopid_config->use_groups) {
                for (uint32_t g = 0; g < autopid_config->group_count; g++) {
                    pid_group_t *group = &autopid_config->groups[g];
                    if (!group->enabled) continue;		

                    // ---> SET THE GLOBAL POINTER <---
                    current_group_mqtt_topic = group->mqtt_topic;
		    
                    bool active = true;
                    int64_t now_ms = esp_timer_get_time() / 1000;

                    if (group->detection_method == DETECTION_VOLTAGE) {
                        float v = 0;
                        if (sleep_mode_get_voltage(&v) == ESP_OK) {
                            active = (v >= group->wake_voltage);
                        } else {
                            active = false;
                        }
                    } else if (group->detection_method == DETECTION_ADAPTIVE_RPM) {
                        active = is_engine_running();	
                    } else if (group->detection_method == DETECTION_MQTT) {
                       if (group->mqtt_active_flag) {
                           if (!group->mqtt_timer_disabled && wc_timer_is_expired(&group->mqtt_active_timer)) {
                               group->mqtt_active_flag = false;
                               active = false;
                               ESP_LOGI(TAG, "Group '%s' MQTT timeout expired. Auto-disabling.", group->name);
                           } else { active = true; }
                       } else { active = false; }

		       
                    } else if (group->detection_method == DETECTION_CUSTOM_PID) {
                        // --- NEW: THE GATEKEEPER LOGIC ---
                        if (now_ms < group->next_allowed_time_ms) {
                            active = false; // Still in 3-second cooldown
                        } else {
                            // 1. Send Init String
                            if (group->custom_pid_init && strlen(group->custom_pid_init) > 0) {
                                send_commands(group->custom_pid_init, 2);
                            }

// 2. Send PID Mode
                            if (group->custom_pid_mode && strlen(group->custom_pid_mode) > 0) {

                                // --- FIXED: Allocate Gatekeeper response on Heap ---
                                response_t *gate_resp = heap_caps_malloc(sizeof(response_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                                if (gate_resp) {
                                    // Flush queue safely
                                    while (xQueueReceive(autopidQueue, gate_resp, 0) == pdPASS) {
                                        if (gate_resp->priority_data) free(gate_resp->priority_data);
                                    }
                                    
                                    char gate_cmd[32];
                                    snprintf(gate_cmd, sizeof(gate_cmd), "%s\r", group->custom_pid_mode);
                                    
                                    #if HARDWARE_VER == WICAN_PRO
                                    if (elm327_process_cmd((uint8_t *)gate_cmd, strlen(gate_cmd), &autopidQueue, elm327_autopid_cmd_buffer, &elm327_autopid_cmd_buffer_len, &elm327_autopid_last_cmd_time, autopid_parser) == ESP_OK)
                                    #else
                                    twai_message_t tx_msg;
                                    if (elm327_process_cmd((uint8_t *)gate_cmd, strlen(gate_cmd), &tx_msg, &autopidQueue) == ESP_OK)
                                    #endif
                                    {
                                        memset(gate_resp, 0, sizeof(response_t));
                                        
                                        xSemaphoreGive(autopid_config->mutex);
                                        BaseType_t got = xQueueReceive(autopidQueue, gate_resp, pdMS_TO_TICKS(500));
                                        xSemaphoreTake(autopid_config->mutex, portMAX_DELAY);
                                        
                                        if (got == pdPASS && strstr((char*)gate_resp->data, "error") == NULL && strstr((char*)gate_resp->data, "NO DATA") == NULL) {
                                            // 3. Evaluate Expression
                                            double val = 0;
                                            char* expr = (group->custom_pid_expr && strlen(group->custom_pid_expr) > 0) ? group->custom_pid_expr : "A";
                                            
                                           if (evaluate_expression((uint8_t*)expr, gate_resp->data, 0, &val)) {
                                            
                                            // --- RESTORE THIS MISSING LOGIC ---
                                            bool cond_met = false;
                                            if (group->custom_pid_operator && strcmp(group->custom_pid_operator, "greater") == 0 && val > group->custom_pid_value) cond_met = true;
                                            else if (group->custom_pid_operator && strcmp(group->custom_pid_operator, "less") == 0 && val < group->custom_pid_value) cond_met = true;
                                            else if (group->custom_pid_operator && strcmp(group->custom_pid_operator, "equals") == 0 && val == group->custom_pid_value) cond_met = true;
                                            // ----------------------------------

                                            if (cond_met) {
                                                active = true; 
                                            } else {
                                                active = false;
                                                group->next_allowed_time_ms = now_ms + 3000;
                                            }
                                        } else {
                                            active = false;
                                            group->next_allowed_time_ms = now_ms + 3000;
                                        }
                                        
                                        if (gate_resp->priority_data != NULL) {
                                            free(gate_resp->priority_data);
                                        }
                                     } else {
                                        active = false;
                                        group->next_allowed_time_ms = now_ms + 3000;
                                    }
                                    
                                    // FREE THE HEAP POINTER
                                    free(gate_resp);

                                // --- FIXED: Close the elm327_process_cmd block! ---
                                } else {
                                    active = false;
                                    group->next_allowed_time_ms = now_ms + 3000; // ELM process failed
                                }
                                // --------------------------------------------------

                            } else {
                                active = false; // Malloc failed
                            }
                        } else {
                            active = false; // Missing PID to query
                        }
                        }
                    }

                    if (!active) continue;

                    bool run_batch = false;
                    if (group->period > 0) {
                        if (wc_timer_is_expired(&group->timer)) {
                            run_batch = true;
                            /* RESTORED: Reset timer at the START of the batch */
                            wc_timer_set(&group->timer, group->period); 
                        }
                    } else { run_batch = true; }
		    
                    if (!run_batch) continue;

		    pids_polled = true;

                    if (group->init && strlen(group->init) > 0) {
                        /* RESTORED: Simple init send without flush loops */
                        send_commands(group->init, 5);
                        // not used current_active_init = group->init;
                    }

                    for (uint32_t i = 0; i < group->pid_count; i++) {
                        pid_data_t *curr_pid = group->pids[i];
                        if (!curr_pid || !curr_pid->enabled) continue;

                        // Check if ANY parameter in this PID is enabled before querying the car
                        bool any_param_enabled = false;
                        for (uint32_t p = 0; p < curr_pid->parameters_count; p++) {
                            if (curr_pid->parameters[p].enabled) {
                                any_param_enabled = true;
                                break;
                            }
                        }
                        
if (any_param_enabled) {
                            // Call our new unified function! (false = Groups use group timers, ignore param timers)
                            execute_pid(curr_pid, false);
                        }
                    }

                    // ==========================================================================================
                    // [NEW] AGGREGATE AND PUBLISH MQTT_GRP AS ONE BULK MESSAGE
                    // ==========================================================================================
                    // Skip while MQTT is offline so on-change history is not updated
                    // for values that were never delivered.
                    if (group->mqtt_topic && strlen(group->mqtt_topic) > 0 && mqtt_connected()) {
                        cJSON *grp_json = NULL;
                        bool has_data = false;

                        for (uint32_t i = 0; i < group->pid_count; i++) {
                            pid_data_t *curr_pid = group->pids[i];
                            if (!curr_pid || !curr_pid->enabled) continue;

                            for (uint32_t p = 0; p < curr_pid->parameters_count; p++) {
                                parameter_t *param = &curr_pid->parameters[p];
                                if (!param->enabled || param->failed) continue;

                                if (param->destination_type == DEST_MQTT_GRP && param->name) {
                                    
                                    // 1. Honor the "On Change" Checkbox
                                    bool include_param = true;
                                    if (param->onchange) {
                                        if (param->raw_string_value != NULL) {
                                            if (param->last_sent_raw_string && strcmp(param->raw_string_value, param->last_sent_raw_string) == 0) {
                                                include_param = false;
                                            }
                                        } else {
                                            if (param->value == param->last_sent_value) {
                                                include_param = false;
                                            }
                                        }
                                    }

                                    // 2. Add to JSON and Update History
                                    if (include_param) {
                                        if (!grp_json) grp_json = cJSON_CreateObject();
                                        if (grp_json) {
                                            if (param->raw_string_value != NULL) {
                                                cJSON_AddStringToObject(grp_json, param->name, param->raw_string_value);
                                                // Update string history
                                                if (param->last_sent_raw_string) free(param->last_sent_raw_string);
                                                param->last_sent_raw_string = strdup_psram(param->raw_string_value);
                                            } else if (param->value != FLT_MAX) {
                                                if (param->sensor_type == BINARY_SENSOR) {
                                                    cJSON_AddStringToObject(grp_json, param->name, param->value > 0 ? "on" : "off");
                                                } else {
                                                    cJSON_AddNumberToObject(grp_json, param->name, param->value);
                                                }
                                                // Update number history
                                                param->last_sent_value = param->value;
                                            }
                                            has_data = true;
                                        }
                                    }
                                }
                            }
                        }

                        // 3. Publish the Bulk Payload
                        if (has_data && grp_json) {
                            limitJsonDecimalPrecision(grp_json);
                            if (config_server_get_mqtt_include_timestamp() == 1) {
                                cJSON_AddNumberToObject(grp_json, "timestamp", (double)time(NULL));
                            }
                            char *payload = cJSON_PrintUnformatted(grp_json);
                            if (payload) {
                                mqtt_publish(group->mqtt_topic, payload, 0, 0, 1);
                                ESP_LOGI(TAG, "Published bulk group payload to %s", group->mqtt_topic);
                                free(payload);
                            }
                        }
                        if (grp_json) {
                            cJSON_Delete(grp_json);
                        }
                    } // Ends: if (group->mqtt_topic ...)
                } // Ends: for (uint32_t g = ...)
                current_group_mqtt_topic = NULL;
		} // Ends: if (autopid_config->use_groups)

            // ==========================================================================================
            // [LEGACY] FLAT LIST MODE
            // ==========================================================================================
            else if (autopid_config->pid_count > 0) {

                for (uint32_t i = 0; i < autopid_config->pid_count; i++) {
                    pid_data_t *curr_pid = &autopid_config->pids[i];

                    // Group bypass check
                    bool is_in_group = false;
                    if (autopid_config->use_groups) {
                        for (uint32_t g = 0; g < autopid_config->group_count; g++) {
                            for (uint32_t gp = 0; gp < autopid_config->groups[g].pid_count; gp++) {
                                if (autopid_config->groups[g].pids[gp] == curr_pid) {
                                    is_in_group = true; break;
                                }
                            }
                            if (is_in_group) break;
                        }
                    }
                    if (is_in_group) continue;

                    if ((curr_pid->pid_type == PID_STD && !autopid_config->pid_std_en) ||
                        (curr_pid->pid_type == PID_CUSTOM && !autopid_config->pid_custom_en) ||
                        (curr_pid->pid_type == PID_SPECIFIC && !autopid_config->pid_specific_en)) continue;

                    if (!curr_pid->enabled) continue;
                    
                    // Check if ANY parameter in this PID is due for a poll right now
                    bool any_due = false;
                    for (uint32_t p = 0; p < curr_pid->parameters_count; p++) {
                        if (curr_pid->parameters[p].enabled && wc_timer_is_expired(&curr_pid->parameters[p].timer)) {
                            any_due = true;
                            break;
                        }
                    }

                    if (any_due) {
		        pids_polled = true;
                        if (curr_pid->pid_type != previous_pid_type) {
                            // FORCE a protocol reset when switching list types
                            // not used current_active_init = NULL; 

                            switch (curr_pid->pid_type) {
                                case PID_CUSTOM: 
                                  if(autopid_config->custom_init) send_commands(autopid_config->custom_init, 2); 
                                  break;
                                case PID_STD: 
                                  if(autopid_config->standard_init) send_commands(autopid_config->standard_init, 2); 
                                  break;
                                case PID_SPECIFIC: 
                                  if(autopid_config->specific_init) send_commands(autopid_config->specific_init, 2); 
                                  break;
                                default: break;
                            }
                            previous_pid_type = curr_pid->pid_type;
                        }

                        // Execute the PID ONCE for all due parameters! (true = check param timers)
                        execute_pid(curr_pid, true);
                    }
                }
            }

            xSemaphoreGive(autopid_config->mutex);

            /* [NEW] Signal processing task ONLY if PIDs were actually polled */
            if (pids_polled && autopid_processing_task_handle != NULL) {
                xTaskNotifyGive(autopid_processing_task_handle);
            }	
        }

        if (autopid_config->can_filters_count > 0 && dev_status_is_autopid_enabled() && !dev_status_is_sleeping())
        {
            while (xQueueReceive(autopidQueue, monitor_rsp, 0) == pdPASS);
            xSemaphoreTake(autopid_config->mutex, portMAX_DELAY);  
            bool did_monitor = false;
            for (uint32_t fi = 0; fi < autopid_config->can_filters_count; fi++) {
                can_filter_t *f = &autopid_config->can_filters[fi];
                if (f->is_vehicle_specific && !autopid_config->pid_specific_en) continue;
                if (f->frame_id == 0 || f->parameters_count == 0) continue;
                bool any_due = false;
                for (uint32_t pi = 0; pi < f->parameters_count; pi++) {
                    if (f->parameters[pi].enabled && wc_timer_is_expired(&f->parameters[pi].timer)) { any_due = true; break; }
                }
                if (!any_due) continue;
                
                ESP_LOGI(TAG, "Monitoring CAN filter frame_id=0x%lX", f->frame_id);
                send_can_filter_cmd(f->frame_id);
                send_commands("ATCAF0\r" , 2);
                autopid_atma_parser_reset();
                autopid_atma_set_expected_frame_id(f->frame_id);
                elm327_run_command("ATMA\r", 0, 1100, &autopidQueue, autopid_atma_parser, true, f->frame_id);
                autopid_atma_set_expected_frame_id(0);
                send_commands("ATCAF1\r" , 2);
                did_monitor = true;

                while (xQueueReceive(autopidQueue, monitor_rsp, 0) == pdPASS) {
                    process_can_filter_frame(f, monitor_rsp);
                }
	    }
            if (did_monitor) {
                send_commands("ATCRA\r", 2);
                autopid_data_update(autopid_config);
                // previous_pid_type = PID_MAX; // (Legacy var not available here, but logic handles reset on next loop)
            }
            xSemaphoreGive(autopid_config->mutex);
        }

        if (wc_timer_is_expired(&ecu_check_timer))
        {
            if (!pid_polling_paused)
            {
               if (all_parameters_failed(autopid_config))
                {
                    xEventGroupClearBits(xautopid_event_group, ECU_CONNECTED_BIT);
                    ESP_LOGW(TAG, "All parameters failed - ECU disconnected");
		    // current_active_init = NULL;
                }
                else
                {
                    xEventGroupSetBits(xautopid_event_group, ECU_CONNECTED_BIT);
                }
            }
            wc_timer_set(&ecu_check_timer, 2000);
        }


	// --- NEW: TRIGGER THE STATUS ENGINE ---
        autopid_publish_status_mqtt();
        // --------------------------------------

        // If PID polling is paused (e.g., low voltage), yield a bit longer to avoid a tight loop.
        // Otherwise, still yield minimally to stay watchdog-safe in edge cases (no work due).
        if (pid_polling_paused)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

static void autopid_init_obd_logger(uint32_t log_period)
{
    ESP_LOGI(TAG, "Initializing Autopid OBD logger...");
    obd_param_entry_t *params = NULL;
    size_t param_count = 0;
    size_t max_params = 0;
    for (uint32_t i = 0; i < autopid_config->pid_count; i++) max_params += autopid_config->pids[i].parameters_count;

    params = heap_caps_malloc(sizeof(obd_param_entry_t) * max_params, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!params) { ESP_LOGE(TAG, "Failed to allocate memory for OBD logger parameters"); return; }

    for (uint32_t i = 0; i < autopid_config->pid_count; i++) {
        pid_data_t *pid = &autopid_config->pids[i];
        for (uint32_t j = 0; j < pid->parameters_count; j++) {
            parameter_t *param = &pid->parameters[j];
            char *metadata = heap_caps_malloc(1024 * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!metadata) continue;
            snprintf(metadata, 1024 * 4, "{\"unit\":\"%s\",\"period\":%lu}", param->unit ? param->unit : "", param->period);
            params[param_count].name = strdup_psram(param->name);
            params[param_count].type = "NUMERIC";
            params[param_count].metadata = metadata;
            param_count++;
        }
    }

    if (mkdir(DB_ROOT_PATH "/" DB_DIR_NAME, 0755) != 0) {
        if (errno != EEXIST) ESP_LOGE(TAG, "Failed to create directory %s: %s", DB_ROOT_PATH "/" DB_DIR_NAME, strerror(errno));
    } else ESP_LOGI(TAG, "Created directory: %s", DB_ROOT_PATH "/" DB_DIR_NAME);

    static obd_logger_t obd_logger = {
        .path = DB_ROOT_PATH "/" DB_DIR_NAME,
        .db_filename = DB_ROOT_PATH "/" DB_DIR_NAME "/" DB_DIR_NAME,
        .obd_logger_get_params_cb = autopid_data_read};
    obd_logger.period_sec = log_period;
    obd_logger.obd_logger_params = params;
    obd_logger.obd_logger_params_count = param_count;

    if (odb_logger_init(&obd_logger) != ESP_OK) { ESP_LOGE(TAG, "Failed to initialize OBD logger"); return; }

    for (size_t i = 0; i < param_count; i++) { free((void *)params[i].name); free((void *)params[i].metadata); }
    free(params);
    ESP_LOGI(TAG, "OBD logger initialized with %zu parameters", param_count);
}

void print_pids(autopid_config_t *autopid_config)
{
    const char *pid_type_str[] = {"Standard", "Custom", "Specific"};
    printf("Total PIDs: %lu\n", autopid_config->pid_count);
    printf("Custom Init: %s\n", autopid_config->custom_init);
    printf("Specific Init: %s\n", autopid_config->specific_init);

    for (int i = 0; i < autopid_config->pid_count; i++) {
        pid_data_t *pid = &autopid_config->pids[i];
        printf("\nPID %d:\n", i + 1);
        printf("  Type: %s\n", pid_type_str[pid->pid_type]);
        printf("  Command: %s\n", pid->cmd ? pid->cmd : "NULL");
        printf("  Init: %s\n", pid->init ? pid->init : "NULL");
        printf("  Period: %lu\n", pid->period);
        printf("  Parameter Count: %lu\n", pid->parameters_count);
        if (pid->parameters) {
            printf("  Parameters:\n");
            printf("    Name: %s\n", pid->parameters->name ? pid->parameters->name : "NULL");
            printf("    Expression: %s\n", pid->parameters->expression ? pid->parameters->expression : "NULL");
            printf("    Unit: %s\n", pid->parameters->unit ? pid->parameters->unit : "NULL");
            printf("    Class: %s\n", pid->parameters->class ? pid->parameters->class : "NULL");
            printf("    Period: %lu\n", pid->parameters->period);
            printf("     Destination: %s\n", pid->parameters->destination ? pid->parameters->destination : "NULL");
        }
        printf("-------------------\n");
    }
}

void autopid_app_reset_timer(void)
{
    if (autopid_bit_set_timer_handle != NULL) {
        if (xTimerReset(autopid_bit_set_timer_handle, 0) != pdPASS) ESP_LOGE(TAG, "Failed to reset autopid bit set timer");
        else ESP_LOGI(TAG, "Autopid bit set timer reset");
    }
}

static void autopid_app_setbit_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Timer callback called every 10 seconds");
    dev_status_set_bits(DEV_AUTOPID_ELM327_APP_BIT);
}

void autopid_init(char *id, bool enable_logging, uint32_t logging_period)
{
    device_id = id;
    if (xautopid_event_group == NULL) xautopid_event_group = xEventGroupCreateStatic(&xautopid_event_group_buffer);

    #if HARDWARE_VER == WICAN_PRO
    auto_pid_buf = (char *)heap_caps_malloc(AUTOPID_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (auto_pid_buf == NULL) { ESP_LOGE(TAG, "Failed to allocate auto_pid_buf in PSRAM"); return; }
    memset(auto_pid_buf, 0, AUTOPID_BUFFER_SIZE);

    elm327_autopid_cmd_buffer = (char *)heap_caps_malloc(AUTOPID_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (elm327_autopid_cmd_buffer == NULL) { ESP_LOGE(TAG, "Failed to allocate elm327_autopid_cmd_buffer in PSRAM"); return; }
    memset(elm327_autopid_cmd_buffer, 0, AUTOPID_BUFFER_SIZE);
    #else
    auto_pid_buf = (char *)malloc(AUTOPID_BUFFER_SIZE);
    #endif

    static StaticQueue_t autopidQueue_buffer;
    static uint8_t *autopidQueue_storage;
    autopidQueue_storage = (uint8_t *)heap_caps_malloc(QUEUE_SIZE * sizeof(response_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (autopidQueue_storage == NULL) { ESP_LOGE(TAG, "Failed to allocate queue storage in PSRAM"); return; }
    memset(autopidQueue_storage, 0, QUEUE_SIZE * sizeof(response_t));
    autopidQueue = xQueueCreateStatic(QUEUE_SIZE, sizeof(response_t), autopidQueue_storage, &autopidQueue_buffer);
    if (autopidQueue == NULL) { ESP_LOGE(TAG, "Failed to create static queue"); return; }

    static StaticQueue_t protocolQueue_buffer;
    static uint8_t protocolQueue_storage[1 * sizeof(int32_t)];
    protocolnumberQueue = xQueueCreateStatic(1, sizeof(int32_t), protocolQueue_storage, &protocolQueue_buffer);
    if (protocolnumberQueue == NULL) { ESP_LOGE(TAG, "Failed to create static protocol queue"); return; }
    autopid_set_protocol_number(-1);

    autopid_config = load_autopid_config();
    if (autopid_config) {
        autopid_config->mutex = xSemaphoreCreateMutex();
        if (!autopid_config->mutex) { ESP_LOGE(TAG, "Failed to create autopid_config mutex"); return; }
        autopid_data.mutex = xSemaphoreCreateMutex();
        if (!autopid_data.mutex) { ESP_LOGE(TAG, "Failed to create autopid_data mutex"); return; }
    } else {
        ESP_LOGE(TAG, "autopid_config is NULL");
        return;
    }

    if (autopid_config->pid_count == 0) { ESP_LOGE(TAG, "No PIDs found in car_data.json or auto_pid.json"); return; }
    (void)autopid_get_config();

    if (dev_status_is_bit_set(DEV_SDCARD_MOUNTED_BIT)) ESP_LOGI(TAG, "SD Card mounted"); else ESP_LOGI(TAG, "SD Card not mounted");

    if (enable_logging && dev_status_is_bit_set(DEV_SDCARD_MOUNTED_BIT)) {
        xSemaphoreTake(autopid_config->mutex, portMAX_DELAY);
        autopid_init_obd_logger(logging_period);
        xSemaphoreGive(autopid_config->mutex);
    }

    static StackType_t *autopid_task_stack;
    static StaticTask_t autopid_task_buffer;
    _Static_assert(sizeof(StackType_t) == 1, "Expected StackType_t to be 1 byte");
    static const size_t autopid_task_stack_depth = (1024 * 10) + (2 * sizeof(response_t));
    autopid_task_stack = heap_caps_malloc(autopid_task_stack_depth, MALLOC_CAP_SPIRAM);
    if (autopid_task_stack == NULL) { ESP_LOGE(TAG, "Failed to allocate autopid_task stack in PSRAM"); return; }
    memset(autopid_task_stack, 0, autopid_task_stack_depth);
    if (autopid_task_stack != NULL) {
        xTaskCreateStatic(autopid_task, "autopid_task", (uint32_t)autopid_task_stack_depth, (void *)AF_INET, 11, autopid_task_stack, &autopid_task_buffer);
    } else { ESP_LOGE(TAG, "Failed to allocate autopid_task stack in PSRAM"); }

    if (strcmp("enable", autopid_config->grouping) == 0) {
        static StackType_t *autopid_publish_task_stack;
        static StaticTask_t autopid_publish_task_buffer;
        static const size_t autopid_publish_task_stack_depth = (1024 * 8);
        autopid_publish_task_stack = heap_caps_malloc(autopid_publish_task_stack_depth, MALLOC_CAP_SPIRAM);
        if (autopid_publish_task_stack == NULL) { ESP_LOGE(TAG, "Failed to allocate autopid_publish_task stack in PSRAM"); }
        else {
            memset(autopid_publish_task_stack, 0, autopid_publish_task_stack_depth);
            if (xTaskCreateStatic(autopid_publish_task, "autopid_publish_task", (uint32_t)autopid_publish_task_stack_depth, NULL, 5, autopid_publish_task_stack, &autopid_publish_task_buffer) != NULL) ESP_LOGI(TAG, "Autopid publish task created");
            else ESP_LOGE(TAG, "Failed to create autopid_publish_task");
        }
    }


    // Publish the most recent data as soon as MQTT (re)connects, e.g. on reaching home Wi-Fi.
    {
        static StackType_t *mqtt_connect_task_stack;
        static StaticTask_t mqtt_connect_task_buffer;
        static const size_t mqtt_connect_task_stack_depth = (1024 * 6);
        mqtt_connect_task_stack = heap_caps_malloc(mqtt_connect_task_stack_depth, MALLOC_CAP_SPIRAM);
        if (mqtt_connect_task_stack == NULL) { ESP_LOGE(TAG, "Failed to allocate mqtt connect publish task stack"); }
        else if (xTaskCreateStatic(autopid_mqtt_connect_publish_task, "ap_mqtt_conn", (uint32_t)mqtt_connect_task_stack_depth, NULL, 5, mqtt_connect_task_stack, &mqtt_connect_task_buffer) == NULL) {
            ESP_LOGE(TAG, "Failed to create mqtt connect publish task");
        }
    }

    /* [NEW] Allocate memory and create the Processing Task (Lower Priority: 5) */
   uint32_t proc_stack_depth = 1024 * 12;
    autopid_processing_task_stack = heap_caps_malloc(proc_stack_depth, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    
    if (autopid_processing_task_stack != NULL) {
        autopid_processing_task_handle = xTaskCreateStatic(
            autopid_processing_task, 
            "pid_proc_task", 
            proc_stack_depth, 
            NULL, 
            5, // Priority 5 (matches web/system tasks)
            autopid_processing_task_stack, 
            &autopid_processing_task_buffer
        );
    }
    
    ha_webhooks_init();
    if (config_server_get_webhook_en()) {
        static StackType_t *autopid_webhook_task_stack;
        static StaticTask_t autopid_webhook_task_buffer;
        static const size_t autopid_webhook_task_stack_depth = (1024 * 20);
        autopid_webhook_task_stack = heap_caps_malloc(autopid_webhook_task_stack_depth, MALLOC_CAP_SPIRAM);
        if (autopid_webhook_task_stack == NULL) { ESP_LOGE(TAG, "Failed to allocate autopid_webhook_task stack in PSRAM"); }
        else {
            memset(autopid_webhook_task_stack, 0, autopid_webhook_task_stack_depth);
            if (xTaskCreateStatic(autopid_webhook_task, "autopid_webhook_task", (uint32_t)autopid_webhook_task_stack_depth, NULL, 5, autopid_webhook_task_stack, &autopid_webhook_task_buffer) != NULL) ESP_LOGI(TAG, "Autopid webhook task created");
            else ESP_LOGE(TAG, "Failed to create autopid_webhook_task");
        }
    } else { ESP_LOGI(TAG, "Webhook disabled in config, not starting autopid_webhook_task"); }

    if (dev_status_is_smartconnect_enabled()) {
        dev_status_clear_autopid_enabled();
        ESP_LOGI(TAG, "Autopid is controlled by SmartConnect, disabling autopid");
    } else {
        dev_status_set_autopid_enabled();
        ESP_LOGI(TAG, "Autopid enabled");
    }
    dev_status_set_bits(DEV_AUTOPID_ELM327_APP_BIT);

    if (autopid_bit_set_timer_handle == NULL) {
        autopid_bit_set_timer_handle = xTimerCreateStatic("autopid_bit_set_timer", pdMS_TO_TICKS(10000), pdTRUE, NULL, autopid_app_setbit_timer_callback, &autopid_bit_set_timer_buffer);
        if (autopid_bit_set_timer_handle != NULL) xTimerStart(autopid_bit_set_timer_handle, 0);
        else ESP_LOGE(TAG, "Failed to create static timer");
    }
}

static void autopid_processing_task(void *pvParameters) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Try to take the lock, but don't hang the whole system if sampler is busy
        if (autopid_lock(500)) { 
            autopid_data_update(autopid_config); // Build JSON
            
            // <--- UNLOCK EARLY! Don't hold the car polling hostage during slow HTTP posts.
            autopid_unlock(); 
            
            autopid_publish_all_destinations(true); // <--- [NEW] EVENT DRIVEN PUBLISH
            
            // Allow the system to breathe after heavy string processing
            vTaskDelay(1); 
        }
    }
}

/* [NEW] Function to toggle a group's state via MQTT with variable timeout */
void autopid_set_group_mqtt_state(const char* group_name, bool active_state, uint32_t timeout_mins) {
    if (!autopid_config || !autopid_config->use_groups || !group_name) return;
    
    if (autopid_lock(100)) {
        for (uint32_t i = 0; i < autopid_config->group_count; i++) {
            pid_group_t *grp = &autopid_config->groups[i];
            if (grp->name && strcmp(grp->name, group_name) == 0) {
                
                grp->mqtt_active_flag = active_state;
                
                /* [NEW] Handle variable timeout */
                if (active_state) {
                    if (timeout_mins == 0) {
                        grp->mqtt_timer_disabled = true;
                        ESP_LOGI(TAG, "Group '%s' MQTT state set to ACTIVE (Indefinite)", group_name);
                    } else {
                        grp->mqtt_timer_disabled = false;
                        // Convert minutes to milliseconds
                        wc_timer_set(&grp->mqtt_active_timer, timeout_mins * 60000); 
                        ESP_LOGI(TAG, "Group '%s' MQTT state set to ACTIVE (Timeout: %lu mins)", group_name, timeout_mins);
                    }
                } else {
                    ESP_LOGI(TAG, "Group '%s' MQTT state set to IDLE", group_name);
                }
                
                break; // Found and updated
            }
        }
        autopid_unlock();
    }
}
