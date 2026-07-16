#include <string.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"       
#include "driver/sdmmc_host.h"
#include "sdcard.h"
#include "hw_config.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_heap_caps.h"  
#include "hw_config.h"
#include "esp_littlefs.h"
#include "dev_status.h"
#include "filesystem.h"
#include "restart_tracker.h"
#include "esp_log.h"
#include "config_server.h"

#define OTA_BUFFER_SIZE 4096  
#define SDCARD_LOG_PATH SD_CARD_MOUNT_POINT "/wican.log"
#define SDCARD_LOG_BACKUP_PATH SD_CARD_MOUNT_POINT "/wican.log.1"
#define SDCARD_LOG_MAX_FILE_SIZE (1024 * 1024)
#define SDCARD_LOG_LINE_SIZE 1024
#define SDCARD_LOG_QUEUE_LENGTH 16
#define SDCARD_LOG_STOP_INDEX UINT8_MAX

static const char *TAG = "SDCARD";
#ifdef USE_SD_FATFS
static sdmmc_card_t *s_card = NULL;
#else
static sdmmc_card_t sdcard;
#endif
static bool s_card_mounted = false;

typedef struct
{
    uint16_t len;
    char text[SDCARD_LOG_LINE_SIZE];
} sdcard_log_entry_t;

static QueueHandle_t s_log_free_queue = NULL;
static QueueHandle_t s_log_pending_queue = NULL;
static sdcard_log_entry_t *s_log_entries = NULL;
static int (*s_previous_log_vprintf)(const char *format, va_list args) = NULL;
static atomic_bool s_log_enabled = false;
static atomic_bool s_log_task_running = false;
static atomic_uint s_log_callbacks_active = 0;
static atomic_uint s_log_dropped = 0;

static size_t sdcard_log_strip_ansi(char *output, size_t output_size,
                                   const char *input, size_t input_size)
{
    size_t output_len = 0;

    for (size_t i = 0; i < input_size && output_len + 1 < output_size; i++)
    {
        if ((unsigned char)input[i] == 0x1b && i + 1 < input_size && input[i + 1] == '[')
        {
            i += 2;
            while (i < input_size)
            {
                unsigned char c = (unsigned char)input[i];
                if (c >= 0x40 && c <= 0x7e)
                {
                    break;
                }
                i++;
            }
            continue;
        }

        output[output_len++] = input[i];
    }

    output[output_len] = '\0';
    return output_len;
}

static bool sdcard_log_ci_prefix_match(const char *text, const char *label, size_t label_len)
{
    for (size_t i = 0; i < label_len; i++)
    {
        char a = text[i];
        char b = label[i];
        if (a == '\0') return false;
        if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
        if (a != b) return false;
    }
    return true;
}

static char *sdcard_log_ci_find(const char *haystack, const char *needle)
{
    size_t needle_len = strlen(needle);
    if (needle_len == 0) return NULL;
    for (const char *p = haystack; *p != '\0'; p++)
    {
        if (sdcard_log_ci_prefix_match(p, needle, needle_len))
        {
            return (char *)p;
        }
    }
    return NULL;
}

static const char *const SDCARD_LOG_SECRET_LABELS[] = {
    "sta_pass:",
    "ap_pass:",
    "home_password:",
    "drive_password:",
    "ble_pass:",
    "mqtt_pass:",
    "batt_mqtt_pass:",
    "batt_alert_pass:",
    "ts_auth_key:",
    "private_key:",
    "preshared_key:",
    "api_token:",
    "api_key:",
    "bearer:",
    "basic_password:",
    /* Not a credential, but a network address -- for a self-hosted VPN
     * this reveals the router's public IP. Same protection mechanism,
     * different category of sensitive data. */
    "esp_wireguard: peer:",
    "esp_wireguard: connecting to ",
    "esp_wireguard: getaddrinfo: unable to resolve `",
};
#define SDCARD_LOG_SECRET_LABEL_COUNT (sizeof(SDCARD_LOG_SECRET_LABELS) / sizeof(SDCARD_LOG_SECRET_LABELS[0]))
#define SDCARD_LOG_REDACTED_MARKER "***REDACTED***"

/* Tries a literal case-insensitive match of `label` in `text`. If `label`
 * ends in ':', also tries the JSON-quoted form ("field":...) for the same
 * field name, since credential fields appear both in plain ESP_LOGx calls
 * and in the raw config.json dump line. Labels that don't end in ':'
 * (e.g. "esp_wireguard: connecting to ") are matched literally only --
 * used for values that are never JSON-quoted, like a logged network
 * endpoint rather than a config field. */
static char *sdcard_log_find_value_start(char *text, const char *label, bool *is_json_quoted)
{
    *is_json_quoted = false;

    size_t label_len = strlen(label);
    bool try_json = (label_len > 1 && label[label_len - 1] == ':');

    char *plain_match = sdcard_log_ci_find(text, label);
    char *json_match = NULL;
    char json_pattern[40];

    if (try_json)
    {
        char field_name[32];
        if (label_len - 1 < sizeof(field_name))
        {
            memcpy(field_name, label, label_len - 1);
            field_name[label_len - 1] = '\0';
            snprintf(json_pattern, sizeof(json_pattern), "\"%s\":", field_name);
            json_match = sdcard_log_ci_find(text, json_pattern);
        }
    }

    char *match;
    const char *pattern;
    if (json_match != NULL && (plain_match == NULL || json_match <= plain_match))
    {
        match = json_match;
        pattern = json_pattern;
        *is_json_quoted = true;
    }
    else if (plain_match != NULL)
    {
        match = plain_match;
        pattern = label;
        *is_json_quoted = false;
    }
    else
    {
        return NULL;
    }

    char *value_start = match + strlen(pattern);
    if (*value_start == ' ')
    {
        value_start++;
    }
    if (*is_json_quoted && *value_start == '"')
    {
        value_start++;
    }
    return value_start;
}

static void sdcard_log_redact_secrets(char *text, size_t buf_size)
{
    size_t marker_len = strlen(SDCARD_LOG_REDACTED_MARKER);

    for (size_t i = 0; i < SDCARD_LOG_SECRET_LABEL_COUNT; i++)
    {
        const char *label = SDCARD_LOG_SECRET_LABELS[i];
        bool is_json_quoted = false;
        char *search_ptr = text;

        while ((search_ptr = sdcard_log_find_value_start(search_ptr, label, &is_json_quoted)) != NULL)
        {
            char *value_end;
            if (is_json_quoted)
            {
                value_end = strchr(search_ptr, '"');
            }
            else
            {
                value_end = strchr(search_ptr, '\n');
            }

            if (value_end != NULL)
            {
                size_t tail_len = strlen(value_end) + 1;
                if ((size_t)(search_ptr - text) + marker_len + tail_len <= buf_size)
                {
                    memmove(search_ptr + marker_len, value_end, tail_len);
                    memcpy(search_ptr, SDCARD_LOG_REDACTED_MARKER, marker_len);
                    search_ptr += marker_len;
                }
                else
                {
                    size_t secret_len = value_end - search_ptr;
                    memset(search_ptr, '*', secret_len);
                    search_ptr += secret_len;
                }
            }
            else
            {
                if ((size_t)(search_ptr - text) + marker_len + 1 <= buf_size)
                {
                    memcpy(search_ptr, SDCARD_LOG_REDACTED_MARKER, marker_len);
                    search_ptr[marker_len] = '\0';
                    search_ptr += marker_len;
                }
                else
                {
                    size_t remaining = buf_size - (search_ptr - text) - 1;
                    memset(search_ptr, '*', remaining);
                    search_ptr[remaining] = '\0';
                    break;
                }
            }
        }
    }
}

static FILE *sdcard_log_open(void)
{
    FILE *file = fopen(SDCARD_LOG_PATH, "a");
    if (file == NULL)
    {
        return NULL;
    }

    if (fseek(file, 0, SEEK_END) == 0 && ftell(file) >= SDCARD_LOG_MAX_FILE_SIZE)
    {
        fclose(file);
        unlink(SDCARD_LOG_BACKUP_PATH);
        rename(SDCARD_LOG_PATH, SDCARD_LOG_BACKUP_PATH);
        file = fopen(SDCARD_LOG_PATH, "w");
    }

    return file;
}

static void sdcard_log_delete_files(void)
{
    if (unlink(SDCARD_LOG_PATH) == 0)
    {
        ESP_LOGI(TAG, "Deleted %s (debug log disabled)", SDCARD_LOG_PATH);
    }
    if (unlink(SDCARD_LOG_BACKUP_PATH) == 0)
    {
        ESP_LOGI(TAG, "Deleted %s (debug log disabled)", SDCARD_LOG_BACKUP_PATH);
    }
}

static void sdcard_log_task(void *arg)
{
    (void)arg;
    uint8_t index;
    bool stop_requested = false;
    char clean_line[SDCARD_LOG_LINE_SIZE];

    while (!stop_requested &&
           xQueueReceive(s_log_pending_queue, &index, portMAX_DELAY) == pdTRUE)
    {
        if (index == SDCARD_LOG_STOP_INDEX)
        {
            break;
        }

        FILE *file = sdcard_log_open();

        do
        {
            if (index == SDCARD_LOG_STOP_INDEX)
            {
                stop_requested = true;
                break;
            }

            if (index < SDCARD_LOG_QUEUE_LENGTH)
            {
                sdcard_log_entry_t *entry = &s_log_entries[index];
                if (file != NULL)
                {
                    size_t clean_len = sdcard_log_strip_ansi(
                        clean_line, sizeof(clean_line), entry->text, entry->len);
                    fwrite(clean_line, 1, clean_len, file);
                }
                (void)xQueueSend(s_log_free_queue, &index, portMAX_DELAY);
            }
        }
        while (xQueueReceive(s_log_pending_queue, &index, 0) == pdTRUE);

        if (file != NULL)
        {
            unsigned int dropped = atomic_exchange(&s_log_dropped, 0);
            if (dropped > 0)
            {
                fprintf(file, "W (SDCARD): Dropped %u SD log lines because the queue was full\n",
                        dropped);
            }
            fflush(file);
            fsync(fileno(file));
            fclose(file);
        }
    }

    atomic_store(&s_log_task_running, false);
    vTaskDelete(NULL);
}

static int sdcard_log_vprintf(const char *format, va_list args)
{
    va_list console_args;
    va_copy(console_args, args);
    int result = s_previous_log_vprintf != NULL
        ? s_previous_log_vprintf(format, console_args)
        : vprintf(format, console_args);
    va_end(console_args);

    atomic_fetch_add(&s_log_callbacks_active, 1);

    if (atomic_load(&s_log_enabled) && !xPortInIsrContext() &&
        s_log_free_queue != NULL && s_log_pending_queue != NULL && s_log_entries != NULL)
    {
        uint8_t index;
        if (xQueueReceive(s_log_free_queue, &index, 0) == pdTRUE)
        {
            sdcard_log_entry_t *entry = &s_log_entries[index];
            va_list file_args;
            va_copy(file_args, args);
            int length = vsnprintf(entry->text, sizeof(entry->text), format, file_args);
            va_end(file_args);

            if (length < 0)
            {
                (void)xQueueSend(s_log_free_queue, &index, 0);
            }
            else
            {
                sdcard_log_redact_secrets(entry->text, sizeof(entry->text));
                entry->len = (uint16_t)strnlen(entry->text, sizeof(entry->text) - 1);
                if (xQueueSend(s_log_pending_queue, &index, 0) != pdTRUE)
                {
                    (void)xQueueSend(s_log_free_queue, &index, 0);
                    atomic_fetch_add(&s_log_dropped, 1);
                }
            }
        }
        else
        {
            atomic_fetch_add(&s_log_dropped, 1);
        }
    }

    atomic_fetch_sub(&s_log_callbacks_active, 1);
    return result;
}

static esp_err_t sdcard_log_start(void)
{
    if (atomic_load(&s_log_enabled))
    {
        return ESP_OK;
    }

    s_log_entries = heap_caps_calloc(
        SDCARD_LOG_QUEUE_LENGTH, sizeof(sdcard_log_entry_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_log_free_queue = xQueueCreate(SDCARD_LOG_QUEUE_LENGTH, sizeof(uint8_t));
    s_log_pending_queue = xQueueCreate(SDCARD_LOG_QUEUE_LENGTH + 1, sizeof(uint8_t));

    if (s_log_entries == NULL || s_log_free_queue == NULL || s_log_pending_queue == NULL)
    {
        goto cleanup;
    }

    for (uint8_t i = 0; i < SDCARD_LOG_QUEUE_LENGTH; i++)
    {
        if (xQueueSend(s_log_free_queue, &i, 0) != pdTRUE)
        {
            goto cleanup;
        }
    }

    atomic_store(&s_log_task_running, true);
    if (xTaskCreate(sdcard_log_task, "sd_log", 4096, NULL, 2, NULL) != pdPASS)
    {
        atomic_store(&s_log_task_running, false);
        goto cleanup;
    }

    atomic_store(&s_log_dropped, 0);
    atomic_store(&s_log_enabled, true);
    s_previous_log_vprintf = esp_log_set_vprintf(sdcard_log_vprintf);
    return ESP_OK;

cleanup:
    if (s_log_free_queue != NULL)
    {
        vQueueDelete(s_log_free_queue);
        s_log_free_queue = NULL;
    }
    if (s_log_pending_queue != NULL)
    {
        vQueueDelete(s_log_pending_queue);
        s_log_pending_queue = NULL;
    }
    if (s_log_entries != NULL)
    {
        heap_caps_free(s_log_entries);
        s_log_entries = NULL;
    }
    return ESP_ERR_NO_MEM;
}

static esp_err_t sdcard_log_stop(void)
{
    bool was_enabled = atomic_exchange(&s_log_enabled, false);
    if (!was_enabled && s_log_free_queue == NULL)
    {
        return ESP_OK;
    }

    if (was_enabled && s_previous_log_vprintf != NULL)
    {
        esp_log_set_vprintf(s_previous_log_vprintf);
    }

    while (atomic_load(&s_log_callbacks_active) != 0)
    {
        taskYIELD();
    }

    if (was_enabled)
    {
        uint8_t stop_index = SDCARD_LOG_STOP_INDEX;
        if (xQueueSend(s_log_pending_queue, &stop_index, pdMS_TO_TICKS(1000)) != pdTRUE)
        {
            return ESP_ERR_TIMEOUT;
        }
    }

    TickType_t start = xTaskGetTickCount();
    while (atomic_load(&s_log_task_running))
    {
        if (xTaskGetTickCount() - start > pdMS_TO_TICKS(2000))
        {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vQueueDelete(s_log_free_queue);
    vQueueDelete(s_log_pending_queue);
    heap_caps_free(s_log_entries);
    s_log_free_queue = NULL;
    s_log_pending_queue = NULL;
    s_log_entries = NULL;
    s_previous_log_vprintf = NULL;
    return ESP_OK;
}

esp_err_t sdcard_perform_ota_update(const char* firmware_path)
{
    if (!sdcard_is_mounted() || !sdcard_is_available()) 
    {
        ESP_LOGE(TAG, "SD card not available for OTA update");
        return ESP_ERR_INVALID_STATE;
    }

    // Construct full path
    char full_path[128];
    snprintf(full_path, sizeof(full_path), "%s%s", SD_CARD_MOUNT_POINT, firmware_path);
    
    // Open firmware file
    FILE *firmware_file = fopen(full_path, "rb");
    if (firmware_file == NULL) 
    {
        ESP_LOGE(TAG, "Failed to open firmware file: %s", full_path);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Deleting config file before OTA update");

    // Delete config file if it exists
    ESP_LOGI(TAG, "Deleting config file before OTA update");

    // Delete config file if it exists
    filesystem_init();
    
    // Delete all configuration files
    filesystem_delete_config_files();

    ESP_LOGI(TAG, "Starting OTA from SD card file: %s", full_path);
    
    // Get file size
    struct stat file_stat;
    if (stat(full_path, &file_stat) != 0) 
    {
        ESP_LOGE(TAG, "Failed to get firmware file size");
        fclose(firmware_file);
        return ESP_FAIL;
    }
    
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) 
    {
        ESP_LOGE(TAG, "Failed to get OTA update partition");
        fclose(firmware_file);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Writing to partition: %s (size: %lu)", update_partition->label, update_partition->size);
    
    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, file_stat.st_size, &ota_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to begin OTA update: %s", esp_err_to_name(err));
        fclose(firmware_file);
        return err;
    }
    
    // Allocate buffer in internal RAM
    uint8_t *buffer = malloc(OTA_BUFFER_SIZE);
    if (buffer == NULL) 
    {
        ESP_LOGE(TAG, "Failed to allocate buffer in internal RAM");
        fclose(firmware_file);
        esp_ota_abort(ota_handle);
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "OTA buffer allocated in internal RAM");
    
    // Read and write firmware in chunks
    size_t bytes_read = 0;
    size_t total_bytes_read = 0;
    int last_percentage = -1;  // Track last percentage to avoid too many log messages
    
    while ((bytes_read = fread(buffer, 1, OTA_BUFFER_SIZE, firmware_file)) > 0) 
    {
        err = esp_ota_write(ota_handle, buffer, bytes_read);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to write OTA data: %s", esp_err_to_name(err));
            fclose(firmware_file);
            free(buffer);
            esp_ota_abort(ota_handle);
            return err;
        }
        
        total_bytes_read += bytes_read;
        
        // Calculate and print percentage progress
        int current_percentage = (total_bytes_read * 100) / file_stat.st_size;
        
        // Only print when percentage changes to avoid flooding the logs
        if (current_percentage != last_percentage) 
        {
            ESP_LOGI(TAG, "OTA progress: %d%% (%d/%d bytes)", 
                    current_percentage, 
                    total_bytes_read, 
                    (int)file_stat.st_size);
            last_percentage = current_percentage;
        }
    }
    
    // Free the allocated buffer
    free(buffer);
    fclose(firmware_file);
    
    // Finalize OTA update
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to finalize OTA update: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set new boot partition
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "OTA update successful, rebooting system...");
    
    // Optional: Add a delay before reboot to ensure logs are printed
    vTaskDelay(pdMS_TO_TICKS(1000));

    restart_tracker_restart(RESTART_TRACKER_PLANNED_REASON_OTA_APPLY,
                            RESTART_TRACKER_SOURCE_OTA,
                            RESTART_TRACKER_FLAG_FILESYSTEM_CHANGED | RESTART_TRACKER_FLAG_FIRMWARE_UPDATED);
    
    return ESP_OK;  // This will never be reached due to restart
}

esp_err_t sd_card_init(void) 
{
    if (s_card_mounted)
    {
        ESP_LOGI(TAG, "SD card already mounted");
        return ESP_OK;
    }

    esp_err_t ret;


    ESP_LOGI(TAG, "Initializing SD card");
    
    // Initialize SDMMC peripheral
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    // This initializes the slot without card detect (CD) and write protect (WP) signals
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    slot_config.width = 4; // 4-bit bus mode
    slot_config.clk = SDCARD_CLK;
    slot_config.cmd = SDCARD_CMD;
    slot_config.d0 = SDCARD_D0;
    slot_config.d1 = SDCARD_D1;
    slot_config.d2 = SDCARD_D2;
    slot_config.d3 = SDCARD_D3;

    // Mount the filesystem
    #ifdef USE_SD_FATFS
    // Mount configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = 
    {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 65536
    };

    // Mount the filesystem
    ret = esp_vfs_fat_sdmmc_mount(SD_CARD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);

    if (ret != ESP_OK) 
    {
        if (ret == ESP_FAIL) 
        {
            ESP_LOGE(TAG, "Failed to mount filesystem");
        } 
        else 
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
        // Print card info
        sdmmc_card_print_info(stdout, s_card);
        ESP_LOGI(TAG, "FAT filesystem mounted successfully");
    #else
        ESP_LOGI(TAG, "Initializing LittleFS filesystem");
        
        // Mount SD card
        ret = sdmmc_host_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize host: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize slot: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Card detection
        ret = sdmmc_card_init(&host, &sdcard);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
            return ret;
        }
        // Print card info
        ESP_LOGI(TAG, "SD card detected: Size: %lluMB", ((uint64_t)sdcard.csd.capacity * sdcard.csd.sector_size) / (1024 * 1024));
        
        esp_vfs_littlefs_conf_t conf = {
            .base_path = MOUNT_POINT,
            .partition_label = NULL,  // Not using internal flash partition
            .partition = NULL,        // Not using internal flash partition
            .sdcard = &sdcard,           // Using SD card
            .format_if_mount_failed = true,
            .dont_mount = false,
            .read_only = false,
            .grow_on_mount = true,
        };

        // Use settings defined above to initialize and mount LittleFS filesystem.
        ret = esp_vfs_littlefs_register(&conf);

        if (ret != ESP_OK) {
            if (ret == ESP_FAIL) {
                ESP_LOGE(TAG, "Failed to mount or format filesystem");
            } else if (ret == ESP_ERR_NOT_FOUND) {
                ESP_LOGE(TAG, "Failed to find LittleFS on SD card");
            } else {
                ESP_LOGE(TAG, "Failed to initialize LittleFS on SD card (%s)", esp_err_to_name(ret));
            }
            return ret;
        }

        size_t total = 0, used = 0;
        ret = esp_littlefs_sdmmc_info(&sdcard, &total, &used);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LittleFS SD card information (%s)", esp_err_to_name(ret));
            esp_littlefs_format_sdmmc(&sdcard);
        } else {
            ESP_LOGI(TAG, "LittleFS SD card info: total: %u bytes, used: %u bytes", total, used);
            ESP_LOGI(TAG, "SD card LittleFS partition size: total: %f MB, used: %f MB", (float)(total/(1024*1024)), (float)(used/(1024*1024)));
        }
    #endif

    s_card_mounted = true;
    dev_status_set_bits(DEV_SDCARD_MOUNTED_BIT);
    /* config.json hasn't been loaded yet at this point in boot -- sd_card_init()
     * runs before config_server_preload_config(), so device_config.sdcard_debug_log_en
     * isn't populated here and this decision can't be made correctly yet. See
     * sdcard_apply_debug_log_config(), called later once config is actually loaded. */
    ESP_LOGI(TAG, "SD card mounted successfully");
    
    return ESP_OK;
}

esp_err_t sdcard_apply_debug_log_config(void)
{
    if (!s_card_mounted)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (config_server_get_sdcard_debug_log_en())
    {
        esp_err_t log_ret = sdcard_log_start();
        if (log_ret != ESP_OK)
        {
            ESP_LOGW(TAG, "SD log capture unavailable: %s", esp_err_to_name(log_ret));
        }
        return log_ret;
    }
    else
    {
        ESP_LOGI(TAG, "SD debug logging disabled by config, skipping");
        sdcard_log_delete_files();
        return ESP_OK;
    }
}

esp_err_t sd_card_deinit(void) 
{
    if (!s_card_mounted) 
    {
        ESP_LOGI(TAG, "SD card not mounted");
        return ESP_OK;
    }

    esp_err_t log_ret = sdcard_log_stop();
    if (log_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to stop SD logging: %s", esp_err_to_name(log_ret));
        return log_ret;
    }

    // Unmount partition and disable SDMMC
    #ifdef USE_SD_FATFS
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(SD_CARD_MOUNT_POINT, s_card);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to unmount SD card");
        return ret;
    }
    s_card = NULL;
    #else
    esp_err_t ret = esp_vfs_littlefs_unregister(MOUNT_POINT);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to unmount LittleFS filesystem (%s)", esp_err_to_name(ret));
        return ret;
    }
    #endif
    s_card_mounted = false;
    dev_status_clear_bits(DEV_SDCARD_MOUNTED_BIT);
    
    ESP_LOGI(TAG, "SD card unmounted successfully");
    return ESP_OK;
}

bool sdcard_is_available(void) 
{
    #ifdef USE_SD_FATFS
    if (!s_card_mounted || s_card == NULL) 
    {
        return false;
    }
    #else
    if (!s_card_mounted) 
    {
        return false;
    }
    #endif

    // Just check if we can get the card status
    #ifdef USE_SD_FATFS
    esp_err_t err = sdmmc_get_status(s_card);
    #else
    esp_err_t err = sdmmc_get_status(&sdcard);
    #endif
    if (err != ESP_OK) 
    {
        ESP_LOGW(TAG, "Failed to get card status");
        return false;
    }

    return true;
}

const char* sdcard_get_mount_point(void) 
{
    return SD_CARD_MOUNT_POINT;
}

bool sdcard_is_mounted(void)
{
    return s_card_mounted;
}

esp_err_t sdcard_get_info(sdmmc_card_info_t *info) 
{
    #ifdef USE_SD_FATFS
    if (!s_card_mounted || !sdcard_is_available() || s_card == NULL) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    info->capacity = (((uint64_t) s_card->csd.capacity) * s_card->csd.sector_size) / (1024 * 1024);
    info->sector_size = s_card->csd.sector_size;
    info->speed = s_card->max_freq_khz;
    info->bus_width = s_card->host.slot;
    
    // Add card name
    strncpy(info->name, s_card->cid.name, sizeof(info->name) - 1);
    info->name[sizeof(info->name) - 1] = '\0';
    
    // Add card type
    if (s_card->is_sdio) {
        info->type = CARD_TYPE_SDIO;
    } else if (s_card->is_mmc) {
        info->type = CARD_TYPE_MMC;
    } else {
        info->type = (s_card->ocr & (1 << 30)) ? CARD_TYPE_SDHC : CARD_TYPE_SDSC;
    }
    #else
    if (!s_card_mounted || !sdcard_is_available()) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    info->capacity = (((uint64_t) sdcard.csd.capacity) * sdcard.csd.sector_size) / (1024 * 1024);
    info->sector_size = sdcard.csd.sector_size;
    info->speed = sdcard.max_freq_khz;
    info->bus_width = sdcard.host.slot;
    
    // Add card name
    strncpy(info->name, sdcard.cid.name, sizeof(info->name) - 1);
    info->name[sizeof(info->name) - 1] = '\0';
    
    // Add card type
    if (sdcard.is_sdio) {
        info->type = CARD_TYPE_SDIO;
    } else if (sdcard.is_mmc) {
        info->type = CARD_TYPE_MMC;
    } else {
        info->type = (sdcard.ocr & (1 << 30)) ? CARD_TYPE_SDHC : CARD_TYPE_SDSC;
    }
    #endif

    return ESP_OK;
}


esp_err_t sdcard_test_rw(void)
{
    if (!sdcard_is_available())
    {
        ESP_LOGE(TAG, "SD card not available for testing");
        return ESP_ERR_INVALID_STATE;
    }

    const char *test_path = SD_CARD_MOUNT_POINT "/test.txt";
    const char *test_content = "WiCAN SD Card Test";
    char read_buffer[32] = {0};
    
    // Test write
    FILE *f = fopen(test_path, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    
    fprintf(f, "%s", test_content);
    fclose(f);
    
    // Test read
    f = fopen(test_path, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    
    size_t bytes_read = fread(read_buffer, 1, strlen(test_content), f);
    fclose(f);
    
    // Verify content
    if (bytes_read != strlen(test_content) || 
        strcmp(read_buffer, test_content) != 0)
    {
        ESP_LOGE(TAG, "Read/write verification failed");
        return ESP_FAIL;
    }
    
    // Clean up test file
    unlink(test_path);
    
    ESP_LOGI(TAG, "SD card read/write test passed");
    return ESP_OK;
}