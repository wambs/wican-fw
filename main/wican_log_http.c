/*
 * wican_log_http.c
 *
 * Adds two HTTP routes for the SD-card debug log written by sdcard.c
 * (sdcard_log_start/_stop):
 *
 *   GET    /download_log   -> streams /sdcard/wican.log as a file download
 *   DELETE /download_log   -> deletes /sdcard/wican.log (and .log.1 backup)
 *
 * Add this file to main/CMakeLists.txt SRCS.
 * Register the routes in config_server.c near the other
 * httpd_register_uri_handler() calls inside register_server_uris():
 *
 *     extern const httpd_uri_t download_log_uri;
 *     extern const httpd_uri_t delete_log_uri;
 *     httpd_register_uri_handler(server, &download_log_uri);
 *     httpd_register_uri_handler(server, &delete_log_uri);
 *
 * NOTE: SDCARD_LOG_PATH / SDCARD_LOG_BACKUP_PATH are private #defines in
 * sdcard.c, so they're duplicated here. Cleaner long-term: move those two
 * #defines into sdcard.h and #include it instead.
 */

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"

#include "sdcard.h"

static const char *TAG = "WICAN_LOG_HTTP";

#define SDCARD_LOG_PATH        SD_CARD_MOUNT_POINT "/wican.log"
#define SDCARD_LOG_BACKUP_PATH SD_CARD_MOUNT_POINT "/wican.log.1"
#define LOG_DL_CHUNK_SIZE      (4 * 1024)

static esp_err_t download_log_get_handler(httpd_req_t *req);
static esp_err_t download_log_delete_handler(httpd_req_t *req);

const httpd_uri_t download_log_uri = {
    .uri     = "/download_log",
    .method  = HTTP_GET,
    .handler = download_log_get_handler,
    .user_ctx = NULL
};

const httpd_uri_t delete_log_uri = {
    .uri     = "/download_log",
    .method  = HTTP_DELETE,
    .handler = download_log_delete_handler,
    .user_ctx = NULL
};

static esp_err_t download_log_get_handler(httpd_req_t *req)
{
    if (!sdcard_is_mounted() || !sdcard_is_available())
    {
        ESP_LOGW(TAG, "Download requested but SD card is not mounted");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_send(req, "SD card not mounted", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    int file = open(SDCARD_LOG_PATH, O_RDONLY);
    if (file < 0)
    {
        ESP_LOGW(TAG, "wican.log not found at %s", SDCARD_LOG_PATH);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    struct stat file_stat;
    if (fstat(file, &file_stat) == 0)
    {
        ESP_LOGI(TAG, "Sending wican.log, size: %ld bytes", (long)file_stat.st_size);
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=wican.log");

    void *chunk = heap_caps_malloc(LOG_DL_CHUNK_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (chunk == NULL)
    {
        chunk = malloc(LOG_DL_CHUNK_SIZE);
    }
    if (chunk == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate send buffer");
        close(file);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ssize_t read_bytes;
    esp_err_t ret = ESP_OK;
    do
    {
        read_bytes = read(file, chunk, LOG_DL_CHUNK_SIZE);
        if (read_bytes > 0)
        {
            if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK)
            {
                ESP_LOGE(TAG, "File sending failed, aborting");
                ret = ESP_FAIL;
                break;
            }
        }
        else if (read_bytes < 0)
        {
            ESP_LOGE(TAG, "Failed to read wican.log");
            ret = ESP_FAIL;
            break;
        }
    } while (read_bytes > 0);

    free(chunk);
    close(file);

    /* Terminate the chunked response */
    httpd_resp_send_chunk(req, NULL, 0);

    return ret;
}

static esp_err_t download_log_delete_handler(httpd_req_t *req)
{
    if (!sdcard_is_mounted() || !sdcard_is_available())
    {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_send(req, "SD card not mounted", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    bool removed_any = false;

    if (unlink(SDCARD_LOG_PATH) == 0)
    {
        ESP_LOGI(TAG, "Deleted %s", SDCARD_LOG_PATH);
        removed_any = true;
    }
    if (unlink(SDCARD_LOG_BACKUP_PATH) == 0)
    {
        ESP_LOGI(TAG, "Deleted %s", SDCARD_LOG_BACKUP_PATH);
        removed_any = true;
    }

    if (!removed_any)
    {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"deleted\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
