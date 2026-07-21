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

#include "autopid_pause.h"

#include <math.h>
#include "dev_status.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "imu.h"
#include "restart_tracker.h"
#include "sleep_mode.h"
#include "wc_timer.h"

#define TAG "AUTOPID_PAUSE"
#define AUTOPID_PAUSE_IMU_STATIONARY_HOLD_MS (30 * 1000)
#define AUTOPID_PAUSE_VOLTAGE_RISE_DELTA_V 0.20f
#define AUTOPID_PAUSE_VOLTAGE_RISE_DEFAULT_TIME_SECONDS 20
#define AUTOPID_PAUSE_VOLTAGE_RISE_HOLD_MS (60 * 1000)
#define AUTOPID_PAUSE_VOLTAGE_RISE_MIN_SAMPLES 3
#define AUTOPID_PAUSE_VOLTAGE_HISTORY_SIZE 32

typedef struct
{
    int64_t timestamp_ms;
    float voltage;
} voltage_history_sample_t;

typedef enum
{
    SUPPLY_TRIGGER_NONE = 0,
    SUPPLY_TRIGGER_BOOT,
    SUPPLY_TRIGGER_IMU,
    SUPPLY_TRIGGER_VOLTAGE_RISE,
} supply_trigger_t;

static bool motion_was_active = false;
static wc_timer_t imu_stationary_hold_timer = 0;
static voltage_history_sample_t voltage_history[AUTOPID_PAUSE_VOLTAGE_HISTORY_SIZE] = {0};
static uint8_t voltage_history_next = 0;
static uint8_t voltage_history_count = 0;
static int64_t voltage_history_last_sample_ms = 0;
static wc_timer_t voltage_rising_hold_timer = 0;
static supply_trigger_t supply_trigger = SUPPLY_TRIGGER_NONE;
static autopid_pause_pid_polling_state_t pid_polling_state = {
    .paused = false,
    .reason = NULL,
    .voltage = NAN,
};

static bool pid_override_mode(const autopid_config_t *config)
{
    if (!config ||
        !config->supply_mode_enabled ||
        !config->supply_mode_pid_name ||
        config->supply_mode_pid_name[0] == '\0' ||
        config->supply_mode_operator[0] == '\0')
    {
        return false;
    }

    return autopid_supply_mode_is_active();
}

bool autopid_pause_is_boot_pid_polling_keep_alive_active(const autopid_config_t *config)
{
    if (!config || config->boot_pid_polling_keep_alive_seconds == 0)
    {
        return false;
    }

    int64_t uptime_us = esp_timer_get_time();
    int64_t keep_alive_us = (int64_t)config->boot_pid_polling_keep_alive_seconds * 1000000LL;
    bool within_boot_threshold = uptime_us < keep_alive_us;

    /* A wake from sleep is implemented as a full software reboot
     * (see sleep_mode.c, RESTART_TRACKER_PLANNED_REASON_POWER_WAKE),
     * which resets the uptime timer. Without this guard the boot
     * keep-alive window re-opens on every wake, so with the battery
     * near the sleep threshold the polling load can pull the voltage
     * back under it, causing a poll -> sag -> sleep -> wake -> poll
     * oscillation. Skip the boot window for wake-from-sleep reboots;
     * the voltage-rise and motion triggers remain the intended resume
     * paths after sleep. If the tracker has no valid record, fall
     * through to the existing behavior. Only checked when we're still
     * inside the boot window, to avoid the extra call once it's expired. */
    if (within_boot_threshold)
    {
        restart_tracker_record_t latest_record;
        if (restart_tracker_get_latest_record(&latest_record) == ESP_OK &&
            latest_record.was_planned &&
            latest_record.planned_reason == RESTART_TRACKER_PLANNED_REASON_POWER_WAKE)
        {
            return false;
        }
    }

    return within_boot_threshold;
}

static uint32_t voltage_rise_time_seconds_or_default(uint32_t rise_time_seconds)
{
    return rise_time_seconds > 0
        ? rise_time_seconds
        : AUTOPID_PAUSE_VOLTAGE_RISE_DEFAULT_TIME_SECONDS;
}

static int64_t voltage_rise_sample_interval_ms(uint32_t rise_time_seconds)
{
    uint32_t seconds = voltage_rise_time_seconds_or_default(rise_time_seconds);
    int64_t interval_ms = ((int64_t)seconds * 1000LL) / 10LL;
    return interval_ms > 0 ? interval_ms : 1;
}

static void voltage_history_record(float voltage, int64_t now_ms, int64_t sample_interval_ms)
{
    if (isnan(voltage))
    {
        return;
    }

    if (voltage_history_count > 0 &&
        (now_ms - voltage_history_last_sample_ms) < sample_interval_ms)
    {
        return;
    }

    voltage_history[voltage_history_next].timestamp_ms = now_ms;
    voltage_history[voltage_history_next].voltage = voltage;
    voltage_history_next = (voltage_history_next + 1) % AUTOPID_PAUSE_VOLTAGE_HISTORY_SIZE;
    if (voltage_history_count < AUTOPID_PAUSE_VOLTAGE_HISTORY_SIZE)
    {
        voltage_history_count++;
    }
    voltage_history_last_sample_ms = now_ms;
}

static void sort_float_values(float *values, uint8_t count)
{
    for (uint8_t i = 1; i < count; i++)
    {
        float value = values[i];
        uint8_t j = i;

        while (j > 0 && values[j - 1] > value)
        {
            values[j] = values[j - 1];
            j--;
        }

        values[j] = value;
    }
}

static bool voltage_history_window_median(int64_t now_ms, int64_t min_age_ms, int64_t max_age_ms, float *median, uint8_t *sample_count)
{
    float values[AUTOPID_PAUSE_VOLTAGE_HISTORY_SIZE];
    uint8_t count = 0;

    for (uint8_t i = 0; i < voltage_history_count; i++)
    {
        int64_t age_ms = now_ms - voltage_history[i].timestamp_ms;
        if (age_ms >= min_age_ms && age_ms <= max_age_ms && !isnan(voltage_history[i].voltage))
        {
            values[count++] = voltage_history[i].voltage;
        }
    }

    if (sample_count)
    {
        *sample_count = count;
    }

    if (count < AUTOPID_PAUSE_VOLTAGE_RISE_MIN_SAMPLES)
    {
        return false;
    }

    sort_float_values(values, count);

    if ((count % 2) == 0)
    {
        *median = (values[(count / 2) - 1] + values[count / 2]) / 2.0f;
    }
    else
    {
        *median = values[count / 2];
    }

    return true;
}

static bool voltage_rising_bypass_active(int64_t now_ms, float threshold_v, uint32_t rise_time_seconds)
{
    if (voltage_rising_hold_timer != 0 && !wc_timer_is_expired(&voltage_rising_hold_timer))
    {
        return true;
    }

    uint32_t seconds = voltage_rise_time_seconds_or_default(rise_time_seconds);
    int64_t current_max_age_ms = ((int64_t)seconds * 1000LL) / 2LL;
    int64_t baseline_min_age_ms = (int64_t)seconds * 1000LL;
    int64_t baseline_max_age_ms = ((int64_t)seconds * 3LL * 1000LL) / 2LL;
    float current_median = NAN;
    float baseline_median = NAN;
    uint8_t current_samples = 0;
    uint8_t baseline_samples = 0;

    bool have_current = voltage_history_window_median(now_ms,
                                                      0,
                                                      current_max_age_ms,
                                                      &current_median,
                                                      &current_samples);
    bool have_baseline = voltage_history_window_median(now_ms,
                                                       baseline_min_age_ms,
                                                       baseline_max_age_ms,
                                                       &baseline_median,
                                                       &baseline_samples);

    if (!have_current || !have_baseline)
    {
        return false;
    }

    float delta = current_median - baseline_median;
    if (delta > threshold_v)
    {
        uint64_t hold_ms = AUTOPID_PAUSE_VOLTAGE_RISE_HOLD_MS;
        wc_timer_set(&voltage_rising_hold_timer, hold_ms);
        ESP_LOGI(TAG,
                 "Voltage rising bypass: current %.2fV (%u samples) baseline %.2fV (%u samples), delta %.2fV threshold %.2fV over %lus hold %lus",
                 current_median,
                 (unsigned int)current_samples,
                 baseline_median,
                 (unsigned int)baseline_samples,
                 delta,
                 threshold_v,
                 (unsigned long)seconds,
                 (unsigned long)(hold_ms / 1000ULL));
        return true;
    }

    return false;
}

static bool evaluate_pid_polling_pause(const autopid_config_t *config, float *out_voltage, const char **out_reason)
{
    if (out_voltage)
        *out_voltage = NAN;
    if (out_reason)
        *out_reason = NULL;

    if (!config)
    {
        if (out_reason)
            *out_reason = "not_configured";
        return false;
    }

    bool boot_keep_alive_active = autopid_pause_is_boot_pid_polling_keep_alive_active(config);

    if (pid_override_mode(config))
    {
        float v = NAN;
        bool above_automate_threshold = false;

        if (config->disable_pid_requests_on_automate_threshold &&
            sleep_mode_get_voltage(&v) == ESP_OK)
        {
            if (out_voltage)
                *out_voltage = v;

            above_automate_threshold = (v >= config->pid_polling_min_voltage);
        }

        if (out_reason)
        {
            if (supply_trigger == SUPPLY_TRIGGER_BOOT)
            {
                *out_reason = "PID override: boot";
            }
            else if (above_automate_threshold)
            {
                *out_reason = "PID override: Over threshold trigger";
            }
            else if (supply_trigger == SUPPLY_TRIGGER_IMU)
            {
                *out_reason = "PID override: Motion trigger";
            }
            else if (supply_trigger == SUPPLY_TRIGGER_VOLTAGE_RISE)
            {
                *out_reason = "PID override: V rise trigger";
            }
            else
            {
                *out_reason = "PID override";
            }
        }
        return false;
    }

    supply_trigger = SUPPLY_TRIGGER_NONE;

    if (boot_keep_alive_active)
    {
        supply_trigger = SUPPLY_TRIGGER_BOOT;
        if (out_reason)
            *out_reason = "boot";
        return false;
    }

    if (config->imu_voltage_override_enabled)
    {
        activity_state_t imu_state = imu_get_activity_state();
        if (imu_state == ACTIVITY_STATE_ACTIVE)
        {
            motion_was_active = true;
            supply_trigger = SUPPLY_TRIGGER_IMU;
            if (out_reason)
                *out_reason = "imu_active";
            return false;
        }
    }
    else
    {
        motion_was_active = false;
        imu_stationary_hold_timer = 0;
    }

    float v = NAN;
    bool have_voltage = (sleep_mode_get_voltage(&v) == ESP_OK);
    if (have_voltage)
    {
        int64_t now_ms = esp_timer_get_time() / 1000LL;

        if (out_voltage)
            *out_voltage = v;

        voltage_history_record(v, now_ms, voltage_rise_sample_interval_ms(config->voltage_rise_time_seconds));
        if (config->voltage_rise_wakeup_enabled &&
            voltage_rising_bypass_active(now_ms,
                                         config->voltage_rise_threshold > 0.0f
                                             ? config->voltage_rise_threshold
                                             : AUTOPID_PAUSE_VOLTAGE_RISE_DELTA_V,
                                         config->voltage_rise_time_seconds))
        {
            if (out_reason)
                *out_reason = "voltage_rise";
            supply_trigger = SUPPLY_TRIGGER_VOLTAGE_RISE;
            return false;
        }
        else if (!config->voltage_rise_wakeup_enabled)
        {
            voltage_rising_hold_timer = 0;
        }
    }

    // Mode: disable PID requests when below Power Saving -> Sleep Voltage threshold.
    // Uses dev_status voltage bit (set by sleep_mode task).
    if (config->disable_pid_requests_on_sleep_voltage && !dev_status_is_wake_voltage_ok())
    {
        if (out_reason)
            *out_reason = "sleep_voltage";
        return true;
    }

    // Mode: disable PID requests when below a custom Automate threshold.
    if (config->disable_pid_requests_on_automate_threshold)
    {
        if (have_voltage)
        {
            if (v < config->pid_polling_min_voltage)
            {
                if (out_reason)
                    *out_reason = "automate_threshold";
                return true;
            }
        }
    }

    if (out_reason)
        *out_reason = "none_found";
    return false;
}

bool autopid_pause_should_pause_pid_polling(const autopid_config_t *config, float *out_voltage, const char **out_reason)
{
    float voltage = NAN;
    const char *reason = NULL;
    bool paused = evaluate_pid_polling_pause(config, &voltage, &reason);

    pid_polling_state.paused = paused;
    pid_polling_state.reason = reason;
    pid_polling_state.voltage = voltage;

    if (out_voltage)
        *out_voltage = voltage;
    if (out_reason)
        *out_reason = reason;

    return paused;
}

autopid_pause_pid_polling_state_t autopid_pause_get_pid_polling_state(void)
{
    return pid_polling_state;
}
