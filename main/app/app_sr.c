/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <sys/queue.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "app_sr.h"
#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_models.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_mn_iface.h"
#include "model_path.h"
#include "bsp_board.h"
#include "app_audio.h"
#include "app_wifi.h"
#include "app_sr.h"
#include "esp_mn_iface.h"       // esp_mn_get_iface_from_name
#include "app_sr_cmds.h" 
#include "robot_motion.h"
static const char *TAG = "app_sr";

static int mn_chunk = 0;   // 加在文件顶部或静态变量区
static esp_afe_sr_iface_t *afe_handle = NULL;
static srmodel_list_t *models = NULL;
static bool manul_detect_flag = false;

sr_data_t *g_sr_data = NULL;

#define I2S_CHANNEL_NUM      2

extern bool record_flag;
extern uint32_t record_total_len;

static void audio_feed_task(void *arg)
{
    ESP_LOGI(TAG, "Feed Task");
    size_t bytes_read = 0;
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *) arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int feed_channel = 3;
    ESP_LOGI(TAG, "audio_chunksize=%d, feed_channel=%d", audio_chunksize, feed_channel);

    /* Allocate audio buffer and check for result */
    int16_t *audio_buffer = heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(audio_buffer);
    g_sr_data->afe_in_buffer = audio_buffer;

    while (true) {
        if (g_sr_data->event_group && xEventGroupGetBits(g_sr_data->event_group)) {
            xEventGroupSetBits(g_sr_data->event_group, FEED_DELETED);
            vTaskDelete(NULL);
        }

        /* Read audio data from I2S bus */
        bsp_i2s_read((char *)audio_buffer, audio_chunksize * I2S_CHANNEL_NUM * sizeof(int16_t), &bytes_read, portMAX_DELAY);

        /* Channel Adjust */
        for (int  i = audio_chunksize - 1; i >= 0; i--) {
            audio_buffer[i * 3 + 2] = 0;
            audio_buffer[i * 3 + 1] = audio_buffer[i * 2 + 1];
            audio_buffer[i * 3 + 0] = audio_buffer[i * 2 + 0];
        }

        /* Checking if WIFI is connected */
        if (WIFI_STATUS_CONNECTED_OK == wifi_connected_already()) {

            /* Feed samples of an audio stream to the AFE_SR */
            afe_handle->feed(afe_data, audio_buffer);
        }
        audio_record_save(audio_buffer, audio_chunksize);
    }
}

static void audio_detect_task(void *arg)
{
    ESP_LOGI(TAG, "Detection task");
    static afe_vad_state_t local_state;
    static uint8_t frame_keep = 0;

    esp_afe_sr_data_t *afe_data = arg;

    /* 第一次进来时，记录 multinet 每帧样本数 */
    if (mn_chunk == 0 && g_sr_data->multinet) {
        mn_chunk = g_sr_data->multinet->get_samp_chunksize(g_sr_data->model_data);
        ESP_LOGI(TAG, "MultiNet chunk = %d samples", mn_chunk);
    }

    bool detect_flag = false;

    while (true) {
        if (NEED_DELETE && xEventGroupGetBits(g_sr_data->event_group)) {
            xEventGroupSetBits(g_sr_data->event_group, DETECT_DELETED);
            vTaskDelete(g_sr_data->handle_task);
            vTaskDelete(NULL);
        }

        /* ---------- AFE fetch ---------- */
        afe_fetch_result_t *res = g_sr_data->afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            continue;
        }

        /* ---------- 1. 唤醒词 ---------- */
        if (res->wakeup_state == WAKENET_DETECTED) {
            sr_result_t wk = {
                .wakenet_mode = WAKENET_DETECTED,
                .state        = ESP_MN_STATE_DETECTING,
            };
            xQueueSend(g_sr_data->result_que, &wk, 0);
        } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED && !detect_flag) {
            detect_flag = true;
            g_sr_data->afe_handle->disable_wakenet(afe_data);   // 关唤醒
            frame_keep = 0;
            ESP_LOGI(TAG, "Channel verified, ch=%d", res->trigger_channel_id);
        }

        /* ---------- 2. 指令检测 ---------- */
        if (!detect_flag) {
            continue;   // 还没进入检测阶段
        }

        /* 2-1 监控 VAD：100 帧静音 → 超时 */
        if (local_state != res->vad_state) {
            local_state = res->vad_state;
            frame_keep  = 0;
        } else {
            frame_keep++;
        }
        if (frame_keep >= 100 && res->vad_state == AFE_VAD_SILENCE) {
            sr_result_t to = {
                .wakenet_mode = WAKENET_NO_DETECT,
                .state        = ESP_MN_STATE_TIMEOUT,
            };
            xQueueSend(g_sr_data->result_que, &to, 0);
            g_sr_data->afe_handle->enable_wakenet(afe_data);
            g_sr_data->multinet->clean(g_sr_data->model_data);
            detect_flag = false;
            continue;
        }

        /* 2-2 把 AFE 输出帧送给 multinet->detect() */
        if (res->data && res->data_size >= mn_chunk * 2) {      // 一帧 16-bit ⇒ ×2
            g_sr_data->multinet->detect(g_sr_data->model_data, res->data);

            esp_mn_results_t *mn_res = g_sr_data->multinet->get_results(g_sr_data->model_data);
            if (mn_res && mn_res->state == ESP_MN_STATE_DETECTED && mn_res->num > 0) {

                int cmd_id   = mn_res->command_id[0];          // 最优结果
                int cmd_prob = (int)(mn_res->prob[0] * 100);   // (0-1)→百分数
                ESP_LOGI(TAG, "[MN] id=%d  prob=%d", cmd_id, cmd_prob);

                sr_result_t ok = {
                    .wakenet_mode = WAKENET_NO_DETECT,
                    .state        = ESP_MN_STATE_DETECTED,
                    .command_id   = cmd_id,
                };
                xQueueSend(g_sr_data->result_que, &ok, 0);

                /* 收到指令 → 复位并回到唤醒态 */
                g_sr_data->afe_handle->enable_wakenet(afe_data);
                g_sr_data->multinet->clean(g_sr_data->model_data);
                detect_flag = false;
            }
        }
    }
    /* never return */
}

esp_err_t app_sr_set_language(sr_language_t new_lang)
{
    ESP_RETURN_ON_FALSE(g_sr_data,            ESP_ERR_INVALID_STATE, TAG, "SR not running");
    ESP_RETURN_ON_FALSE(new_lang < SR_LANG_MAX, ESP_ERR_INVALID_ARG,   TAG, "lang invalid");

    if (new_lang == g_sr_data->lang) {
        ESP_LOGW(TAG, "language unchanged");
        return ESP_OK;
    }
    /* ---------- 1. 清理旧模型 & 命令 ---------- */
    ESP_ERROR_CHECK(app_sr_remove_all_cmd());
    if (strstr(g_sr_data->mn_name ?: "", "mn6")) {
                esp_mn_commands_clear();
    }

    g_sr_data->cmd_num = 49;

    free(g_sr_data->mn_name);          // 释放上一轮 strdup 的名字

    g_sr_data->lang = new_lang;
    ESP_LOGI(TAG, "Set language → %s", new_lang == SR_LANG_EN ? "EN" : "CN");

    /* ---------- 2. 切 Wakenet ---------- */
    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "");
    ESP_LOGI(TAG, "load wakenet:%s", wn_name);
    g_sr_data->afe_handle->set_wakenet(g_sr_data->afe_data, wn_name);

    /* ---------- 3. 创建 Multinet (v1.4.4 API) ---------- */
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX,
                              new_lang == SR_LANG_EN ? ESP_MN_ENGLISH : ESP_MN_CHINESE);
    ESP_RETURN_ON_FALSE(mn_name, ESP_ERR_NOT_FOUND, TAG, "no multinet");

    esp_mn_iface_t *mn_iface = esp_mn_handle_from_name(mn_name);
    ESP_RETURN_ON_FALSE(mn_iface, ESP_ERR_NOT_FOUND, TAG, "iface null");

    /* 第二个参数是内部环形缓冲区尺寸（字节）。官方 sample 用 5760，能跑 16 kHz，80-ms。*/
    model_iface_data_t *model_data = mn_iface->create(mn_name, 5760);
    ESP_RETURN_ON_FALSE(model_data, ESP_FAIL, TAG, "create mn fail");

    g_sr_data->multinet  = mn_iface;
    g_sr_data->model_data = model_data;
    g_sr_data->mn_name    = strdup(mn_name);   // 留给 strstr()

    ESP_LOGI(TAG, "load multinet: %s", mn_name);

    /* ---------- 4. 重新注册命令 ---------- */
    const sr_cmd_t *cmd_table;          // 这里只示例英文
    size_t          cmd_cnt;

    if (new_lang == SR_LANG_EN) {
               cmd_table = g_robot_cmds_en;          /* 只含 5 条 */
               cmd_cnt   = ROBOT_CMDS_EN_COUNT;      /* =5 */
    
    }

    for (size_t i = 0; i < cmd_cnt; i++) {
                ESP_ERROR_CHECK(app_sr_add_cmd(&cmd_table[i]));     /* cmd_num++ 内部自增 */
    }
    ESP_ERROR_CHECK(app_sr_update_cmds());

    /* ---------- 5. 更新到 Multinet ---------- */
    return app_sr_update_cmds();      // 里面会做 esp_mn_commands_update()
    return ESP_OK;
}

esp_err_t app_sr_remove_all_cmd(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    sr_cmd_t *it;
    while (!SLIST_EMPTY(&g_sr_data->cmd_list)) {
        it = SLIST_FIRST(&g_sr_data->cmd_list);
        SLIST_REMOVE_HEAD(&g_sr_data->cmd_list, next);
        heap_caps_free(it);
    }
    SLIST_INIT(&g_sr_data->cmd_list);
    return ESP_OK;
}

esp_err_t app_sr_start(bool record_en)
{   esp_log_level_set("*", ESP_LOG_INFO); 
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(NULL == g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR already running");

    g_sr_data = heap_caps_calloc(1, sizeof(sr_data_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_NO_MEM, TAG, "Failed create sr data");

    g_sr_data->result_que = xQueueCreate(3, sizeof(sr_result_t));
    ESP_GOTO_ON_FALSE(NULL != g_sr_data->result_que, ESP_ERR_NO_MEM, err, TAG, "Failed create result queue");

    g_sr_data->event_group = xEventGroupCreate();
    ESP_GOTO_ON_FALSE(NULL != g_sr_data->event_group, ESP_ERR_NO_MEM, err, TAG, "Failed create event_group");

    BaseType_t ret_val;
    models = esp_srmodel_init("model");
    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();

    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    afe_config.aec_init = false;

    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);
    g_sr_data->afe_handle = afe_handle;
    g_sr_data->afe_data = afe_data;

    g_sr_data->lang = SR_LANG_MAX;
    ret = app_sr_set_language(SR_LANG_EN);
    ESP_GOTO_ON_FALSE(ESP_OK == ret, ESP_FAIL, err, TAG,  "Failed to set language");

    ret_val = xTaskCreatePinnedToCore(&audio_feed_task, "Feed Task", 8 * 1024, (void *)afe_data, 5, &g_sr_data->feed_task, 0);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,  "Failed create audio feed task");

    ret_val = xTaskCreatePinnedToCore(&audio_detect_task, "Detect Task", 10 * 1024, (void *)afe_data, 5, &g_sr_data->detect_task, 1);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,  "Failed create audio detect task");

    ret_val = xTaskCreatePinnedToCore(&sr_handler_task, "SR Handler Task", 8 * 1024, NULL, 5, &g_sr_data->handle_task, 0);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,  "Failed create audio handler task");

    audio_record_init();

    return ESP_OK;
err:
    app_sr_stop();
    return ret;
}

esp_err_t app_sr_stop(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    xEventGroupSetBits(g_sr_data->event_group, NEED_DELETE);
    xEventGroupWaitBits(g_sr_data->event_group, NEED_DELETE | FEED_DELETED | DETECT_DELETED | HANDLE_DELETED, 1, 1, portMAX_DELAY);

    if (g_sr_data->result_que) {
        vQueueDelete(g_sr_data->result_que);
        g_sr_data->result_que = NULL;
    }

    if (g_sr_data->event_group) {
        vEventGroupDelete(g_sr_data->event_group);
        g_sr_data->event_group = NULL;
    }

    if (g_sr_data->fp) {
        fclose(g_sr_data->fp);
        g_sr_data->fp = NULL;
    }

    if (g_sr_data->model_data) {
        g_sr_data->multinet->destroy(g_sr_data->model_data);
    }

    if (g_sr_data->afe_data) {
        g_sr_data->afe_handle->destroy(g_sr_data->afe_data);
    }

    if (g_sr_data->afe_in_buffer) {
        heap_caps_free(g_sr_data->afe_in_buffer);
    }

    if (g_sr_data->afe_out_buffer) {
        heap_caps_free(g_sr_data->afe_out_buffer);
    }

    heap_caps_free(g_sr_data);
    g_sr_data = NULL;
    return ESP_OK;
}

esp_err_t app_sr_get_result(sr_result_t *result, TickType_t xTicksToWait)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    xQueueReceive(g_sr_data->result_que, result, xTicksToWait);
    return ESP_OK;
}

esp_err_t app_sr_start_once(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    manul_detect_flag = true;
    return ESP_OK;
}

esp_err_t app_sr_add_cmd(const sr_cmd_t *cmd)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    ESP_RETURN_ON_FALSE(NULL != cmd, ESP_ERR_INVALID_ARG, TAG, "pointer of cmd is invalid");
    ESP_RETURN_ON_FALSE(cmd->lang == g_sr_data->lang, ESP_ERR_INVALID_ARG, TAG, "cmd lang error");
    ESP_RETURN_ON_FALSE(ESP_MN_MAX_PHRASE_NUM >= g_sr_data->cmd_num, ESP_ERR_INVALID_STATE, TAG, "cmd is full");

    sr_cmd_t *item = (sr_cmd_t *)heap_caps_calloc(1, sizeof(sr_cmd_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(NULL != item, ESP_ERR_NO_MEM, TAG, "memory for sr cmd is not enough");
    memcpy(item, cmd, sizeof(sr_cmd_t));
    item->id = g_sr_data->cmd_num; 
    item->next.sle_next = NULL;
#if 1 // insert after
    sr_cmd_t *last = SLIST_FIRST(&g_sr_data->cmd_list);
    if (last == NULL) {
        SLIST_INSERT_HEAD(&g_sr_data->cmd_list, item, next);
    } else {
        sr_cmd_t *it;
        while ((it = SLIST_NEXT(last, next)) != NULL) {
            last = it;
        }
        SLIST_INSERT_AFTER(last, item, next);
    }
#else  // insert head
    SLIST_INSERT_HEAD(&g_sr_data->cmd_list, it, next);
#endif

    if (strstr(g_sr_data->mn_name, "mn6_en")) {
        esp_mn_commands_add(g_sr_data->cmd_num, (char *)cmd->str);
    } else {
        esp_mn_commands_add(g_sr_data->cmd_num, (char *)cmd->phoneme);
    }
    g_sr_data->cmd_num++;
    return ESP_OK;
}

esp_err_t app_sr_update_cmds(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");

    // uint32_t count = 0;
    // sr_cmd_t *it;
    // SLIST_FOREACH(it, &g_sr_data->cmd_list, next) {
    //     it->id = count++;
    // }

    esp_mn_error_t *err_id = esp_mn_commands_update(g_sr_data->multinet, g_sr_data->model_data);
    if (err_id) {
        for (int i = 0; i < err_id->num; i++) {
            ESP_LOGE(TAG, "err cmd id:%d", (int)err_id->phrases[i]); 
        }
    }
    esp_mn_commands_print();

    return ESP_OK;
}