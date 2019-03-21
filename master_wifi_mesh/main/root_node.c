
/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "mdf_common.h"
#include "mwifi.h"
#include "../../../../AkashTimer/main/ee202.h"
#include "driver/gpio.h"
#include "soc/frc_timer_reg.h"
#include "soc/rtc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// EE202
xQueueHandle global_timer_queue;
uint8_t src_addrs[];
uint8_t src_counts[];
uint8_t TIME_SYNC_TIMER_GROUP = 0;
uint8_t TIME_SYNC_TIMER_CHANNEL = 0;
uint8_t slave_count = 0;
uint64_t last_time = 0;

static const char *TAG = "get_started";

void check_add_src_address(uint8_t address)
{
    bool gate = true;
    for (uint8_t i = 0; i < slave_count; i++)
    {
        if (src_addrs[i] == address)
        {
            gate = false;
            src_counts[i] = src_counts[i] + 1;
        }
    }
    if (gate)
    {
        printf("Adding Element to the Address Buffer!");
        src_addrs[slave_count] = address;
        src_counts[slave_count] = 0;
        slave_count = slave_count + 1;
    }
}

uint32_t get_address_counts(uint8_t address)
{
    bool gate = true;
    for (uint8_t i = 0; i < slave_count; i++)
    {
        if (src_addrs[i] == address)
        {
            gate = false;
            src_counts[i] = src_counts[i] + 1;
        }
    }
    if (gate)
    {
        printf("Adding Element to the Address Buffer!");
        src_addrs[slave_count] = address;
        src_counts[slave_count] = 0;
        slave_count = slave_count + 1;
    }
    return 0;
}

static void root_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type = {0};
    uint8_t print_count = 0;
    MDF_LOGI("Root is running");

    // EE202 Codes
    uint64_t mytime0;
    uint64_t mytime1;
    size = 20;
    for (int i = 0;; ++i)
    {
        if (!mwifi_is_started())
        {
            vTaskDelay(100 / portTICK_RATE_MS);
            continue;
        }
        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        
        //MDF_LOGI("Root receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
        //check_add_src_address(src_addr);

        timer_get_counter_value1(TIME_SYNC_TIMER_GROUP, TIME_SYNC_TIMER_CHANNEL, &mytime0);
        sprintf(data, "%llu" PRIx64, (uint64_t)mytime0);
        ret = mwifi_root_write(src_addr, 1, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_root_recv, ret: %x", ret);
        timer_get_counter_value1(TIME_SYNC_TIMER_GROUP, TIME_SYNC_TIMER_CHANNEL, &mytime0);
        //MDF_LOGI("Root send, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
        if(print_count < loop_count)
        {
            printf("\nroot_time: %llu", (uint64_t)mytime0);
            print_count += 1;
        }
        else
        {
            printf("\n---------------");
            printf("\nroot_time: %llu", (uint64_t)mytime0);
            print_count = 1;
        }
        
    }
    MDF_LOGW("Root is exit");
    MDF_FREE(data);
    vTaskDelete(NULL);
}
/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(void *timer)
{
    // NAT HACK
    uint64_t mytime0;
    timer_get_counter_value1(0, 0, &mytime0);
    //printf("From Nat and Akash: Current Time is: %llu \n", mytime0);
    // Begin normal
    uint8_t primary = 0;
    wifi_second_chan_t second = 0;
    mesh_addr_t parent_bssid = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    mesh_assoc_t mesh_assoc = {0x0};
    wifi_sta_list_t wifi_sta_list = {0x0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_wifi_vnd_mesh_get(&mesh_assoc);
    esp_mesh_get_parent_bssid(&parent_bssid);

    //MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
    //         ", parent rssi: %d, node num: %d, free heap: %u",
    //         primary,
    //         esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
    //         mesh_assoc.rssi, esp_mesh_get_total_node_num(), esp_get_free_heap_size());

    //for (int i = 0; i < wifi_sta_list.num; i++)
    //{
    //    MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    //}

#ifdef MEMORY_DEBUG
    if (!heap_caps_check_integrity_all(true))
    {
        MDF_LOGE("At least one heap is corrupt");
    }

    mdf_mem_print_heap();
    mdf_mem_print_record();
#endif /**< MEMORY_DEBUG */
}

static mdf_err_t wifi_init()
{
    mdf_err_t ret = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    tcpip_adapter_init();
    MDF_ERROR_ASSERT(esp_event_loop_init(NULL, NULL));
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());

    return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event)
    {
    case MDF_EVENT_MWIFI_STARTED:
        MDF_LOGI("MESH is started");
        break;

    case MDF_EVENT_MWIFI_PARENT_CONNECTED:
        MDF_LOGI("Parent is connected on station interface");
        break;

    case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
        MDF_LOGI("Parent is disconnected on station interface");
        break;

    default:
        break;
    }

    return MDF_OK;
}

/*
 * The main task of this example program
 */

void app_main()
{
    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config = {
        .channel = CONFIG_MESH_CHANNEL,
        .mesh_id = CONFIG_MESH_ID,
        .mesh_type = CONFIG_DEVICE_TYPE,
    };

    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    /**
     * @brief Initialize wifi mesh.
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());

    /**
     * @brief Data transfer between wifi mesh devices
     */
    ee202_gpio_init(TIMER_GPIO_PIN);
    ee202_timer_init(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL);
    ee202_timer_init(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL + 1);
    global_timer_group0_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Same Timer on the update and the gpio toggle
    ee202_periodic_timer_event_init_master(TIME_SYNC_TIMER_GROUP, TIME_SYNC_TIMER_CHANNEL, TIMER_INTERVAL0_SEC); // Will activate GPIO Toggle event every x amount of time
    uint8_t priority = 5;
    xTaskCreate(root_task, "root_task", 4 * 1024, NULL, priority, NULL);
    //TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS, true, NULL, print_system_info_timercb);
    //xTimerStart(timer, 0);
}
