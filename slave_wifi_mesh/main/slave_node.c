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
// #define MEMORY_DEBUG
xQueueHandle global_timer_group1_queue;

uint64_t slave_time_buff[];
uint64_t root_time_buff[];
uint64_t time_val_before_read = 0;
uint64_t root_time = 0;
uint64_t time_val_after_read = 0;
uint8_t count = 0;
uint64_t offset = 0;
uint64_t mega_delay = 0;
bool first_read = true;
static const char *TAG = "get_started";

uint64_t S64(const char *s)
{ 
    uint64_t i;
    char c;
    int scanned = sscanf(s, "%llu" PRIx64 "%c", &i, &c);
    if (scanned == 1)
        return i;
    if (scanned > 1)
    {
        // TBD about extra data found
        return i;
    }
    // TBD failed to scan;
    return 0;
}

static void node_read_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t data_type = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};

    MDF_LOGI("Note read task is running");

    for (;;)
    {
        if (!mwifi_is_connected())
        {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        //size = MWIFI_PAYLOAD_LEN;
        size = 20;
        memset(data, 0, size);
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_read, ret: %x", ret);
        MDF_LOGI("Node receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
    }

    MDF_LOGW("Note read task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

void node_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    int count = 0;
    size_t size = 0;
    char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    mwifi_data_type_t data_type = {0x0};

    // EE202 Codes
    uint64_t mytime0;
    uint64_t mytime1;

    MDF_LOGI("Node write task is running");

    for (;;)
    {
        if (!mwifi_is_connected())
        {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        printf("EE202 Says Hello! \n");
        timer_get_counter_value2(0, 0, &mytime0, &mytime1);
        //print_timer_counter(mytime0);
        //print_timer_counter(mytime1);

        sprintf(data, "%llu" PRIx64, (uint64_t)mytime0);
        //itoa(mytime0, data, 10);
        size = 20; //sprintf("*!* %lld", ee202_data);

        //size = sprintf(data, "(%d) Hello root!", count++);
        ret = mwifi_write(NULL, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_write, ret: %x", ret);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    MDF_LOGW("Node write task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

static void ee202_time_synch(void *arg)
{
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    mdf_err_t ret = MDF_OK;
    //uint8_t size = 20;
    char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t data_type = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    //EE202 Params
    uint64_t corrected_time = 0;
    uint64_t first_time = 0;
    //vTaskDelay(2000 / portTICK_RATE_MS);
    while(1)
    {
        vTaskDelay(2000 / portTICK_RATE_MS);
        printf("Writing Out to Master");
        for(uint8_t i = 0; i<15; i++)
        {
            printf("\n Count: %i", i);
            timer_get_counter_value1(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL+1, &first_time);
            if (!mwifi_is_connected())
            {
                vTaskDelay(100 / portTICK_RATE_MS);
                continue;
            }
            timer_get_counter_value1(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL+1, &time_val_before_read);
            sprintf(data, "%llu" PRIx64, (uint64_t)time_val_before_read);

            ret = mwifi_write(NULL, &data_type, data, size, true); // junk val
            MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_write, ret: %x", ret);
            //timer_get_counter_value1(TIMER_GROUP_0, TIMER_1, &time_val_before_read);
            ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
            //MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_read, ret: %x", ret);
            timer_get_counter_value1(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL+1, &time_val_after_read);
            root_time = S64(data);
            vTaskDelay(100 / portTICK_RATE_MS);
            mega_delay = (time_val_after_read - time_val_before_read) / 2;
            offset = (2 * root_time - time_val_before_read - time_val_after_read) / 2;
            //timer_get_counter_value1(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL, &time_val_before_read);
            corrected_time = (uint64_t)(offset + time_val_after_read);
            //MDF_LOGI("Node receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
            timer_set_counter_value1(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL + 1, corrected_time);
            timer_set_counter_value1(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL, corrected_time + 3980000); //4480000

            //timer_set_counter_value(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL, offset + time_val_before_read);
            printf("\nRoot Time: %llu", root_time);
            printf("\nSlave Time: %llu", corrected_time);
            printf("\nSlave Time w/ MD: %llu", corrected_time - mega_delay);
            printf("\nOffset: %llu", offset);
            printf("\nMega Delay: %llu", mega_delay);
            
            if (first_read)
            {
                first_read = false;
                first_time = corrected_time;
                ee202_periodic_timer_event_init_slave(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL, TIMER_INTERVAL0_SEC);
            }
        }
        //ee202_check_timer_alarm(first_time);
    }
    
    printf("DONE INIT ROUTINE!!");
    MDF_LOGW("Note read task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

static void print_system_info_timercb(void *timer)
{
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

    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %u",
             primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mesh_assoc.rssi, esp_mesh_get_total_node_num(), esp_get_free_heap_size());


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
    printf("Hello World!");
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
    uint8_t priority = 4;
    printf("START Time Sync");
    xSemaphore = xSemaphoreCreateBinary();
    ee202_gpio_init(TIMER_GPIO_PIN);
    ee202_timer_init(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL);
    ee202_timer_init(EE202_TIME_SYNCH_GROUP, EE202_TIME_SYNCH_CHANNEL + 1);

    global_timer_group0_queue = xQueueCreate(10, sizeof(timer_event_t));
    vTaskDelay(1000);
    //xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    xTaskCreate(ee202_time_synch, "ee202_time_synch", 4 * 1024, NULL, priority, NULL);
    //TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS, true, NULL, print_system_info_timercb);
    //xTimerStart(timer, 0);
}
