#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "driver/gpio.h"

static esp_err_t led_gpio_init(int io_num)
{
    gpio_config_t io_conf = {0};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = 1ULL << io_num;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    esp_err_t ret = gpio_config(&io_conf);
	return ret;
}

static esp_err_t led_set_state(int io_num, bool state)
{
    if (state)
		return gpio_set_level(io_num, 1);
	else
		return gpio_set_level(io_num, 0);
}


// scan parameters
static esp_ble_scan_params_t ble_scan_params = {
		.scan_type              = BLE_SCAN_TYPE_ACTIVE,
		.own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ONLY_WLST,
		.scan_interval          = 0x50,
		.scan_window            = 0x30
};

// GAP callback
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
		
		case ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT:

			printf("ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT\n");
			if(param->update_whitelist_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Update whitelist successful\n\n");
			}
			else printf("Unable to update whitelist, error code %d\n\n", param->update_whitelist_cmpl.status);
			break;

        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: 
				
			printf("ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT\n");
			if(param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Scan parameters set, update whitelist and start scanning\n\n");
                esp_bd_addr_t whitelist_addr = {0xa4, 0xc1, 0x38, 0x63, 0x16, 0x13};
                esp_ble_gap_update_whitelist(true, whitelist_addr, BLE_ADDR_TYPE_PUBLIC);
				esp_ble_gap_start_scanning(0);
			}
			else printf("Unable to set scan parameters, error code %d\n\n", param->scan_param_cmpl.status);
			break;
		
		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
			
			printf("ESP_GAP_BLE_SCAN_START_COMPLETE_EVT\n");
			if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Scan started\n\n");
			}
			else printf("Unable to start scan process, error code %d\n\n", param->scan_start_cmpl.status);
			break;
		
		case ESP_GAP_BLE_SCAN_RESULT_EVT:
			
            led_set_state(5, true);
			printf("ESP_GAP_BLE_SCAN_RESULT_EVT\n");
            printf("Device found: ADDR=");
            for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
                printf("%02X", param->scan_rst.bda[i]);
                if(i != ESP_BD_ADDR_LEN -1) printf(":");
            }
            
            // try to read the complete name
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;
            adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if(adv_name) {
                printf("\nFULL NAME=");
                for(int i = 0; i < adv_name_len; i++) printf("%c", adv_name[i]);
            }

			printf("\nAdv_data_len=%u\n", param->scan_rst.adv_data_len);

			for (int i = 0; i < param->scan_rst.adv_data_len; i++) {
				printf("%02X ", param->scan_rst.ble_adv[i]);
			}			
            
            int16_t temperature = param->scan_rst.ble_adv[10] | (param->scan_rst.ble_adv[11] << 8);
            uint16_t humidity = param->scan_rst.ble_adv[12] | (param->scan_rst.ble_adv[13] << 8);
			uint8_t battery_level = param->scan_rst.ble_adv[16];
			printf("\ntemperature = %i, humidity = %u, battery level = %u\n\n", temperature, humidity, battery_level);
			
			vTaskDelay(10 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(led_set_state(5, false));

            break;

		default:
		
			printf("Event %d unhandled\n\n", event);
			break;
	}
}


void app_main() {

	ESP_ERROR_CHECK(led_gpio_init(5));
	ESP_ERROR_CHECK(led_gpio_init(19));
    ESP_ERROR_CHECK(led_set_state(19, false));
	printf("- LED indicators initialized\n");

	printf("BT scan\n\n");

	// set components to log only errors
	esp_log_level_set("*", ESP_LOG_ERROR);
	
	// initialize nvs
	ESP_ERROR_CHECK(nvs_flash_init());
	printf("- NVS init ok\n");
	
	// release memory reserved for classic BT (not used)
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	printf("- Memory for classic BT released\n");
	
	// initialize the BT controller with the default config
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
	printf("- BT controller init ok\n");
	
	// enable the BT controller in BLE mode
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
	printf("- BT controller enabled in BLE mode\n");
	
	// initialize Bluedroid library
	esp_bluedroid_init();
    esp_bluedroid_enable();
	printf("- Bluedroid initialized and enabled\n");
	
	// register GAP callback function
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
	printf("- GAP callback registered\n\n");
	
	// configure scan parameters
	esp_ble_gap_set_scan_params(&ble_scan_params);
}