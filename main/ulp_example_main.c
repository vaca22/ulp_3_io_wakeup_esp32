#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"


#include "esp32/ulp.h"

#include "ulp_main.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_ulp_program(void);
static void update_pulse_count(void);

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    printf("cause %d\n", cause);
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup, initializing ULP\n");
        init_ulp_program();
    } else {
        printf("ULP wakeup, saving pulse count\n");
        update_pulse_count();
        vTaskDelay(300);
    }

    printf("Entering deep sleep\n\n");
    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);


    gpio_num_t gpio_num = GPIO_NUM_12;
    int rtcio_num = rtc_io_number_get(gpio_num);
    assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");


    gpio_num_t gpio_num2 = GPIO_NUM_13;
    int rtcio_num2 = rtc_io_number_get(gpio_num2);
    assert(rtc_gpio_is_valid_gpio(gpio_num2) && "GPIO used for pulse counting must be an RTC IO");



    gpio_num_t gpio_num3 = GPIO_NUM_38;
    int rtcio_num3 = rtc_io_number_get(gpio_num3);
    assert(rtc_gpio_is_valid_gpio(gpio_num3) && "GPIO used for pulse counting must be an RTC IO");



    ulp_next_edge_1 = 0;
    ulp_init_edge_1 = 1-ulp_next_edge_1;
    ulp_io_number_1 = rtcio_num; /* map from GPIO# to RTC_IO# */
    ulp_debounce_counter_1 = 10;
    ulp_debounce_max_count_1 = 10;


    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_en(gpio_num);
    rtc_gpio_hold_en(gpio_num);

    rtc_gpio_init(gpio_num2);
    rtc_gpio_set_direction(gpio_num2, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num2);
    rtc_gpio_pullup_en(gpio_num2);
    rtc_gpio_hold_en(gpio_num2);

    rtc_gpio_init(gpio_num3);
    rtc_gpio_set_direction(gpio_num3, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num3);
    rtc_gpio_pullup_en(gpio_num3);
    rtc_gpio_hold_en(gpio_num3);

    esp_deep_sleep_disable_rom_logging();


    ulp_set_wakeup_period(0, 20000);


    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static void update_pulse_count(void)
{
    const char* namespace = "plusecnt";
    const char* count_key = "count";

    nvs_flash_init();
    nvs_handle_t handle;
    nvs_open(namespace, NVS_READWRITE, &handle);
    uint32_t pulse_count = 0;
    esp_err_t err = nvs_get_u32(handle, count_key, &pulse_count);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    printf("Read pulse count from NVS: %5d\n", pulse_count);




    pulse_count += 1;
    nvs_set_u32(handle, count_key, pulse_count);
    nvs_commit(handle);
    nvs_close(handle);
    printf("Wrote updated pulse count to NVS: %5d\n", pulse_count);
}
