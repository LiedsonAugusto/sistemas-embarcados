#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define BTN_COUNT GPIO_NUM_1
#define BTN_MODE  GPIO_NUM_2

#define D0 GPIO_NUM_4
#define D1 GPIO_NUM_5
#define D2 GPIO_NUM_6
#define D3 GPIO_NUM_7

#define ANTI_BOUNCE 200000  // ms

static uint8_t bin_state = 0;
static uint8_t step = 1;

static int64_t last_btn1_time = 0;
static int64_t last_btn2_time = 0;

void set_binary_output(uint8_t num) {
    gpio_set_level(D0, num & 0x01);
    gpio_set_level(D1, (num >> 1) & 0x01);
    gpio_set_level(D2, (num >> 2) & 0x01);
    gpio_set_level(D3, (num >> 3) & 0x01);
}

void configure_gpio_pins() {
    gpio_config_t out_pins = {
        .pin_bit_mask = (1ULL << D0) | (1ULL << D1) | (1ULL << D2) | (1ULL << D3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_pins);

    gpio_config_t in_pins = {
        .pin_bit_mask = (1ULL << BTN_COUNT) | (1ULL << BTN_MODE),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_pins);
}

void app_main() {
    configure_gpio_pins();
    set_binary_output(bin_state);

    last_btn1_time = last_btn2_time = esp_timer_get_time();

    while (1) {
        int64_t now = esp_timer_get_time();

        if (gpio_get_level(BTN_COUNT) == 1 && (now - last_btn1_time) > ANTI_BOUNCE) {
            last_btn1_time = now;
            bin_state = (bin_state + step) & 0x0F;
            set_binary_output(bin_state);
        }

        if (gpio_get_level(BTN_MODE) == 1 && (now - last_btn2_time) > ANTI_BOUNCE) {
            last_btn2_time = now;
            step = (step == 1) ? 2 : 1;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
