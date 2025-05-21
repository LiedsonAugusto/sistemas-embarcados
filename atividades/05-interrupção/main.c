#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define PIN_BTN_NEXT   GPIO_NUM_1
#define PIN_BTN_STEP   GPIO_NUM_2

#define LED_0          GPIO_NUM_4
#define LED_1          GPIO_NUM_5
#define LED_2          GPIO_NUM_6
#define LED_3          GPIO_NUM_7

static volatile bool next_pressed = false;
static volatile bool step_toggled = false;

static uint8_t current_value = 0;
static uint8_t increment = 1;

static void IRAM_ATTR on_next_button(void *arg) {
    gpio_intr_disable(PIN_BTN_NEXT);
    next_pressed = true;
}

static void IRAM_ATTR on_step_button(void *arg) {
    gpio_intr_disable(PIN_BTN_STEP);
    step_toggled = true;
}

static void update_leds(uint8_t value) {
    gpio_set_level(LED_0, value & 0x01);
    gpio_set_level(LED_1, (value >> 1) & 0x01);
    gpio_set_level(LED_2, (value >> 2) & 0x01);
    gpio_set_level(LED_3, (value >> 3) & 0x01);
}

static void setup_io() {
    gpio_config_t input_cfg = {
        .pin_bit_mask = (1ULL << PIN_BTN_NEXT) | (1ULL << PIN_BTN_STEP),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = false,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&input_cfg);

    gpio_config_t output_cfg = {
        .pin_bit_mask = (1ULL << LED_0) | (1ULL << LED_1) | (1ULL << LED_2) | (1ULL << LED_3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = false,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BTN_NEXT, on_next_button, NULL);
    gpio_isr_handler_add(PIN_BTN_STEP, on_step_button, NULL);
}

void app_main() {
    setup_io();
    update_leds(current_value);

    uint8_t prev_next_state = 1, prev_step_state = 1;
    TickType_t last_next_time = 0, last_step_time = 0;
    const TickType_t debounce_time = pdMS_TO_TICKS(50);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        uint8_t read_next = gpio_get_level(PIN_BTN_NEXT);
        uint8_t read_step = gpio_get_level(PIN_BTN_STEP);

        if (next_pressed) {
            next_pressed = false;

            if (read_next == 0 && prev_next_state == 1 && (now - last_next_time) > debounce_time) {
                current_value = (current_value + increment) & 0x0F;
                update_leds(current_value);
                last_next_time = now;
            }
            gpio_intr_enable(PIN_BTN_NEXT);
        }

        if (step_toggled) {
            step_toggled = false;

            if (read_step == 0 && prev_step_state == 1 && (now - last_step_time) > debounce_time) {
                increment = (increment == 1) ? 2 : 1;
                last_step_time = now;
            }
            gpio_intr_enable(PIN_BTN_STEP);
        }

        prev_next_state = read_next;
        prev_step_state = read_step;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
