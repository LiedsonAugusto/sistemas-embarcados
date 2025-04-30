#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_1 12
#define LED_2 23 

void pisca_led_1(void *pvParameter) {
    gpio_set_direction(LED_1, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(LED_1, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_1, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void pisca_led_2(void *pvParameter) {
    gpio_set_direction(LED_2, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(LED_2, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_2, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void) {
    xTaskCreate(pisca_led_1, "Pisca LED 1s", 2048, NULL, 1, NULL);
    xTaskCreate(pisca_led_2, "Pisca LED 200ms", 2048, NULL, 1, NULL);
}

