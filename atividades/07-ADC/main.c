#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "int_i2c.h"
#include "esp_log.h"

#define LED_A 4
#define LED_B 5
#define LED_C 6
#define LED_D 7
#define BTN_UP 37
#define BTN_DOWN 2
#define BUZZER_PIN 11
#define SDA_PIN 8
#define SCL_PIN 9

#define TEMP_SENSOR_PIN 1
#define TEMP_ADC_CHANNEL ADC1_CHANNEL_0
#define BETA_CONST 3950.0
#define MAX_ADC 4095.0
#define ROOM_TEMP_K 298.15

#define I2C_PORT I2C_NUM_0
#define I2C_FREQ 100000
#define LCD_I2C_ADDR 0x27
#define LCD_DIM DISPLAY_16X02

static const char *LOG_TAG = "TEMP_CTRL";

int temp_limite = 25;
int temp_lida = 20;

lcd_i2c_handle_t visor;

volatile bool flag_btn_up = false;
volatile bool flag_btn_down = false;

static void IRAM_ATTR isr_btn_up(void* arg) {
    gpio_intr_disable(BTN_UP);
    flag_btn_up = true;
}

static void IRAM_ATTR isr_btn_down(void* arg) {
    gpio_intr_disable(BTN_DOWN);
    flag_btn_down = true;
}

static void iniciar_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

void configurar_botoes() {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BTN_UP) | (1ULL << BTN_DOWN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);

    gpio_set_intr_type(BTN_UP, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(BTN_DOWN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_UP, isr_btn_up, NULL);
    gpio_isr_handler_add(BTN_DOWN, isr_btn_down, NULL);
}

void configurar_pwm_buzzer() {
    ledc_timer_config_t timer_cfg = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 2000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ch_cfg);
}

void ativar_buzzer() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void desativar_buzzer() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

float capturar_temperatura() {
    int leitura_adc = adc1_get_raw(TEMP_ADC_CHANNEL);
    float resistencia = 10000.0 / ((MAX_ADC / (float)leitura_adc) - 1.0);
    float temp_k = 1.0 / (log(resistencia / 10000.0) / BETA_CONST + (1.0 / ROOM_TEMP_K));
    return temp_k - 273.15;
}

void exibir_lcd() {
    char linha[20];
    lcd_i2c_cursor_set(&visor, 0, 0);
    snprintf(linha, sizeof(linha), "Alarme: %d C", temp_limite);
    lcd_i2c_print(&visor, linha);

    lcd_i2c_cursor_set(&visor, 0, 1);
    snprintf(linha, sizeof(linha), "Atual : %d C", temp_lida);
    lcd_i2c_print(&visor, linha);
}

void gerenciar_leds(int temp, int lim, bool alarme) {

    if (temp >= 25) {
        gpio_set_level(LED_A, 1);
        gpio_set_level(LED_B, 1);
        gpio_set_level(LED_C, 1);
        gpio_set_level(LED_D, 1);
    } else if (temp >= 23) {
        gpio_set_level(LED_A, 0);
        gpio_set_level(LED_B, 0);
        gpio_set_level(LED_C, 0);
        gpio_set_level(LED_D, 1);
    } else if (temp >= 20) {
        gpio_set_level(LED_A, 0);
        gpio_set_level(LED_B, 0);
        gpio_set_level(LED_C, 1);
        gpio_set_level(LED_D, 0);
    } else if (temp >= 15) {
        gpio_set_level(LED_A, 0);
        gpio_set_level(LED_B, 1);
        gpio_set_level(LED_C, 0);
        gpio_set_level(LED_D, 0);
    } else if (temp >= 10) {
        gpio_set_level(LED_A, 1);
        gpio_set_level(LED_B, 0);
        gpio_set_level(LED_C, 0);
        gpio_set_level(LED_D, 0);
    } else {
        gpio_set_level(LED_A, 0);
        gpio_set_level(LED_B, 0);
        gpio_set_level(LED_C, 0);
        gpio_set_level(LED_D, 0);
    }
}

void app_main(void) {

    iniciar_i2c();
    visor.address = LCD_I2C_ADDR;
    visor.num = I2C_PORT;
    visor.backlight = 1;
    visor.size = LCD_DIM;
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_i2c_init(&visor);

    gpio_config_t cfg_leds = {
        .pin_bit_mask = (1ULL << LED_A) | (1ULL << LED_B) | (1ULL << LED_C) | (1ULL << LED_D),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&cfg_leds);

    configurar_botoes();
    configurar_pwm_buzzer();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TEMP_ADC_CHANNEL, ADC_ATTEN_DB_11);

    lcd_i2c_cursor_set(&visor, 0, 0);
    lcd_i2c_print(&visor, "Controle de Temp");
    lcd_i2c_cursor_set(&visor, 0, 1);
    lcd_i2c_print(&visor, "Carregando...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_i2c_write(&visor, 0, CLEAR_DISPLAY);

    TickType_t ultima_leitura_up = 0;
    TickType_t ultima_leitura_down = 0;
    const TickType_t intervalo_debounce = pdMS_TO_TICKS(50);

    while (1) {
        if (flag_btn_up) {
            flag_btn_up = false;
            TickType_t agora = xTaskGetTickCount();
            if ((agora - ultima_leitura_up) > intervalo_debounce) {
                temp_limite += 5;
                ESP_LOGI(LOG_TAG, "Novo limite: %d C", temp_limite);
                ultima_leitura_up = agora;
            }
            gpio_intr_enable(BTN_UP);
        }

        if (flag_btn_down) {
            flag_btn_down = false;
            TickType_t agora = xTaskGetTickCount();
            if ((agora - ultima_leitura_down) > intervalo_debounce) {
                temp_limite -= 5;
                ESP_LOGI(LOG_TAG, "Novo limite: %d C", temp_limite);
                ultima_leitura_down = agora;
            }
            gpio_intr_enable(BTN_DOWN);
        }

        temp_lida = (int)capturar_temperatura();
        bool alarme = temp_lida >= temp_limite;

        if (alarme) ativar_buzzer();
        else desativar_buzzer();

        gerenciar_leds(temp_lida, temp_limite, alarme);
        exibir_lcd();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
