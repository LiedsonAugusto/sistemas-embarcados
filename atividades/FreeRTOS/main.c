#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "int_i2c.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"

#define SEG_G 4
#define SEG_F 5
#define SEG_A 6
#define SEG_B 7
#define SEG_C 15
#define SEG_D 17
#define SEG_E 18

#define BTN_UP 2
#define BTN_DOWN 37
#define BUZZER 3
#define I2C_SDA 8
#define I2C_SCL 9
#define SENSOR 1
#define SENSOR_CH ADC1_CHANNEL_0
#define BETA 3950.0
#define ADC_MAX 4095.0
#define T0_K 298.15
#define SD_MISO GPIO_NUM_13
#define SD_MOSI GPIO_NUM_11
#define SD_CLK GPIO_NUM_12
#define SD_CS GPIO_NUM_10
#define SD_MOUNT "/sdcard"
#define LOG_FILE SD_MOUNT"/temp_log.csv"
#define MIN_TEMP 10
#define MAX_TEMP 50
#define LOG_INTERVAL 10000
#define I2C_NUM I2C_NUM_0
#define I2C_FREQ 100000
#define LCD_ADDR 0x27
#define LCD_SIZE DISPLAY_16X02

static const char *TAG = "TEMP_MONITOR";

SemaphoreHandle_t mutex;
int alarm_temp = 25;
int current_temp = 20;
bool alarm_active = false;
bool sd_ready = false;
lcd_i2c_handle_t lcd;

QueueHandle_t btn_queue;

static void IRAM_ATTR btn_up_isr(void* arg) {
    int btn_id = 1;
    xQueueSendFromISR(btn_queue, &btn_id, NULL);
}

static void IRAM_ATTR btn_down_isr(void* arg) {
    int btn_id = 2;
    xQueueSendFromISR(btn_queue, &btn_id, NULL);
}

void read_temp(void *pvParam);
void handle_buttons(void *pvParam);
void sound_alarm(void *pvParam);
void update_lcd(void *pvParam);
void update_display(void *pvParam);
void log_data(void *pvParam);

void show_digit(char digit);

void setup_gpio() {
    gpio_config_t seg_config = {
        .pin_bit_mask = (1ULL << SEG_A) | (1ULL << SEG_B) | (1ULL << SEG_C) |
                        (1ULL << SEG_D) | (1ULL << SEG_E) | (1ULL << SEG_F) |
                        (1ULL << SEG_G),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&seg_config);

    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BTN_UP) | (1ULL << BTN_DOWN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_UP, btn_up_isr, NULL);
    gpio_isr_handler_add(BTN_DOWN, btn_down_isr, NULL);
    
    show_digit(' ');
}

void setup_pwm() {
    ledc_timer_config_t pwm_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 2000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config(&pwm_channel);
}

void setup_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };
    i2c_param_config(I2C_NUM, &conf);
    i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);
}

void setup_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_CH, ADC_ATTEN_DB_12);
}

float read_temp_c() {
    int raw = adc1_get_raw(SENSOR_CH);
    float resistance = 10000.0 / ((ADC_MAX / (float)raw) - 1.0);
    float temp_k = 1.0 / (log(resistance / 10000.0) / BETA + (1.0 / T0_K));
    return temp_k - 273.15;
}

bool init_sd() {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI error: 0x%x", ret);
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount error: 0x%x", ret);
        return false;
    }
    
    struct stat st;
    if (stat(LOG_FILE, &st) == -1) {
        FILE *f = fopen(LOG_FILE, "w");
        if (f) {
            fclose(f);
        }
    }
    return true;
}

void write_log() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    FILE *f = fopen(LOG_FILE, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "File error");
        return;
    }

    fprintf(f, "%04d-%02d-%02d,%02d:%02d:%02d,%d,%d,%s\n",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
            current_temp, alarm_temp, alarm_active ? "ON" : "OFF");
    
    fclose(f);
    ESP_LOGI(TAG, "Data logged");
}

void read_temp(void *pvParam) {
    while(1) {
        float temp = read_temp_c();
        
        if (xSemaphoreTake(mutex, portMAX_DELAY)) {
            current_temp = (int)temp;
            alarm_active = (current_temp >= alarm_temp);
            xSemaphoreGive(mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

bool btn_up_pressed = false;
bool btn_down_pressed = false;

static void IRAM_ATTR local_btn_up_isr(void* arg) {
        gpio_intr_disable(BTN_UP);
        btn_up_pressed = true;
    }

static void IRAM_ATTR local_btn_down_isr(void* arg) {
        gpio_intr_disable(BTN_DOWN);
        btn_down_pressed = true;
    }

void handle_buttons(void *pvParam) {
    uint8_t last_up_state = 1;
    uint8_t last_down_state = 1;
    TickType_t last_up_time = 0;
    TickType_t last_down_time = 0;
    const TickType_t debounce = pdMS_TO_TICKS(50);
    
    gpio_isr_handler_remove(BTN_UP);
    gpio_isr_handler_remove(BTN_DOWN);
    gpio_isr_handler_add(BTN_UP, local_btn_up_isr, NULL);
    gpio_isr_handler_add(BTN_DOWN, local_btn_down_isr, NULL);

    while(1) {
        if (btn_up_pressed) {
            btn_up_pressed = false;
            uint8_t current_state = gpio_get_level(BTN_UP);
            TickType_t now = xTaskGetTickCount();

            if (current_state == 0 && last_up_state == 1) {
                if ((now - last_up_time) >= debounce) {
                    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
                        if (alarm_temp < MAX_TEMP) {
                            alarm_temp += 5;
                            ESP_LOGI(TAG, "New limit: %d", alarm_temp);
                        }
                        xSemaphoreGive(mutex);
                    }
                    last_up_time = now;
                }
            }
            last_up_state = current_state;
            gpio_intr_enable(BTN_UP);
        }

        if (btn_down_pressed) {
            btn_down_pressed = false;
            uint8_t current_state = gpio_get_level(BTN_DOWN);
            TickType_t now = xTaskGetTickCount();

            if (current_state == 0 && last_down_state == 1) {
                if ((now - last_down_time) >= debounce) {
                    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
                        if (alarm_temp > MIN_TEMP) {
                            alarm_temp -= 5;
                            ESP_LOGI(TAG, "New limit: %d", alarm_temp);
                        }
                        xSemaphoreGive(mutex);
                    }
                    last_down_time = now;
                }
            }
            last_down_state = current_state;
            gpio_intr_enable(BTN_DOWN);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void sound_alarm(void *pvParam) {
    while(1) {
        bool alarm;
        
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100))) {
            alarm = alarm_active;
            xSemaphoreGive(mutex);
        } else {
            continue;
        }
        
        if (alarm) {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 2500);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(100));
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

void update_lcd(void *pvParam) {
    char buffer[20];
    while(1) {
        int temp_now, temp_limit;
        
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100))) {
            temp_now = current_temp;
            temp_limit = alarm_temp;
            xSemaphoreGive(mutex);
        } else {
            continue;
        }
        
        lcd_i2c_cursor_set(&lcd, 0, 0);
        snprintf(buffer, sizeof(buffer), "NTC: %d C", temp_now);
        lcd_i2c_print(&lcd, buffer);
        
        lcd_i2c_cursor_set(&lcd, 0, 1);
        snprintf(buffer, sizeof(buffer), "Alarme: %d C", temp_limit);
        lcd_i2c_print(&lcd, buffer);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void show_digit(char digit) {
    gpio_set_level(SEG_A, 1);
    gpio_set_level(SEG_B, 1);
    gpio_set_level(SEG_C, 1);
    gpio_set_level(SEG_D, 1);
    gpio_set_level(SEG_E, 1);
    gpio_set_level(SEG_F, 1);
    gpio_set_level(SEG_G, 1);

    switch (digit) {
        case '0':
            gpio_set_level(SEG_A, 0);
            gpio_set_level(SEG_B, 0);
            gpio_set_level(SEG_C, 0);
            gpio_set_level(SEG_D, 0);
            gpio_set_level(SEG_E, 0);
            gpio_set_level(SEG_F, 0);
            break;
            
        case '3':
            gpio_set_level(SEG_A, 0);
            gpio_set_level(SEG_B, 0);
            gpio_set_level(SEG_C, 0);
            gpio_set_level(SEG_D, 0);
            gpio_set_level(SEG_G, 0);
            break;
            
        case '7':
            gpio_set_level(SEG_A, 0);
            gpio_set_level(SEG_B, 0);
            gpio_set_level(SEG_C, 0);
            break;
            
        case 'D':
            gpio_set_level(SEG_B, 0);
            gpio_set_level(SEG_C, 0);
            gpio_set_level(SEG_D, 0);
            gpio_set_level(SEG_E, 0);
            gpio_set_level(SEG_G, 0);
            break;
            
        case 'F':
            gpio_set_level(SEG_A, 0);
            gpio_set_level(SEG_E, 0);
            gpio_set_level(SEG_F, 0);
            gpio_set_level(SEG_G, 0);
            break;
            
        default:
            break;
    }
}

void update_display(void *pvParam) {
    char digit = ' ';
    uint64_t last_blink = 0;
    bool display_on = true;
    
    while(1) {
        int temp_now, temp_limit;
        bool alarm;
        
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100))) {
            temp_now = current_temp;
            temp_limit = alarm_temp;
            alarm = alarm_active;
            xSemaphoreGive(mutex);
        } else {
            continue;
        }
        
        int diff = temp_limit - temp_now;
        
        if (alarm) {
            digit = 'F';
        } else if (diff <= 2) {
            digit = 'D';
        } else if (diff <= 10) {
            digit = '7';
        } else if (diff <= 15) {
            digit = '3';
        } else if (diff <= 20) {
            digit = '0';
        } else {
            digit = ' ';
        }
        
        if (alarm) {
            uint64_t now = esp_timer_get_time();
            if (now - last_blink > 500000) {
                last_blink = now;
                display_on = !display_on;
            }
        } else {
            display_on = true;
        }
        
        if (display_on) {
            show_digit(digit);
        } else {
            show_digit(' ');
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void log_data(void *pvParam) {
    uint64_t last_log = 0;
    
    while(1) {
        uint64_t now = esp_timer_get_time() / 1000;
        
        if ((now - last_log) >= LOG_INTERVAL) {
            if (sd_ready) {
                write_log();
                last_log = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {
    
    setup_gpio();
    setup_pwm();
    setup_i2c();
    setup_adc();
    
    lcd.address = LCD_ADDR;
    lcd.num = I2C_NUM;
    lcd.backlight = 1;
    lcd.size = LCD_SIZE;
    lcd_i2c_init(&lcd);
    
    sd_ready = init_sd();
    
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_cursor_set(&lcd, 0, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);

    mutex = xSemaphoreCreateMutex();
    btn_queue = xQueueCreate(10, sizeof(int));
    
    xTaskCreate(read_temp, "temp", 2048, NULL, 5, NULL);
    xTaskCreate(handle_buttons, "buttons", 2048, NULL, 4, NULL);
    xTaskCreate(sound_alarm, "alarm", 2048, NULL, 3, NULL);
    xTaskCreate(update_lcd, "lcd", 2048, NULL, 2, NULL);
    xTaskCreate(update_display, "display", 2048, NULL, 2, NULL);
    xTaskCreate(log_data, "sd", 4096, NULL, 1, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}