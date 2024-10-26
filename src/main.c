/*
* SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

#include "driver/gpio.h"

#define LED_RED GPIO_NUM_0
#define LED_GREEN GPIO_NUM_2
#define LED_BLUE GPIO_NUM_4

// Define the GPIO pins for the interrupt sources
#define ISR_LED_RED  25
#define ISR_LED_GREEN  26
#define ISR_LED_BLUE  27

bool gpio_value = false;

static void reset_to_output(gpio_num_t gpio_num) {
    if (gpio_reset_pin(gpio_num)) {
        printf("Reset failed\n");
    }

    if (gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT)) {
        printf("Set Direction failed\n");
    }
}

static void reset_led(gpio_num_t gpio_num) {
    reset_to_output(gpio_num);
    if (gpio_set_level(gpio_num, false)) {
        printf("failed gpio_set_level(%i, false)\n", gpio_num);
    }
}

static void toggle_led(gpio_num_t gpio_num) {
    #define MAX_GPIOS 32 // Adjust if you have more GPIOs

    // Static array to store the last written GPIO values
    static int gpio_states[MAX_GPIOS] = {0};

    if (gpio_num < 0 || gpio_num >= MAX_GPIOS) {
        printf("Invalid GPIO number: %d\n", gpio_num);
        return;
    }

    // Toggle the state in the array
    gpio_states[gpio_num] = !gpio_states[gpio_num];

    // Set the GPIO level based on the updated state
    if (gpio_set_level(gpio_num, gpio_states[gpio_num])) {
        printf("failed gpio_set_level(%d, %d)\n", gpio_num, gpio_states[gpio_num]);
    } else {
        printf("GPIO %d toggled to %d\n", gpio_num, gpio_states[gpio_num]);
    }
}

// Debounce time in milliseconds
#define DEBOUNCE_TIME_MS   50

// Variables to store the last interrupt time for each GPIO
static uint32_t last_interrupt_time_1 = 0;
static uint32_t last_interrupt_time_2 = 0;
static uint32_t last_interrupt_time_3 = 0;

// ISR handler for GPIO 25 (triggers Red LED)
static void IRAM_ATTR gpio_isr_handler_1(void* arg) {
    printf("gpio_isr_handler_1");
    const uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - last_interrupt_time_1 > DEBOUNCE_TIME_MS) {
        gpio_intr_disable(ISR_LED_RED);
        toggle_led(LED_RED);
        gpio_intr_enable(ISR_LED_RED);
        last_interrupt_time_1 = current_time;
    }
}

// ISR handler for GPIO 26 (triggers Green LED)
static void IRAM_ATTR gpio_isr_handler_2(void* arg) {
    printf("gpio_isr_handler_2");
    const uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - last_interrupt_time_2 > DEBOUNCE_TIME_MS) {
        gpio_intr_disable(ISR_LED_BLUE);
        toggle_led(LED_GREEN);
        gpio_intr_enable(ISR_LED_BLUE);
        last_interrupt_time_2 = current_time;
    }
}

// ISR handler for GPIO 27 (triggers Blue LED)
static void IRAM_ATTR gpio_isr_handler_3(void* arg) {
    printf("gpio_isr_handler_3");
    const uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_interrupt_time_2 > DEBOUNCE_TIME_MS) {
        gpio_intr_disable(ISR_LED_GREEN);
        toggle_led(LED_BLUE);
        gpio_intr_enable(ISR_LED_GREEN);
        last_interrupt_time_2 = current_time;
    }
}

// Function to configure GPIO and enable interrupt
static void configure_gpio_interrupt(const gpio_num_t gpio_num, const gpio_isr_t isr_handler) {
    if (gpio_reset_pin(gpio_num)) {
        printf("Reset failed\n");
    }


    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << gpio_num);
    
    if (gpio_config(&io_conf)) {
        printf("config failed\n");
    }

    if ( gpio_isr_handler_add(gpio_num, isr_handler, NULL) ) {
        printf("gpio_isr_handler_add failed\n");
    }

}

void app_main(void)
{
    printf("Hello world!\n");

    reset_led(LED_RED);
    reset_led(LED_GREEN);
    reset_led(LED_BLUE);

    // Install the GPIO ISR service
    if (gpio_install_isr_service()) {
        printf("Failed to start gpio_install_isr_service ");
    }

    configure_gpio_interrupt(ISR_LED_RED, gpio_isr_handler_1);
    configure_gpio_interrupt(ISR_LED_GREEN, gpio_isr_handler_2);
    configure_gpio_interrupt(ISR_LED_BLUE, gpio_isr_handler_3);

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%uMB %s flash\n", flash_size / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Jumping to GPIO in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Jumping now.\n");
    fflush(stdout);

    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}