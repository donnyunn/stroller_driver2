#include "ledbar.h"

#define GPIO_LEDBAR GPIO_NUM_21

volatile bool led_state = 0;
int blink = 0;
bool toggle = false;

void ledbar_toggle(void)
{
    toggle = true;
}

void ledbar_on(void)
{
    gpio_set_level(GPIO_LEDBAR, 1);
    led_state = true;
}

void ledbar_off(void)
{
    gpio_set_level(GPIO_LEDBAR, 0);
    led_state = false;
}

void ledbar_blink_start(int num)
{
    blink = num;
}

void ledbar_blink_task(void* arg)
{
    for (;;) {
        if (blink != 0) {
            bool state = led_state;
            ledbar_off();
            for (;blink != 0; blink--) {
                vTaskDelay(250 / portTICK_PERIOD_MS);
                ledbar_on();
                vTaskDelay(250 / portTICK_PERIOD_MS);
                ledbar_off();
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
            if (state) ledbar_on();
        } else if (toggle) {
            if (led_state) {
                ledbar_off();
            } else {
                ledbar_on();
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
            toggle = false;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void ledbar_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_LEDBAR);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    ledbar_off();

    xTaskCreate(ledbar_blink_task, "ledbar_blink_task", 2048, NULL, 10, NULL);
}