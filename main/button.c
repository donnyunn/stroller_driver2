#include "button.h"

#define BUTTON_IO GPIO_NUM_19

volatile bool button_pressed = false;

static void IRAM_ATTR button_isr_handler(void* arg)
{
    button_pressed = true;
}

void releaseButton(void)
{
    button_pressed = false;
}

bool isButtonPressed(void)
{
    return button_pressed;
}

void button_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL<<BUTTON_IO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_IO, button_isr_handler, (void*)BUTTON_IO);

    releaseButton();
}