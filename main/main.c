/*
   This code is written by Rotom Ltd. but it is in the Public Domain 
   (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This file is a main code for the controller device.
*
****************************************************************************/

#include "main.h"

#define TAG "rotom app"

typedef enum {
    WORK_INIT = 0,
    WORK_ADVERTISING,
    WORK_CONNECTED,
} main_work_t;
main_work_t work = WORK_INIT;

ble_spp_server_t spp;
packet_t packet;
driver_t driver = {
    .direction = true,
    .speed = 0,
    .steering = 0,
};

void app_main(void)
{
    uint8_t * spp_data;
    uint16_t temp;
    drive_mode_t drive_mode = DRIVE_MODE_STOP;

    while (1) {
        vTaskDelay(10/portTICK_PERIOD_MS);
        switch (work) {
            case WORK_INIT:
                driver_init(&driver);
                battery_init();
                ledbar_init();
                button_init();
                ble_spp_server_init(&spp);
                driver_emergency_brake();
                work = WORK_ADVERTISING;
            break;
            case WORK_ADVERTISING:
                if (isConnected()) {
                    work = WORK_CONNECTED;
                } else {
                    driver_emergency_brake();
                    vTaskDelay(500/portTICK_PERIOD_MS);
                }
            break;
            case WORK_CONNECTED:
                // 컨트롤러 명령 수신 이벤트 발생 처리 (timeout = 500ms)
                if (xQueueReceive(spp.data_queue, &spp_data, 500/portTICK_RATE_MS)) {
                    // 조이스틱 값 디코딩
                    pakcet_decoding(&packet, spp_data, 10);
                    free(spp_data);

                    // LED bar 제어 명령 처리
                    if (packet.ledbar != 0) {
                        ledbar_toggle();
                    }

                    if (packet.brake != 0) {
                        // 브레이크 명령 처리
                        driver_emergency_brake();

                    } else if (battery_check() == false) {
                        // Low-level Battery 상황
                        driver_emergency_brake();
                        ESP_LOGI(TAG, "Low Battery Power!");
                        ledbar_blink_start(1);

                    } else {
                        // 주행 명령 처리
                        driver_set_speed(packet.forward, packet.clockwise);

                    }       
                } else {
                    // 컨트롤러 명령 수신 대기 Timeout 상황
                    driver_emergency_brake();
                    if (!isConnected()) {
                        work = WORK_ADVERTISING;
                    }

                }
            break;
        }

        // 사용자 버튼 이벤트 처리
        if (isButtonPressed()) {
            brake_mode_t brake = driver_get_brake_mode();
            switch (brake) {
                case BRAKE_MODE_NONE:
                    driver_set_brake_mode(BRAKE_MODE_1);
                    ledbar_blink_start(1);
                break;
                case BRAKE_MODE_1:
                    driver_set_brake_mode(BRAKE_MODE_2);
                    ledbar_blink_start(2);
                break;
                case BRAKE_MODE_2:
                    driver_set_brake_mode(BRAKE_MODE_3);
                    ledbar_blink_start(3);
                break;
                case BRAKE_MODE_3:
                    driver_set_brake_mode(BRAKE_MODE_1);
                    ledbar_blink_start(1);
                break;
                case BRAKE_MODE_MAX:
                    driver_set_brake_mode(BRAKE_MODE_1);
                    ledbar_blink_start(1);
                break;
            } 
            ESP_LOGI(TAG, "Brake Mode = %d", driver_get_brake_mode());
            vTaskDelay(500/portTICK_PERIOD_MS);
            releaseButton();
        }
    }
}
