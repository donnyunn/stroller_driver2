#include "driver.h"

#include "crc.h"
#include "datatypes.h"
#include "buffer.h"

#define TAG "motor driver"

static const int RX_BUF_SIZE = 1024;

#define DIRECT_CTRL 0
#define SLOWSTART_CTRL 1
#define IIR_FACTOR 64 // 1~256

#define TXD1_PIN (GPIO_NUM_26)
#define RXD1_PIN (GPIO_NUM_27)

#define TXD2_PIN (GPIO_NUM_17)
#define RXD2_PIN (GPIO_NUM_16)

/* Brake Current List */
#define BRAKE1_CURRENT 7
#define BRAKE2_CURRENT 7 // 조이스틱 놓았을 경우로 선택한 상태
#define BRAKE3_CURRENT 7
#define BRAKE4_CURRENT 25 // 브레이크 버튼 눌렀을 경우로 선택한 상태

/* PID 게인 값 조정 */
#define PID_KP 0.008
#define PID_KI 0.002
#define PID_KD 0.0001

/* RPM Feedback Cut Value List */
/* Pre-Calculation Method
 * Diameter (D) = 0.25 m, 
 * Pole-pair (n) = 14 
 * Ref Velocity (V) = 10000 m/h
 * SET ERPM = (V / 60) / (D * Pi) * n = 2970
 */
#define RPM_LIMIT_1 700 // 2.4km/h
#define RPM_LIMIT_2 1188 // 4km/h
#define RPM_LIMIT_3 2080 // 7km/h

brake_mode_t brake_mode = BRAKE_MODE_1;

infoPackage info1;
infoPackage info2;
driver_t * this;
uint8_t driver1_info[256];
uint8_t driver2_info[256];

bool processReadPacket(infoPackage * info, uint8_t * message)
{
    COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			info->tempFET            = buffer_get_float16(message, 10.0, &ind);
			info->tempMotor          = buffer_get_float16(message, 10.0, &ind);
			info->avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);
			info->avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);
			ind += 8; // Skip the next 8 bytes
			info->dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);
			info->rpm 				= buffer_get_int32(message, &ind);
			info->inpVoltage 		= buffer_get_float16(message, 10.0, &ind);
			info->ampHours 			= buffer_get_float32(message, 10000.0, &ind);
			info->ampHoursCharged 	= buffer_get_float32(message, 10000.0, &ind);
			ind += 8; // Skip the next 8 bytes 
			info->tachometer 		= buffer_get_int32(message, &ind);
			info->tachometerAbs 		= buffer_get_int32(message, &ind);
			return true;

		break;

		default:
			return false;
		break;
	}
}

bool unpackPayload(uint8_t * message, int lenMes, uint8_t * payload)
{
    uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);
	
	if (crcPayload == crcMessage) {
		return true;
	} else {
		return false;
	}
}

int packSendPayload(uint8_t * payload, int lenPay, uart_port_t port) 
{
    uint16_t crcPayload = crc16(payload, lenPay);
    int count = 0;
    uint8_t messageSend[256];

    if (lenPay <= 256) {
        messageSend[count++] = 2;
        messageSend[count++] = lenPay;
    } else {
        messageSend[count++] = 3;
        messageSend[count++] = (uint8_t)(lenPay >> 8);
        messageSend[count++] = (uint8_t)(lenPay & 0xff);
    }

    memcpy(&messageSend[count], payload, lenPay);
    count += lenPay;

    messageSend[count++] = (uint8_t)(crcPayload >> 8);
    messageSend[count++] = (uint8_t)(crcPayload & 0xff);
    messageSend[count++] = 3;
    messageSend[count] = '\0';

    return uart_write_bytes(port, messageSend, count);
}

void setCurrent(float current, uart_port_t port) 
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);

	packSendPayload(payload, 5, port);
}

void setBrakeCurrent(float brakeCurrent, uart_port_t port)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, 5, port);
}

void setRPM(float rpm, uart_port_t port) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_RPM ;
	buffer_append_int32(payload, (int32_t)(rpm), &index);

	packSendPayload(payload, 5, port);
}

void setDuty(float duty, uart_port_t port)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, 5, port);
}

bool getVesc1Values(void)
{
    uint8_t command[1] = { COMM_GET_VALUES };
	packSendPayload(command, 1, UART_NUM_1);
    return true;
}
bool getVesc2Values(void)
{
    uint8_t command[1] = { COMM_GET_VALUES };
	packSendPayload(command, 1, UART_NUM_2);
    return true;
}

uint16_t driver_get_speed(int num) 
{
    return 0;
}

void driver_set_direction(bool dir)
{
    this->direction = dir;
}

void driver_set_brake_mode(brake_mode_t mode)
{
    brake_mode = mode;
}

brake_mode_t driver_get_brake_mode(void)
{
    return brake_mode;
}

void driver_set_brake(brake_mode_t brake)
{
    switch (brake) {
        case BRAKE_MODE_NONE:
            setBrakeCurrent(0, UART_NUM_1);
            setBrakeCurrent(0, UART_NUM_2);
            break;
        case BRAKE_MODE_1:
            setBrakeCurrent(BRAKE1_CURRENT, UART_NUM_1);
            setBrakeCurrent(BRAKE1_CURRENT, UART_NUM_2);
            brake_mode = BRAKE_MODE_1;
        break;
        case BRAKE_MODE_2:
            setBrakeCurrent(BRAKE2_CURRENT, UART_NUM_1);
            setBrakeCurrent(BRAKE2_CURRENT, UART_NUM_2);
            brake_mode = BRAKE_MODE_2;
        break;
        case BRAKE_MODE_3:
            setBrakeCurrent(BRAKE3_CURRENT, UART_NUM_1);
            setBrakeCurrent(BRAKE3_CURRENT, UART_NUM_2);
            brake_mode = BRAKE_MODE_3;
        break;
        case BRAKE_MODE_MAX:
            setBrakeCurrent(BRAKE4_CURRENT, UART_NUM_1);
            setBrakeCurrent(BRAKE4_CURRENT, UART_NUM_2);
        break;
    }
}

brake_mode_t driver_get_brake(void)
{
    return brake_mode;
}

void driver_emergency_brake(void)
{
    setBrakeCurrent(BRAKE4_CURRENT, UART_NUM_1);
    setBrakeCurrent(BRAKE4_CURRENT, UART_NUM_2);
}

static float g_i_r = 0;
static float g_i_l = 0;
static float g_pid_last_error_r = 0;
static float g_pid_last_error_l = 0;
float PID_R(float error, float dt, float kp, float ki)
{
    float output = 0;
    float p = 0;
    //static float i_r = 0;
    float d = 0;
    //static float pid_last_error_r = 0;

    p = error * kp;
    g_i_r += error * ki * dt;
    d = (error - g_pid_last_error_r) / dt * PID_KD;

    output = p + g_i_r + d;

    g_pid_last_error_r = error;

    return output;
}
float PID_L(float error, float dt, float kp, float ki)
{
    float output = 0;
    float p = 0;
    //static float i_l = 0;
    float d = 0;
    //static float pid_last_error_l = 0;

    p = error * kp;
    g_i_l += error * ki * dt;
    d = (error - g_pid_last_error_l) / dt * PID_KD;

    output = p + g_i_l + d;

    g_pid_last_error_l = error;

    return output;
}

#define _DUTY 0
#define _CURR 1
#define _RPM  0
void driver_set_speed(int16_t speed, int16_t steering)
{
#if _DUTY
    /* for duty */
    float _speed = (float)speed*0.0002;
    float _steering = (float)steering*0.0001;
    static float dutyR = 0;
    static float dutyL = 0;
    float tmpR, tmpL;
#elif _CURR
    /* for current */
    // 조이스틱 조작에 대한 전류량 변환 식 (Current = a * Analog Value)
    const float a1 = 0.01f;
    const float a2 = 0.005f;
    float _speed = (float)speed*a1; // mapping coefficient
    float _steering = (float)steering*a2; // mapping coefficient
    static float _speed_previous = 0;
    static float _steering_previous = 0;
    // 실제 VESC에 전달할 전류값
    static float currR = 0;
    static float currL = 0;
    static float currR_pre = 0;
    static float currL_pre = 0;

    static bool reaccelFlag_l = false;
    static bool reaccelFlag_r = false;
    float tmp_kp = PID_KP;
    float tmp_ki = PID_KI;
    float tmp_ksteering = 1;
#elif _RPM
    /* for rpm */
    float _speed = (float)speed;
    float _steering = (float)steering;
    static float rpmR = 0;
    static float rpmL = 0;
    float tmpR, tmpL;
#endif
    /* 조이스틱을 놓았을 경우 */
    if (speed == 0 && steering == 0) {
        setBrakeCurrent(BRAKE2_CURRENT, UART_NUM_1);
        setBrakeCurrent(BRAKE2_CURRENT, UART_NUM_2);
        _speed_previous = 0;
        _steering_previous = 0;
        currR = 0;
        currL = 0;
        currL_pre = 0;
        currR_pre = 0;
        
        reaccelFlag_l = false;
        reaccelFlag_r = false;

        g_i_l = 0;
        g_i_r = 0;
        g_pid_last_error_l = 0;
        g_pid_last_error_r = 0;

    } else {
#if _DUTY
        dutyR = _speed + _steering;
        dutyL = _speed - _steering;
        setDuty(dutyR, UART_NUM_2);
        setDuty(dutyL, UART_NUM_1);
#elif _CURR
#if DIRECT_CTRL
        /* 제어 명령 상황에 따라 구분하여 결정 */
        if (speed == 0) {
            if (_steering == 0) {
                currR = 0;
                currL = 0;
            } else if (_steering > 0) {
                currR = _steering;
                currL = 0;
            } else {
                currR = 0;
                currL = -1*_steering;
            }
        } else if (speed > 0) {
            if (_speed > _speed_previous) {
                _speed = (9*_speed_previous + _speed) / 10;
            }
            
            if (steering > 0) {
                if (_steering > _steering_previous) {
                    _steering = (_steering_previous + _steering) / 2;
                }
            } else {
                if (_steering < _steering_previous) {
                    _steering = (_steering_previous + _steering) / 2;
                }
            }
            _speed_previous = _speed;
            _steering_previous = _steering;

            currR = _speed + _steering;
            currL = _speed - _steering;
            if (currR < _speed) currR = 0;
            if (currL < _speed) currL = 0;
        } else if (speed < 0) {
            if (_speed < _speed_previous) {
                _speed = (9*_speed_previous + _speed) / 10;
            }
            
            if (steering > 0) {
                if (_steering > _steering_previous) {
                    _steering = (_steering_previous + _steering) / 2;
                }
            } else {
                if (_steering < _steering_previous) {
                    _steering = (_steering_previous + _steering) / 2;
                }
            }
            _speed_previous = _speed;
            _steering_previous = _steering;

            currR = _speed + _steering;
            currL = _speed - _steering;
            if (currR > _speed) currR = 0;
            if (currL > _speed) currL = 0;
        }
        printf("%ld, %ld\n", info1.rpm, info2.rpm);
        
        /* RPM 상한선 적용, 관성주행 시도 */
        const float coeffCurrRelease1 = 0.3f;
        const float coeffCurrRelease2 = 0.3f;
        const float coeffCurrRelease3 = 0.3f;
        if (brake_mode == BRAKE_MODE_1) {
            currL = (5*currL_pre + currL) / 6;
            currR = (5*currR_pre + currR) / 6;
            if (info1.rpm > RPM_LIMIT_1) {
                currL_pre = currL * coeffCurrRelease1;
                reaccelFlag_l = true;
            } else {
                if (reaccelFlag_l) currL_pre = (3*currL_pre + currL) / 4;
                else currL_pre = currL;
            }
            if (info2.rpm > RPM_LIMIT_1) {
                currR_pre = currR * coeffCurrRelease1;
                reaccelFlag_r = true;
            } else {
                if (reaccelFlag_r) currR_pre = (3*currR_pre + currR) / 4;
                else currR_pre = currR;
            }
        } else if (brake_mode == BRAKE_MODE_2) {
            currL = (7*currL_pre + currL) / 8;
            currR = (7*currR_pre + currR) / 8;
            if (info1.rpm > RPM_LIMIT_2) {
                currL_pre = currL * coeffCurrRelease2;
                reaccelFlag_l = true;
            } else {
                if (reaccelFlag_l) currL_pre = (3*currL_pre + currL) / 4;
                else currL_pre = currL;
            }
            if (info2.rpm > RPM_LIMIT_2) {
                currR_pre = currR * coeffCurrRelease2;
                reaccelFlag_r = true;
            } else {
                if (reaccelFlag_r) currR_pre = (3*currR_pre + currR) / 4;
                else currR_pre = currR;
            }
        } else if (brake_mode == BRAKE_MODE_3) {
            currL = (9*currL_pre + currL) / 10;
            currR = (9*currR_pre + currR) / 10;
            if (info1.rpm > RPM_LIMIT_3) {
                currL_pre = currL * coeffCurrRelease3;
            } else {
                currL_pre = currL;
            }
            if (info2.rpm > RPM_LIMIT_3) {
                currR_pre = currR * coeffCurrRelease3;
            } else {
                currR_pre = currR;
            }
        }
        
#elif SLOWSTART_CTRL
        /* RPM 상한선 적용 */
        int rpm_limit = RPM_LIMIT_1;
        if (brake_mode == BRAKE_MODE_1) {
            tmp_kp = PID_KP/4;
            tmp_ki = PID_KI/2;
            tmp_ksteering = 3;
            rpm_limit = RPM_LIMIT_1;
        } else if (brake_mode == BRAKE_MODE_2) {
            tmp_kp = PID_KP;
            tmp_ki = PID_KI;
            tmp_ksteering = 1;
            rpm_limit = RPM_LIMIT_2;
        } else if (brake_mode == BRAKE_MODE_3) {
            tmp_kp = PID_KP;
            tmp_ki = PID_KI;
            tmp_ksteering = 1;
            rpm_limit = RPM_LIMIT_3;
        }
        if (speed < 0) _speed = (float)speed * 1.25 * rpm_limit / 2000;
        else _speed = (float)speed * rpm_limit / 2000;
        if (steering < 0) _steering = (float)steering * 1.25 * rpm_limit / 2000;
        else _steering = (float)steering * rpm_limit / 2000;
        
        /* 제어 명령 상황에 따라 구분하여 결정 */
        if (speed == 0) {
            if (_steering > 0) {
                currR = (float)steering*a2;// PID_R(tmp_ksteering*_steering - info2.rpm, 0.11f, tmp_kp, tmp_ki);
                currL = 0;
                setBrakeCurrent(BRAKE4_CURRENT, UART_NUM_1);
            } else {
                currR = 0;
                setBrakeCurrent(BRAKE4_CURRENT, UART_NUM_2);
                currL = -1*(float)steering*a2;//PID_L(-1*tmp_ksteering*_steering - info1.rpm, 0.11f, tmp_kp, tmp_ki);
            }
        } else if (speed > 0) {
            if (_speed > _speed_previous) {
                _speed = (5*_speed_previous + _speed) / 6;
            }
            
            if (steering > 0) {
                if (_steering > _steering_previous) {
                    _steering = (_steering_previous + _steering) / 2;
                }
            } else {
                if (_steering < _steering_previous) {
                    _steering = (_steering_previous + _steering) / 2;
                }
            }
            _speed_previous = _speed;
            _steering_previous = _steering;

            //currR = PID_R(_speed + (tmp_ksteering*_steering) - info2.rpm, 0.11f, tmp_kp, tmp_ki);
            //currL = PID_L(_speed - (tmp_ksteering*_steering) - info1.rpm, 0.11f, tmp_kp, tmp_ki);
            currR = PID_R(_speed - info2.rpm, 0.11f, tmp_kp, tmp_ki) + (float)steering*a2;
            currL = PID_L(_speed - info1.rpm, 0.11f, tmp_kp, tmp_ki) - (float)steering*a2;
            //if (currR < _speed) currR = 0;
            //if (currL < _speed) currL = 0;
        } else if (speed < 0) {
            if (_speed < _speed_previous) {
                _speed = (5*_speed_previous + _speed) / 6;
            }
            
            if (steering > 0) {
                if (_steering > _steering_previous) {
                    _steering = (_steering_previous + _steering) / 2;
                }
            } else {
                if (_steering < _steering_previous) {
                    _steering = (_steering_previous + _steering) / 2;
                }
            }
            _speed_previous = _speed;
            _steering_previous = _steering;

            //currR = PID_R(_speed + (tmp_ksteering*_steering) - info2.rpm, 0.11f, tmp_kp, tmp_ki);
            //currL = PID_L(_speed - (tmp_ksteering*_steering) - info1.rpm, 0.11f, tmp_kp, tmp_ki);
            currR = PID_R(_speed - info2.rpm, 0.11f, tmp_kp, tmp_ki) + (float)steering*a2;
            currL = PID_L(_speed - info1.rpm, 0.11f, tmp_kp, tmp_ki) - (float)steering*a2;
            //if (currR > _speed) currR = 0;
            //if (currL > _speed) currL = 0;
        }
        ESP_LOGI(TAG, "%.2f, %.2f", currR, currL);
#endif
        /* VESC로 전달 */
        if (currL == 0) {
            setBrakeCurrent(BRAKE1_CURRENT, UART_NUM_1);
        } else {
            setCurrent(currL, UART_NUM_1);
        }
        if (currR == 0) {
            setBrakeCurrent(BRAKE1_CURRENT, UART_NUM_2);
        } else {
            setCurrent(currR, UART_NUM_2);
        }
#elif _RPM  
#if DIRECT_CTRL
        rpmR = _speed + _steering;
        rpmL = _speed - _steering;
#elif SLOWSTART_CTRL
        tmpR = _speed + _steering;
        tmpL = _speed - _steering;
        /* IIR Filter */
        if (tmpR > 0) {
            if (rpmR < tmpR) {
                rpmR = ((IIR_FACTOR-1)*rpmR + tmpR) / IIR_FACTOR;
            } else {
                rpmR = tmpR;
            }
        } else {
            if (rpmR > tmpR) {
                rpmR = ((IIR_FACTOR-1)*rpmR + tmpR) / IIR_FACTOR;
            } else {
                rpmR = tmpR;
            }
        }
        if (tmpL > 0) {
            if (rpmL < tmpL) {
                rpmL = ((IIR_FACTOR-1)*rpmL + tmpL) / IIR_FACTOR;
            } else {
                rpmL = tmpL;
            }
        } else {
            if (rpmL > tmpL) {
                rpmL = ((IIR_FACTOR-1)*rpmL + tmpL) / IIR_FACTOR;
            } else {
                rpmL = tmpL;
            }
        }
#endif
        /* Upper Limit Cut */
        if (brake_mode == BRAKE_MODE_1) {
            if (rpmR > RPM_LIMIT_1) rpmR = RPM_LIMIT_1;
            else if (rpmR < -RPM_LIMIT_1) rpmR = -RPM_LIMIT_1;
            if (rpmL > RPM_LIMIT_1) rpmL = RPM_LIMIT_1;
            else if (rpmL < -RPM_LIMIT_1) rpmL = -RPM_LIMIT_1;
        } else if (brake_mode == BRAKE_MODE_2) {
            if (rpmR > RPM_LIMIT_2) rpmR = RPM_LIMIT_2;
            else if (rpmR < -RPM_LIMIT_2) rpmR = -RPM_LIMIT_2;
            if (rpmL > RPM_LIMIT_2) rpmL = RPM_LIMIT_2;
            else if (rpmL < -RPM_LIMIT_2) rpmL = -RPM_LIMIT_2;
        }
        setRPM(rpmR, UART_NUM_2);
        setRPM(rpmL, UART_NUM_1);
#endif
    }
}

static void driver_rx1_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        getVesc1Values();
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 10 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            unpackPayload(data, rxBytes, driver1_info);
            if (rxBytes > 55) {
                if (processReadPacket(&info1, driver1_info)) {
                    // ESP_LOGI(TAG, "rpmL: %ld", info1.rpm);
                }
            }
        }
    }
    free(data);
}

static void driver_rx2_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        getVesc2Values();
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 10 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            unpackPayload(data, rxBytes, driver2_info);
            if (rxBytes > 55) {
                if (processReadPacket(&info2, driver2_info)) {
                    // ESP_LOGI(TAG, "rpmR: %ld", info2.rpm);
                }
            }
        }
    }
    free(data);
}

void driver_init(driver_t* driver)
{
    this = driver;

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // semaphore = xSemaphoreCreateMutex();
    // xSemaphoreGive(semaphore);

    xTaskCreate(driver_rx1_task, "driver_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(driver_rx2_task, "driver_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}