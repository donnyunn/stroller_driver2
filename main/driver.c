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

#define BRAKE1_CURRENT 2
#define BRAKE2_CURRENT 5
#define BRAKE3_CURRENT 7
#define BRAKE4_CURRENT 50

brake_mode_t brake_mode = BRAKE_MODE_1;

infoPackage info;
driver_t * this;
uint8_t driver2_info[256];

// SemaphoreHandle_t semaphore;

bool processReadPacket(uint8_t * message)
{
    COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			info.tempFET            = buffer_get_float16(message, 10.0, &ind);
			info.tempMotor          = buffer_get_float16(message, 10.0, &ind);
			info.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);
			info.avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);
			ind += 8; // Skip the next 8 bytes
			info.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);
			info.rpm 				= buffer_get_int32(message, &ind);
			info.inpVoltage 		= buffer_get_float16(message, 10.0, &ind);
			info.ampHours 			= buffer_get_float32(message, 10000.0, &ind);
			info.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &ind);
			ind += 8; // Skip the next 8 bytes 
			info.tachometer 		= buffer_get_int32(message, &ind);
			info.tachometerAbs 		= buffer_get_int32(message, &ind);
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

void setVescValues(void)
{
    // setRPM(this->speed, UART_NUM_1);
    // setRPM(this->speed, UART_NUM_2);
    // if (this->direction) {

    // } else {

    // }
}

bool getVescValues(void)
{
    uint8_t command[1] = { COMM_GET_VALUES };
	packSendPayload(command, 1, UART_NUM_1);
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
    // setRPM(0, UART_NUM_1);
    // setRPM(0, UART_NUM_2);
    setBrakeCurrent(BRAKE4_CURRENT, UART_NUM_1);
    setBrakeCurrent(BRAKE4_CURRENT, UART_NUM_2);
}

#define _DUTY 0
#define _CURR 0
#define _RPM  1
#define RPM_LIMIT_1 1000
#define RPM_LIMIT_2 1500
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
    float _speed = (float)speed*0.005;
    float _steering = (float)steering*0.003;
    float currR, currL;
#elif _RPM
    /* for rpm */
    float _speed = (float)speed;
    float _steering = (float)steering;
    static float rpmR = 0;
    static float rpmL = 0;
    float tmpR, tmpL;
#endif
    ESP_LOGI(TAG, "%d, %d", speed, steering);

    if (speed == 0 && steering == 0) {
        // dutyR = 0;
        // dutyL = 0;
        // driver_set_brake(BRAKE_MODE_MAX);
        rpmR = 0;
        rpmL = 0;
        setRPM(rpmR, UART_NUM_2);
        setRPM(rpmL, UART_NUM_1);
    } else {
#if _CURR
        currR = _speed + _steering;
        currL = _speed - _steering;
        setCurrent(currR, UART_NUM_2);
        setCurrent(currL, UART_NUM_1);
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

static void driver_rx_task(void *arg)
{
    // static const char *RX_TASK_TAG = "RX2_TASK";
    // esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        getVescValues();
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            unpackPayload(data, rxBytes, driver2_info);
            if (rxBytes > 55) {
                if (processReadPacket(driver2_info)) {
                    ESP_LOGI(TAG, "avgMotorCurrent: %f", info.avgMotorCurrent);
                    ESP_LOGI(TAG, "avgInputCurrent: %f", info.avgInputCurrent);
                    ESP_LOGI(TAG, "dutyCycleNow: %f", info.dutyCycleNow);
                    ESP_LOGI(TAG, "rpm: %ld", info.rpm);
                    ESP_LOGI(TAG, "inputVoltage: %f", info.inpVoltage);
                    ESP_LOGI(TAG, "ampHours: %f", info.ampHours);
                    ESP_LOGI(TAG, "ampHoursCharges: %f", info.ampHoursCharged);
                    ESP_LOGI(TAG, "tachometer: %ld", info.tachometer);
                    ESP_LOGI(TAG, "tachometerAbs: %ld", info.tachometerAbs);
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

    // xTaskCreate(driver_rx1_task, "driver_rx1_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(driver_tx1_task, "driver_tx1_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(driver_rx_task, "driver_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(driver_tx_task, "driver_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}