#include "main.h"
#include "LoRa.h"
#include "stm32l1xx_hal.h"

#define COMMAND_READ_PREFIX 0xC1
#define COMMAND_WRITE_PREFIX 0xC0
#define SLEEP_TIME 1000

GPIO_TypeDef *GPIOx_M0;
GPIO_TypeDef *GPIOx_M1;
GPIO_TypeDef *GPIOx_AUX;
GPIO_TypeDef *GPIOx_LED;
uint16_t PIN_M0;
uint16_t PIN_M1;
uint16_t PIN_AUX;
uint16_t PIN_LED;
UART_HandleTypeDef *LoRa_UART;
UART_HandleTypeDef *COM_UART;

uint8_t command_buffer[4];
uint8_t response_buffer[4];
uint8_t com_response_buffer[12];

void LoRa_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2)
{
	uint8_t preamble[18] = "Init LoRa Module:\n";
	uint8_t postamble[11] = "Init done.\n";

	LoRa_UART = huart1;
	COM_UART = huart2;

	GPIOx_M0 = M0_GPIO_Port;
	GPIOx_M1 = M1_GPIO_Port;
	GPIOx_AUX = AUX_GPIO_Port;
	GPIOx_LED = LED_GPIO_Port;

	PIN_M0 = M0_Pin;
	PIN_M1 = M1_Pin;
	PIN_LED = LED_Pin;
	PIN_AUX = AUX_Pin;

	HAL_UART_Transmit(COM_UART, preamble, sizeof(preamble), SLEEP_TIME);
	LoRa_ModeSelect(MODE_CONFIG);
	LoRa_ReadRegister(ADDH);
	LoRa_ReadRegister(ADDL);
	LoRa_ReadRegister(NETID);
	LoRa_ReadRegister(REG0);
	LoRa_ReadRegister(REG1);
	LoRa_ReadRegister(REG2);
	LoRa_ReadRegister(REG3);
    HAL_UART_Transmit(COM_UART, postamble, sizeof(postamble), SLEEP_TIME);
}

// 00 - Normal mode
// 01 - WOR mode
// 10 - Configuration mode
// 11 - Deep sleep mode
void LoRa_ModeSelect(enum Mode mode) {
	switch (mode) {
	case MODE_NORMAL:
	    HAL_GPIO_WritePin(GPIOx_M0, PIN_M0, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOx_M1, PIN_M1, GPIO_PIN_RESET);
	    break;
	case MODE_WOR:
	    HAL_GPIO_WritePin(GPIOx_M0, PIN_M0, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOx_M1, PIN_M1, GPIO_PIN_RESET);
	    break;
	case MODE_CONFIG:
	    HAL_GPIO_WritePin(GPIOx_M0, PIN_M0, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOx_M1, PIN_M1, GPIO_PIN_SET);
	    break;
	case MODE_SLEEP:
	    HAL_GPIO_WritePin(GPIOx_M0, PIN_M0, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOx_M1, PIN_M1, GPIO_PIN_SET);
	    break;
	}
	HAL_Delay(100);
}

// Commands are formatted as Cx+Address+Length+Parameter
// Example "C10501" - reading register 05H
// It was decided to let user only modify one register at a time, so length is set at 0x1
void LoRa_ReadRegister(uint8_t address) {
	uint8_t command_index = 0;
    command_buffer[command_index++] = COMMAND_READ_PREFIX;
    command_buffer[command_index++] = address;
    command_buffer[command_index++] = 0x1;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, SLEEP_TIME);
    HAL_UART_Receive(LoRa_UART, response_buffer, sizeof(response_buffer), SLEEP_TIME);
    LoRa_Convert(response_buffer, com_response_buffer, sizeof(response_buffer));
    HAL_UART_Transmit(COM_UART, com_response_buffer, sizeof(com_response_buffer), SLEEP_TIME);
}

// To return just the register's value
uint8_t LoRa_ReadRegisterValue(uint8_t address) {
	uint8_t command_index = 0;
    command_buffer[command_index++] = COMMAND_READ_PREFIX;
    command_buffer[command_index++] = address;
    command_buffer[command_index++] = 0x1;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, SLEEP_TIME);
    HAL_UART_Receive(LoRa_UART, response_buffer, sizeof(response_buffer), SLEEP_TIME);
    return response_buffer[3];
}

void LoRa_WriteRegister(uint8_t address, uint8_t parameter) {
	uint8_t command_index = 0;
    command_buffer[command_index++] = COMMAND_WRITE_PREFIX;
    command_buffer[command_index++] = address;
    command_buffer[command_index++] = 0x1;
    command_buffer[command_index++] = parameter;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, SLEEP_TIME);
    HAL_UART_Receive(LoRa_UART, response_buffer, sizeof(response_buffer), SLEEP_TIME);
    LoRa_Convert(response_buffer, com_response_buffer, sizeof(response_buffer));
    HAL_UART_Transmit(COM_UART, com_response_buffer, sizeof(com_response_buffer), SLEEP_TIME);
}

// Register PID has 7 bytes
void LoRa_ReadProductInfo() {
	uint8_t command_index = 0;
	uint8_t product_response_buffer[10];
	uint8_t com_product_response_buffer[30];
	uint8_t preamble[14] = "Product info:\n";
    command_buffer[command_index++] = COMMAND_READ_PREFIX;
    command_buffer[command_index++] = PID;
    command_buffer[command_index++] = 0x7;

	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, 1000);
    HAL_UART_Receive(LoRa_UART, product_response_buffer, sizeof(product_response_buffer), SLEEP_TIME);
    LoRa_Convert(product_response_buffer, com_product_response_buffer, sizeof(product_response_buffer));
    HAL_UART_Transmit(COM_UART, preamble, sizeof(preamble), 1000);
    HAL_UART_Transmit(COM_UART, com_product_response_buffer, sizeof(com_product_response_buffer), SLEEP_TIME);
    HAL_Delay(SLEEP_TIME);
}

void LoRa_Convert(uint8_t *buffer, uint8_t *com_buffer, uint8_t buffer_size) {
	for (int i = 0; i < buffer_size; i++) {
		com_buffer[i * 3] 		= (buffer[i] >> 4) < 10 ? (buffer[i] >> 4) + '0' : (buffer[i] >> 4) - 10 + 'A';
		com_buffer[i * 3 + 1] 	= (buffer[i] & 0x0F) < 10 ? (buffer[i] & 0x0F) + '0' : (buffer[i] & 0x0F) - 10 + 'A';
		com_buffer[i * 3 + 2]	= ' ';
	}
	com_buffer[buffer_size * 3 - 1] = '\n';
}

// Returns to factory default parameters
// Frequency		- 868.125MHz
// Address			- 0x0000
// Channel			- 0x12
// Air data rate	- 2.4kbps
// Baud rate		- 9600
// Parity format	- 8N1
// Power			- 22dbm
void LoRa_ResetFirmware() {
	uint8_t preamble[20] = "Resetting firmware:\n";
	uint8_t postamble[12] = "Reset done.\n";
    HAL_UART_Transmit(COM_UART, preamble, sizeof(preamble), SLEEP_TIME);
    LoRa_WriteRegister(ADDH, 0x0);
    LoRa_WriteRegister(ADDL, 0x0);
    LoRa_WriteRegister(NETID, 0x0);
    LoRa_WriteRegister(REG0, 0x62);
    LoRa_WriteRegister(REG1, 0x00);
    LoRa_WriteRegister(REG2, 0x12);
    LoRa_WriteRegister(REG3, 0x3);
    LoRa_WriteRegister(CRYPT_H, 0x0);
    LoRa_WriteRegister(CRYPT_L, 0x0);
    HAL_UART_Transmit(COM_UART, postamble, sizeof(postamble), SLEEP_TIME);
}

// REG0 SETS BEGIN
void LoRa_Set_SerialPortRate(enum Serial_Port_Rate spr) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG0) & 0b00011111;
	uint8_t parameter = register_value | (spr << 5);
	LoRa_WriteRegister(REG0, parameter);
}

void LoRa_Set_SerialParityBit(enum Serial_Parity_Bit spb) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG0) & 0b11100111;
	uint8_t parameter = register_value | (spb << 3);
	LoRa_WriteRegister(REG0, parameter);
}

void LoRa_Set_AirDataRate(enum Air_Data_Rate adr) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG0) & 0b11111000;
	uint8_t parameter = register_value | adr;
	LoRa_WriteRegister(REG0, parameter);
}
// REG0 SETS END

// REG1 SETS BEGIN
void LoRa_Set_SubPacketSetting(enum Sub_Packet_Setting sps) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG1) & 0b00111111;
	uint8_t parameter = register_value | (sps << 6);
	LoRa_WriteRegister(REG1, parameter);
}

void LoRa_Set_RSSIAmbientNoise(unsigned char state) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG1) & 0b11011111;
	uint8_t parameter = register_value | (state << 5);
	LoRa_WriteRegister(REG1, parameter);
}

void LoRa_Set_TransmittingPower(enum Transmitting_Power tp) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG1) & 0b11111100;
	uint8_t parameter = register_value | tp;
	LoRa_WriteRegister(REG1, parameter);
}
// REG1 SETS END

// REG3 SETS BEGIN
void LoRa_Set_RSSIEnable(unsigned char state) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG3) & 0b01111111;
	uint8_t parameter = register_value | (state << 7);
	LoRa_WriteRegister(REG1, parameter);
}

void LoRa_Set_TransmissionMode(enum Transmission_Mode tm) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG3) & 0b10111111;
	uint8_t parameter = register_value | (tm << 6);
	LoRa_WriteRegister(REG1, parameter);
}

void LoRa_Set_ReplyEnable(unsigned char state) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG3) & 0b11011111;
	uint8_t parameter = register_value | (state << 5);
	LoRa_WriteRegister(REG1, parameter);
}

void LoRa_Set_LBTEnable(unsigned char state) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG3) & 0b11101111;
	uint8_t parameter = register_value | (state << 4);
	LoRa_WriteRegister(REG1, parameter);
}

void LoRa_Set_WORTransceiverControl(enum WOR_Transceiver_Control state) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG3) & 0b11110111;
	uint8_t parameter = register_value | (state << 3);
	LoRa_WriteRegister(REG1, parameter);
}

void LoRa_Set_WORCycle(enum WOR_Cycle cycle) {
	uint8_t register_value = LoRa_ReadRegisterValue(REG3) & 0b11111000;
	uint8_t parameter = register_value | cycle;
	LoRa_WriteRegister(REG1, parameter);
}
// REG3 SETS END

// External interrupt on AUX change - writes the state to LED output
// Might remove it due to LED being useless here
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == AUX_Pin) {
//		GPIO_PinState AUX_state = HAL_GPIO_ReadPin(GPIOx_AUX, PIN_AUX);
//		HAL_GPIO_WritePin(GPIOx_LED, PIN_LED, AUX_state);
//	}
//}
