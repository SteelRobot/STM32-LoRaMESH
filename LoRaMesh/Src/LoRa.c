#include "main.h"
#include "LoRa.h"
#include <string.h>
#include <stdio.h>

#define COMMAND_READ_PREFIX 0xC1
#define COMMAND_WRITE_PREFIX 0xC0
#define SLEEP_TIME 1000

static GPIO_TypeDef *GPIOx_M0;
static GPIO_TypeDef *GPIOx_M1;
static GPIO_TypeDef *GPIOx_AUX;
static GPIO_TypeDef *GPIOx_LED;
static uint16_t PIN_M0;
static uint16_t PIN_M1;
static uint16_t PIN_AUX;
static uint16_t PIN_LED;
static UART_HandleTypeDef *LoRa_UART;
static UART_HandleTypeDef *COM_UART;

static uint8_t rx_data_to_send[512] = { 0 };
static uint8_t rx_buffer[512] = { 0 };
static uint8_t rx_byte;
static uint8_t rx_index = 0;

static uint8_t tx_data_to_send[512] = { 0 };
static uint8_t tx_buffer[512] = { 0 };
static uint8_t tx_byte;
static uint8_t tx_index = 0;

static uint8_t command_buffer[4];
static uint8_t response_buffer[4];

uint8_t current_node_address;
uint8_t current_node_channel;

uint8_t target_address;

void LoRa_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2) {

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

	printf("Init LoRa Module:\n");
	LoRa_ModeSelect(MODE_CONFIG);
	uint8_t addh = LoRa_ReadRegister(ADDH);
	uint8_t addl = LoRa_ReadRegister(ADDL);
	printf("ADDH =  0x%04X\n", addh);
	printf("ADDL =  0x%04X\n", addl);
	current_node_address = (addh << 8) + addl;
	printf("Address = %d\n", current_node_address);
	printf("NETID = 0x%04X\n", LoRa_ReadRegister(NETID));
	printf("REG0 =  0x%04X\n", LoRa_ReadRegister(REG0));
	printf("REG1 =  0x%04X\n", LoRa_ReadRegister(REG1));
	current_node_channel = LoRa_ReadRegister(REG2);
	printf("REG2 =  0x%04X\n", current_node_channel);
	printf("REG3 =  0x%04X\n", LoRa_ReadRegister(REG3));

	HAL_UART_Receive_IT(LoRa_UART, &rx_byte, 1);
	HAL_UART_Receive_IT(COM_UART, &tx_byte, 1);
	LoRa_ModeSelect(MODE_NORMAL);
	printf("Init done.\n");
}

// 00 - Normal mode
// 01 - WOR mode
// 10 - Configuration mode
// 11 - Deep sleep mode
void LoRa_ModeSelect(enum Mode mode) {
	GPIO_PinState AUX_state = HAL_GPIO_ReadPin(GPIOx_AUX, PIN_AUX);
	if (AUX_state == GPIO_PIN_SET) {
		HAL_Delay(2); // As per the user manual, switching only after 2ms when the output of AUX is high.
	}

	/*
	 What if AUX_state is 0?
	 1. If module is sending data to MCU after receiving wireless data - falling edge occurs to wake up MCU and then during data transfer

	 2. During wireless transmitting, if AUX = 1 user can input data less than 1000 bytes into internal buffer.
	 When AUX = 0 internal buffer didn't finish writing to RFIC completely.

	 3. When power-on or instruction reset happens, OR module switches from deep sleep mode (Mode 3), self-check happens.
	 During self-check AUX = 0. After the rising edge it's ready to work
	 */
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

// To return just the register's value
uint8_t LoRa_ReadRegister(uint8_t address) {
	uint8_t command_index = 0;
	command_buffer[command_index++] = COMMAND_READ_PREFIX;
	command_buffer[command_index++] = address;
	command_buffer[command_index++] = 0x1;
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	LoRa_UART->RxState = HAL_UART_STATE_READY;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, SLEEP_TIME);
	HAL_UART_Receive(LoRa_UART, response_buffer, sizeof(response_buffer),
			SLEEP_TIME);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	return response_buffer[3];
}

void LoRa_WriteRegister(uint8_t address, uint8_t parameter) {
	uint8_t command_index = 0;
	command_buffer[command_index++] = COMMAND_WRITE_PREFIX;
	command_buffer[command_index++] = address;
	command_buffer[command_index++] = 0x1;
	command_buffer[command_index++] = parameter;
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	LoRa_UART->RxState = HAL_UART_STATE_READY;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, SLEEP_TIME);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void LoRa_SendData(uint8_t *buffer, uint8_t buffer_size) {
	HAL_UART_Transmit(LoRa_UART, buffer, buffer_size, SLEEP_TIME);
}

// Register PID has 7 bytes
void LoRa_ReadProductInfo(void) {
	uint8_t command_index = 0;
	uint8_t product_response_buffer[10];
	uint8_t com_product_response_buffer[30];
	command_buffer[command_index++] = COMMAND_READ_PREFIX;
	command_buffer[command_index++] = PID;
	command_buffer[command_index++] = 0x7;
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	LoRa_UART->RxState = HAL_UART_STATE_READY;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, 1000);
	HAL_UART_Receive(LoRa_UART, product_response_buffer,
			sizeof(product_response_buffer), SLEEP_TIME);
	printf("Product info:\n");
	printf("%s\n", com_product_response_buffer);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

// Returns to factory default parameters
// Frequency		- 868.125MHz
// Address			- 0x0000
// Channel			- 0x12
// Air data rate	- 2.4kbps
// Baud rate		- 9600
// Parity format	- 8N1
// Power			- 22dbm
void LoRa_ResetFirmware(void) {
	printf("Resetting firmware...\n");
	LoRa_WriteRegister(ADDH, 0x0);
	LoRa_WriteRegister(ADDL, 0x0);
	LoRa_WriteRegister(NETID, 0x0);
	LoRa_WriteRegister(REG0, 0x62);
	LoRa_WriteRegister(REG1, 0x00);
	LoRa_WriteRegister(REG2, 0x12);
	LoRa_WriteRegister(REG3, 0x3);
	LoRa_WriteRegister(CRYPT_H, 0x0);
	LoRa_WriteRegister(CRYPT_L, 0x0);
	printf("Reset done!\n");
}

// ADDR NETID SETS BEGIN
void LoRa_Set_Address(uint16_t addr) { // Can be set as 0xFFFF for broadcast monitoring
	LoRa_WriteRegister(ADDH, addr >> 8);
	LoRa_WriteRegister(ADDL, addr & 0xFF);
}

void LoRa_Set_NetID(uint8_t netid) {
	LoRa_WriteRegister(NETID, netid);
}
// ADDR NETID SETS BEGIN
void LoRa_Set_SerialPortRate(enum Serial_Port_Rate spr) {
	uint8_t register_value = LoRa_ReadRegister(REG0) & 0b00011111;
	uint8_t parameter = register_value | (spr << 5);
	LoRa_WriteRegister(REG0, parameter);
}

void LoRa_Set_SerialParityBit(enum Serial_Parity_Bit spb) {
	uint8_t register_value = LoRa_ReadRegister(REG0) & 0b11100111;
	uint8_t parameter = register_value | (spb << 3);
	LoRa_WriteRegister(REG0, parameter);
}

void LoRa_Set_AirDataRate(enum Air_Data_Rate adr) {
	uint8_t register_value = LoRa_ReadRegister(REG0) & 0b11111000;
	uint8_t parameter = register_value | adr;
	LoRa_WriteRegister(REG0, parameter);
}
// REG0 SETS END

// REG1 SETS BEGIN
void LoRa_Set_SubPacketSetting(enum Sub_Packet_Setting sps) {
	uint8_t register_value = LoRa_ReadRegister(REG1) & 0b00111111;
	uint8_t parameter = register_value | (sps << 6);
	LoRa_WriteRegister(REG1, parameter);
}

void LoRa_Set_RSSIAmbientNoise(bool state) {
	uint8_t register_value = LoRa_ReadRegister(REG1) & 0b11011111;
	uint8_t parameter = register_value | (state << 5);
	LoRa_WriteRegister(REG1, parameter);
}

void LoRa_Set_TransmittingPower(enum Transmitting_Power tp) {
	uint8_t register_value = LoRa_ReadRegister(REG1) & 0b11111100;
	uint8_t parameter = register_value | tp;
	LoRa_WriteRegister(REG1, parameter);
}
// REG1 SETS END

// REG2 SETS BEGIN
void LoRa_Set_Channel(uint8_t channel) {
	LoRa_WriteRegister(REG2, channel);
}
// REG2 SETS END

// REG3 SETS BEGIN
void LoRa_Set_RSSIEnable(bool state) {
	uint8_t register_value = LoRa_ReadRegister(REG3) & 0b01111111;
	uint8_t parameter = register_value | (state << 7);
	LoRa_WriteRegister(REG3, parameter);
}

void LoRa_Set_TransmissionMode(enum Transmission_Mode tm) {
	uint8_t register_value = LoRa_ReadRegister(REG3) & 0b10111111;
	uint8_t parameter = register_value | (tm << 6);
	LoRa_WriteRegister(REG3, parameter);
}

void LoRa_Set_ReplyEnable(bool state) {
	uint8_t register_value = LoRa_ReadRegister(REG3) & 0b11011111;
	uint8_t parameter = register_value | (state << 5);
	LoRa_WriteRegister(REG3, parameter);
}

void LoRa_Set_LBTEnable(bool state) {
	uint8_t register_value = LoRa_ReadRegister(REG3) & 0b11101111;
	uint8_t parameter = register_value | (state << 4);
	LoRa_WriteRegister(REG3, parameter);
}

void LoRa_Set_WORTransceiverControl(enum WOR_Transceiver_Control state) {
	uint8_t register_value = LoRa_ReadRegister(REG3) & 0b11110111;
	uint8_t parameter = register_value | (state << 3);
	LoRa_WriteRegister(REG3, parameter);
}

void LoRa_Set_WORCycle(enum WOR_Cycle cycle) {
	uint8_t register_value = LoRa_ReadRegister(REG3) & 0b11111000;
	uint8_t parameter = register_value | cycle;
	LoRa_WriteRegister(REG3, parameter);
}
// REG3 SETS END

void LoRa_Set_Encryption(uint16_t key) {
	LoRa_WriteRegister(CRYPT_H, key >> 8);
	LoRa_WriteRegister(CRYPT_L, key & 0xFF);
}

// External interrupt on AUX change - writes the state to LED output
// Might remove it due to LED being useless here
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == AUX_Pin) {
		GPIO_PinState AUX_state = HAL_GPIO_ReadPin(GPIOx_AUX, PIN_AUX);
		HAL_GPIO_WritePin(GPIOx_LED, PIN_LED, AUX_state);
	}
}

// UART Interrupt to receive data from LoRa module, and to send your own data from keyboard
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == LoRa_UART->Instance) {
		rx_buffer[rx_index] = rx_byte;
		if (++rx_index >= sizeof(rx_buffer))
			rx_index = 0;

		if (rx_byte == '\n' || rx_byte == '\r') {
			memset(rx_data_to_send, 0, sizeof(rx_data_to_send));
			memcpy(rx_data_to_send, rx_buffer, rx_index);
			memset(rx_buffer, 0, sizeof(rx_buffer));
			printf("%s\n", rx_data_to_send);
			rx_index = 0;
		}
		HAL_UART_Receive_IT(LoRa_UART, &rx_byte, 1);
	}

	else if (huart->Instance == COM_UART->Instance) {

		tx_buffer[tx_index] = tx_byte;
		if (++tx_index >= sizeof(tx_buffer))
			tx_index = 0;

		if (tx_index > 2)
			target_address = tx_buffer[0];

		if (tx_byte == '\n' || tx_byte == '\r') {
			memset(tx_data_to_send, 0, sizeof(tx_data_to_send));
			memcpy(tx_data_to_send, tx_buffer, tx_index);
			memset(tx_buffer, 0, sizeof(tx_buffer));
			Mesh_SendData(target_address, tx_data_to_send, tx_index);
			tx_index = 0;
		}
		HAL_UART_Receive_IT(COM_UART, &tx_byte, 1);
	}
}
