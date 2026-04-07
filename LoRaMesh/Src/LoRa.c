#include "main.h"
#include "LoRa.h"
#include "Mesh.h"
#include "Packet_Handlers.h"
#include "Util.h"
#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#define COMMAND_READ_PREFIX 0xC1
#define COMMAND_WRITE_PREFIX 0xC0
#define SLEEP_TIME 150
#define MODE_WORKING MODE_NORMAL

#define RX_FINAL_SIZE 1024
#define RX_SIZE 64

static GPIO_TypeDef *GPIOx_M0;
static GPIO_TypeDef *GPIOx_M1;
static GPIO_TypeDef *GPIOx_AUX;
static GPIO_TypeDef *GPIOx_LED;
static uint16_t PIN_M0;
static uint16_t PIN_M1;
static uint16_t PIN_AUX;
static uint16_t PIN_LED;
UART_HandleTypeDef *LoRa_UART;
UART_HandleTypeDef *COM_UART;
RTC_HandleTypeDef *Mesh_RTC;

// DMA Circular Buffer
static uint8_t rx_buffer[RX_SIZE];

static uint8_t addh = 0;
static uint8_t addl = 0;
static uint8_t netid = 0;
static uint8_t reg0 = 0;
static uint8_t reg1 = 0;
static uint8_t reg2 = 0;
static uint8_t reg3 = 0;

static uint8_t rx_final_buffer[RX_FINAL_SIZE];
static uint16_t indx_1 = 0, indx_2 = 0;
static bool receiving_config_data = false;

static uint8_t command_buffer[4];

uint16_t my_id;
uint8_t my_channel;

void LoRa_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, RTC_HandleTypeDef *hrtc) {

#ifdef DEBUG
	DEBUG_lora_init_timestamp = Get_Timestamp();
#endif

	LoRa_UART = huart1;
	COM_UART = huart2;
	Mesh_RTC = hrtc;

	GPIOx_M0 = M0_GPIO_Port;
	GPIOx_M1 = M1_GPIO_Port;
	GPIOx_AUX = AUX_GPIO_Port;
	GPIOx_LED = LED_GPIO_Port;

	PIN_M0 = M0_Pin;
	PIN_M1 = M1_Pin;
	PIN_LED = LED_Pin;
	PIN_AUX = AUX_Pin;

	LoRa_Start_Receive();
	LoRa_ReadRegister(ADDH, 8); //Reading all registers and parsing the reply as one function call

	// It is recommended to set global LoRa parameters here or in the LoRa_init() from LoRa.c
	// Address should be set up in the main.c, but that is in case that each MCU has a separate project of its own
	// Otherwise just assign an address to one MCU, change it to another in main.c, and continue until you've programmed all the nodes
	// START GLOBAL

#ifdef DEBUG
	LoRa_Set_Encryption(0xCAFE);

	LoRa_Set_NetID(0xBF);

	LoRa_Set_SerialPortRate(SPR_9600);
	LoRa_Set_SerialParityBit(SPB_8N1);
	LoRa_Set_AirDataRate(ADR_62_5K);

	LoRa_Set_SubPacketSetting(SPS_240);
	LoRa_Set_RSSIAmbientNoise(false);
	LoRa_Set_TransmittingPower(TP_10DBM);

	LoRa_Set_Channel(19);

	LoRa_Set_RSSIEnable(false);
	LoRa_Set_TransmissionMode(TM_FIXED_POINT);
	LoRa_Set_ReplyEnable(false);
	LoRa_Set_LBTEnable(false);
	LoRa_Set_WORTransceiverControl(WOR_RECEIVER);
	LoRa_Set_WORCycle(CYCLE_2000MS);
#endif

	// END GLOBAL

	my_channel = reg2;
	my_id = (addh << 8) + addl;

#ifdef DEBUG
	printf("Init LoRa Module:\n");
	printf("ADDH =  0x%02X\n", addh);
	printf("ADDL =  0x%02X\n", addl);
	printf("NETID = 0x%02X\n", netid);
	printf("REG0 =  0x%02X\n", reg0);
	printf("REG1 =  0x%02X\n", reg1);
	printf("REG2 =  0x%02X\n", my_channel);
	printf("REG3 =  0x%02X\n", reg3);

	printf("ID: %d, Channel: %d, NETID: %d\n", my_id, my_channel, netid);

	printf("Init done.\n");

	printf("\t\t\t\tDEBUG: Time initialize LoRa: %" PRIu32 "ms\n", DEBUG_End_Timing(DEBUG_lora_init_timestamp));
#endif
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
void LoRa_ReadRegister(uint8_t starting_address, uint8_t length) {
	LoRa_ModeSelect(MODE_CONFIG);
	receiving_config_data = true;
	uint8_t command_index = 0;
	command_buffer[command_index++] = COMMAND_READ_PREFIX;
	command_buffer[command_index++] = starting_address;
	command_buffer[command_index++] = length;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, SLEEP_TIME);
	HAL_Delay(SLEEP_TIME);
	LoRa_ModeSelect(MODE_WORKING);
}

void LoRa_WriteRegister(uint8_t address, uint8_t length, uint8_t parameter) {
	LoRa_ModeSelect(MODE_CONFIG);
	receiving_config_data = true;
	uint8_t command_index = 0;
	command_buffer[command_index++] = COMMAND_WRITE_PREFIX;
	command_buffer[command_index++] = address;
	command_buffer[command_index++] = length;
	command_buffer[command_index++] = parameter;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, SLEEP_TIME);
	HAL_Delay(SLEEP_TIME);
	LoRa_ModeSelect(MODE_WORKING);
}

// Register PID has 7 bytes
void LoRa_ReadProductInfo(void) {
	printf("Product info:\n");
	LoRa_ReadRegister(PID, 7);
}

// Returns to factory default parameters
// Frequency		- 869.125MHz
// Address			- 0x0000
// Channel			- 0x13
// Air data rate	- 2.4kbps
// Baud rate		- 9600
// Parity format	- 8N1
// Power			- 22dbm
void LoRa_ResetFirmware(void) {
	printf("Resetting firmware...\n");
	LoRa_WriteRegister(ADDH, 1, 0x00);
	LoRa_WriteRegister(ADDL, 1, 0x00);
	LoRa_WriteRegister(NETID, 1, 0x00);
	LoRa_WriteRegister(REG0, 1, 0x62);
	LoRa_WriteRegister(REG1, 1, 0x00);
	LoRa_WriteRegister(REG2, 1, 0x13);
	LoRa_WriteRegister(REG3, 1, 0x3);
	LoRa_WriteRegister(CRYPT_H, 1, 0x00);
	LoRa_WriteRegister(CRYPT_L, 1, 0x00);
	printf("Reset done!\n");
}

// ADDR NETID SETS BEGIN
void LoRa_Set_Address(uint16_t addr) { // Can be set as 0xFFFF for broadcast monitoring
	LoRa_WriteRegister(ADDH, 1, addr >> 8);
	LoRa_WriteRegister(ADDL, 1, addr & 0xFF);
}

void LoRa_Set_NetID(uint8_t netid) {
	LoRa_WriteRegister(NETID, 1, netid);
}
// ADDR NETID SETS BEGIN
void LoRa_Set_SerialPortRate(enum Serial_Port_Rate spr) {
	uint8_t register_value = reg0 & 0b00011111;
	uint8_t parameter = register_value | (spr << 5);
	LoRa_WriteRegister(REG0, 1, parameter);
}

void LoRa_Set_SerialParityBit(enum Serial_Parity_Bit spb) {
	uint8_t register_value = reg0 & 0b11100111;
	uint8_t parameter = register_value | (spb << 3);
	LoRa_WriteRegister(REG0, 1, parameter);
}

void LoRa_Set_AirDataRate(enum Air_Data_Rate adr) {
	uint8_t register_value = reg0 & 0b11111000;
	uint8_t parameter = register_value | adr;
	LoRa_WriteRegister(REG0, 1, parameter);
}
// REG0 SETS END

// REG1 SETS BEGIN
void LoRa_Set_SubPacketSetting(enum Sub_Packet_Setting sps) {
	uint8_t register_value = reg1 & 0b00111111;
	uint8_t parameter = register_value | (sps << 6);
	LoRa_WriteRegister(REG1, 1, parameter);
}

void LoRa_Set_RSSIAmbientNoise(bool state) {
	uint8_t register_value = reg1 & 0b11011111;
	uint8_t parameter = register_value | (state << 5);
	LoRa_WriteRegister(REG1, 1, parameter);
}

void LoRa_Set_TransmittingPower(enum Transmitting_Power tp) {
	uint8_t register_value = reg1 & 0b11111100;
	uint8_t parameter = register_value | tp;
	LoRa_WriteRegister(REG1, 1, parameter);
}
// REG1 SETS END

// REG2 SETS BEGIN
void LoRa_Set_Channel(uint8_t channel) {
	LoRa_WriteRegister(REG2, 1, channel);
}
// REG2 SETS END

// REG3 SETS BEGIN
void LoRa_Set_RSSIEnable(bool state) {
	uint8_t register_value = reg3 & 0b01111111;
	uint8_t parameter = register_value | (state << 7);
	LoRa_WriteRegister(REG3, 1, parameter);
}

void LoRa_Set_TransmissionMode(enum Transmission_Mode tm) {
	uint8_t register_value = reg3 & 0b10111111;
	uint8_t parameter = register_value | (tm << 6);
	LoRa_WriteRegister(REG3, 1, parameter);
}

void LoRa_Set_ReplyEnable(bool state) {
	uint8_t register_value = reg3 & 0b11011111;
	uint8_t parameter = register_value | (state << 5);
	LoRa_WriteRegister(REG3, 1, parameter);
}

void LoRa_Set_LBTEnable(bool state) {
	uint8_t register_value = reg3 & 0b11101111;
	uint8_t parameter = register_value | (state << 4);
	LoRa_WriteRegister(REG3, 1, parameter);
}

void LoRa_Set_WORTransceiverControl(enum WOR_Transceiver_Control state) {
	uint8_t register_value = reg3 & 0b11110111;
	uint8_t parameter = register_value | (state << 3);
	LoRa_WriteRegister(REG3, 1, parameter);
}

void LoRa_Set_WORCycle(enum WOR_Cycle cycle) {
	uint8_t register_value = reg3 & 0b11111000;
	uint8_t parameter = register_value | cycle;
	LoRa_WriteRegister(REG3, 1, parameter);
}
// REG3 SETS END

void LoRa_Set_Encryption(uint16_t key) {
	LoRa_WriteRegister(CRYPT_H, 1, key >> 8);
	LoRa_WriteRegister(CRYPT_L, 1, key & 0xFF);
}

void LoRa_Start_Receive(void) {
	HAL_UARTEx_ReceiveToIdle_DMA(LoRa_UART, rx_buffer, RX_SIZE);
}

void LoRa_SendData(uint8_t *buffer, uint8_t buffer_size) {
	HAL_UART_Transmit(LoRa_UART, buffer, buffer_size, SLEEP_TIME);
#ifdef DEBUG
	if (DEBUG_receive_to_send_flag)
		printf("\t\t\t\tDEBUG: Time between receiving a packet and sending a reply: %" PRIu32 "ms\n", DEBUG_End_Timing(DEBUG_receive_to_send_timestamp));
	DEBUG_receive_to_send_flag = false;
#endif
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == AUX_Pin) {
		GPIO_PinState AUX_state = HAL_GPIO_ReadPin(GPIOx_AUX, PIN_AUX);
		HAL_GPIO_WritePin(GPIOx_LED, PIN_LED, AUX_state);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	// Copy new data from DMA circular buffer to final buffer
	memcpy(rx_final_buffer + indx_2, rx_buffer + indx_1, Size);

	// This part with calculating indexes gave me a LOT of pain
	indx_2 += Size - indx_1;
	if (Size == RX_SIZE) {
#ifdef DEBUG
	printf("Received POSSIBLY NOT-FULL %d bytes | Packet type: 0x%02X | Payload: %d\n", indx_2, rx_final_buffer[0], rx_final_buffer[1]);
#endif
		indx_1 = 0;
	} else {
#ifdef DEBUG
	printf("Received %d bytes | Packet type: 0x%02X | Payload: %d\n", indx_2, rx_final_buffer[0], rx_final_buffer[1]);
#endif
		indx_1 = Size;
	}

//    __disable_irq();

	if (receiving_config_data) {
		Process_LoRa_Reply();
	} else {
		Process_Packet();
	}

//    __enable_irq();

#ifdef DEBUG
	printf("Done processing packets\n");
#endif
	LoRa_Start_Receive();
}

void Process_Packet(void) {
	while (indx_2 >= 2) {
		uint8_t ptype = rx_final_buffer[0];
		uint8_t payload_length = rx_final_buffer[1];
		uint16_t total_packet_size = OPCODE_OFFSET + LENGTH_OFFSET + payload_length;

		if (indx_2 >= total_packet_size) {
#ifdef DEBUG
			DEBUG_receive_to_send_timestamp = Get_Timestamp();
			DEBUG_receive_to_send_flag = true;
#endif
			Receive_Packet_Handler(rx_final_buffer, total_packet_size, ptype);

#ifdef DEBUG
			DEBUG_receive_to_send_flag = false;
#endif

			if (indx_2 > total_packet_size) {
				memmove(rx_final_buffer, rx_final_buffer + total_packet_size,
						indx_2 - total_packet_size);
				indx_2 -= total_packet_size;
			} else {
				indx_2 = 0;
			}
		} else {
			break;
		}
	}
}

void Process_LoRa_Reply(void) {
	while (indx_2 >= 3) {  // Must be: C1 | STARTING_ADDRESS | LENGTH
		if (rx_final_buffer[0] != 0xC1) {
#ifdef DEBUG
			printf("A LoRa reply had \"0x%02X\" as its reply code\n", rx_final_buffer[0]);
#endif
			memmove(rx_final_buffer, rx_final_buffer + 1, indx_2 - 1);
			indx_2--;
			continue;
		}

		uint8_t starting_address = rx_final_buffer[1];
		uint8_t data_length = rx_final_buffer[2];
		uint16_t total_reply_size = 3 + data_length;
		if (indx_2 >= total_reply_size) {
			LoRa_Reply_Handler(rx_final_buffer, starting_address, data_length);

			if (indx_2 > total_reply_size) {
				memmove(rx_final_buffer, rx_final_buffer + total_reply_size, indx_2 - total_reply_size);
				indx_2 -= total_reply_size;
			} else {
				indx_2 = 0;
			}
		} else {
			break;
		}
	}
}

void LoRa_Reply_Handler(uint8_t rx_final_buffer[], uint8_t starting_address, uint8_t data_length) {
	receiving_config_data = false;

	if (starting_address == PID) {
		if (data_length != 7)
			return;
		for (int i = 0; i < 6; i++)
			printf("0x%02X ", rx_final_buffer[i]);
		return;
	}

	uint8_t current_data;

	for (int i = 0; i < data_length; i++) {
		current_data = rx_final_buffer[3 + i];
		switch (starting_address + i) {
		case ADDH:
			addh = current_data;
			break;
		case ADDL:
			addl = current_data;
			break;
		case NETID:
			netid = current_data;
			break;
		case REG0:
			reg0 = current_data;
			break;
		case REG1:
			reg1 = current_data;
			break;
		case REG2:
			reg2 = current_data;
			break;
		case REG3:
			reg3 = current_data;
			break;
		default:
			break;
		}
	}
}
