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

#define UART_QUEUE_SIZE 1024
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
TIM_HandleTypeDef *TASK_TIM;

// DMA Circular Buffer
static uint8_t rx_buffer[RX_SIZE] = {0};
static uint8_t UART_Queue[UART_QUEUE_SIZE] = {0};
static uint16_t queue_head = 0;
static uint16_t queue_tail = 0;

static uint8_t addh = 0;
static uint8_t addl = 0;
static uint8_t netid = 0;
static uint8_t reg0 = 0;
static uint8_t reg1 = 0;
static uint8_t reg2 = 0;
static uint8_t reg3 = 0;

static uint16_t rx_buffer_head = 0;
static bool receiving_config_data = false;

static uint8_t command_buffer[4];

uint16_t my_id;
uint8_t my_channel;

void LoRa_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, RTC_HandleTypeDef *hrtc, TIM_HandleTypeDef *tim) {

#ifdef DEBUG
	DEBUG_lora_init_timestamp = Get_Timestamp();
#endif

	LoRa_UART = huart1;
	COM_UART = huart2;
	Mesh_RTC = hrtc;
	TASK_TIM = tim;

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
	LoRa_Set_AirDataRate(ADR_2_4K);

	LoRa_Set_SubPacketSetting(SPS_240);
	LoRa_Set_RSSIAmbientNoise(false);
	LoRa_Set_TransmittingPower(TP_22DBM);

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

	HAL_TIM_Base_Start_IT(tim);

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

	printf("[T] \t\t\t\tDEBUG: Time initialize LoRa: %" PRIu32 "ms\n", DEBUG_End_Timing(DEBUG_lora_init_timestamp));
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
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == AUX_Pin) {
		GPIO_PinState AUX_state = HAL_GPIO_ReadPin(GPIOx_AUX, PIN_AUX);
		HAL_GPIO_WritePin(GPIOx_LED, PIN_LED, AUX_state);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	uint16_t new_bytes = Size - rx_buffer_head;

	// Copy new from DMA circular buffer to final buffer
	if (new_bytes > 0) {
		Queue_Push(rx_buffer + rx_buffer_head, new_bytes);
	}
	// This part with calculating indexes gave me a LOT of pain
	rx_buffer_head = Size;

    if (Size == RX_SIZE) {
    	rx_buffer_head = 0;
    }

	if (receiving_config_data) {
		Process_LoRa_Reply();
	}

	LoRa_Start_Receive();
}

void Process_LoRa_Reply(void) {
	while (Queue_Available() > 3) {  // Must be: C1 | STARTING_ADDRESS | LENGTH
		uint8_t *data = Queue_Peek();

		if (data == NULL)
			break;

		uint8_t ptype = data[0];
		uint8_t starting_address = data[1];
		uint8_t data_length = data[2];
		uint8_t total_reply_size = LORA_REPLY_OFFSET + data_length;

		if (ptype != LORA_REPLY_CODE) {
#ifdef DEBUG
			printf("A LoRa reply had \"0x%02X\" as its reply code\n", ptype);
#endif
			Queue_Pop(1);
			continue;
		}

#ifdef DEBUG
		printf("Valid LoRa reply: starting_address=0x%02X, data_length=%d\n",
				starting_address, data_length);
#endif

		if (Queue_Available() < total_reply_size)
			break;

		if (starting_address == PID) {
			if (data_length != 7)
				return;
			for (int i = 0; i < 6; i++)
				printf("0x%02X ", data[i]);
			return;
		}

		uint8_t current_data;

		for (int i = 0; i < data_length; i++) {
			current_data = data[3 + i];
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
		receiving_config_data = false;
		Queue_Pop(total_reply_size);
	}
}

void Queue_Push(uint8_t *data, uint16_t size) {
    if (queue_tail + size <= UART_QUEUE_SIZE) {
        memcpy(&UART_Queue[queue_tail], data, size);
        queue_tail += size;
    } else {
        uint16_t first_part = UART_QUEUE_SIZE - queue_tail;
        memcpy(&UART_Queue[queue_tail], data, first_part);
        memcpy(&UART_Queue[0], data + first_part, size - first_part);
        queue_tail = size - first_part;
    }

    uint16_t available = Queue_Available();
    if (available > UART_QUEUE_SIZE) {
        queue_head = queue_tail;
    }
}

uint16_t Queue_Available(void) {
	if (queue_tail >= queue_head) {
		return queue_tail - queue_head;
	} else {
        return UART_QUEUE_SIZE - queue_head + queue_tail;
	}
}

uint8_t* Queue_Peek(void) {
    if (queue_head == queue_tail) return NULL;
    return &UART_Queue[queue_head];
}

void Queue_Pop(uint16_t size) {
    uint16_t available = Queue_Available();
    size = (size > available) ? available : size;
    queue_head = (queue_head + size) % UART_QUEUE_SIZE;
}

void Queue_Process(void) {
    while (Queue_Available() >= 2) {
        uint8_t *data = Queue_Peek();

        if (data == NULL)
        	break;

        uint8_t ptype = data[0];
        uint16_t payload_length = data[1];
		uint16_t total_packet_size = OPCODE_OFFSET + LENGTH_OFFSET + payload_length;
//		uint16_t total_packet_size += CRC_LENGTH; // Later......
//      uint8_t crc = data[total_packet_size - 1]; // Add to Queue_Validate_Packet

        if (!Queue_Validate_Packet(data, ptype, total_packet_size)) {
			Queue_Pop(1);
			continue;
        }

		if (Queue_Available() < total_packet_size)
			break;

#ifdef DEBUG
		DEBUG_receive_to_send_timestamp = Get_Timestamp();
		DEBUG_receive_to_send_flag = true;
        printf("Valid packet: opcode=0x%02X, payload_len=%d\n", ptype, payload_length);
#endif

		Receive_Packet_Handler(data, total_packet_size, ptype);

#ifdef DEBUG
		DEBUG_receive_to_send_flag = false;
#endif

		Queue_Pop(total_packet_size);

    }
}

bool Queue_Validate_Packet(uint8_t *data, uint8_t ptype, uint16_t total_packet_size) {
	if (ptype > VALID_OPCODES) {
#ifdef DEBUG
		printf("Invalid opcode 0x%02X at offset 0\n", ptype);
#endif
		return FAIL;
	}

	uint16_t expected_size;

	switch (ptype) {
	case RREQ_PACKET:
		expected_size = RREQ_PKT_LEN - LORA_OFFSET;
		if (total_packet_size != expected_size) {
#ifdef DEBUG
		printf("Expected RREQ length = %d, got length = %d\n", expected_size, total_packet_size);
#endif
			return FAIL;
		}
		break;
	case RREP_PACKET:
		expected_size = RREP_PKT_LEN - LORA_OFFSET;
		if (total_packet_size != expected_size) {
#ifdef DEBUG
		printf("Expected RREP length = %d, got length = %d\n", expected_size, total_packet_size);
#endif
			return FAIL;
		}
		break;
	case PING_PACKET:
		expected_size = PING_PKT_LEN - LORA_OFFSET;
		if (total_packet_size != expected_size) {
#ifdef DEBUG
		printf("Expected PING length = %d, got length = %d\n", expected_size, total_packet_size);
#endif
			return FAIL;
		}
		break;
	case ACK_PACKET:
		expected_size = ACK_PKT_LEN - LORA_OFFSET;
		if (total_packet_size != expected_size) {
#ifdef DEBUG
		printf("Expected ACK length = %d, got length = %d\n", expected_size, total_packet_size);
#endif
			return FAIL;
		}
		break;
	default:
		break;
	}

//	uint8_t calculated_crc = Calculate_CRC8(data, total_packet_size - 1);
//	if (crc != calculated_crc) {
//#ifdef DEBUG
//		printf("Calculated CRC = %d, got CRC = %d", calculated_crc, crc);
//#endif
//		return FAIL;
//	}
	return SUCCESS;
}
