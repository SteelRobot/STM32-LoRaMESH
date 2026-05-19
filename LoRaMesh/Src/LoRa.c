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
#define SLEEP_TIME 200
#define MODE_WORKING MODE_NORMAL
#define UART_QUEUE_SIZE 1024
#define RX_SIZE 64

static const RegisterMap E22_registers = {
    .addh = 0x0, .addl = 0x1, .netid = 0x2, .reg0 = 0x3,
    .reg1 = 0x4, .reg2 = 0x5, .reg3 = 0x6,
	.crypt_h = 0x7, .crypt_l = 0x8, .pid = 0x80
};

static const ModeConfig E22_modes = {
    .normal = {GPIO_PIN_RESET, GPIO_PIN_RESET},
    .wor = {GPIO_PIN_SET, GPIO_PIN_RESET},
    .config = {GPIO_PIN_RESET, GPIO_PIN_SET},
    .sleep = {GPIO_PIN_SET, GPIO_PIN_SET},
    .wor_transmitting = {GPIO_PIN_SET, GPIO_PIN_RESET},  // Unused - defaults to WOR
    .wor_receiving = {GPIO_PIN_SET, GPIO_PIN_RESET}  	 // Unused - defaults to WOR
};

static const RegisterMap E220_registers = {
    .addh = 0x0, .addl = 0x1, .reg0 = 0x2,
    .reg1 = 0x3, .reg2 = 0x4, .reg3 = 0x5,
    .crypt_h = 0x6, .crypt_l = 0x7,
    .netid = 0xFF, .pid = 0xFF // Unavailable in this module
};

static const ModeConfig E220_modes = {
    .normal = {GPIO_PIN_RESET, GPIO_PIN_RESET},
    .wor_transmitting = {GPIO_PIN_SET, GPIO_PIN_RESET},
    .wor_receiving = {GPIO_PIN_RESET, GPIO_PIN_SET},
    .sleep = {GPIO_PIN_SET, GPIO_PIN_SET},
    .wor = {GPIO_PIN_SET, GPIO_PIN_RESET},	// Unused - defaults to WOR Transmitting
    .config = {GPIO_PIN_SET, GPIO_PIN_SET}	// Same as sleep
};

static ModuleDriver current_driver = {0};

static GPIO_TypeDef *GPIOx_M0, *GPIOx_M1, *GPIOx_AUX, *GPIOx_LED;
static uint16_t PIN_M0, PIN_M1, PIN_AUX, PIN_LED;
UART_HandleTypeDef *LoRa_UART, *COM_UART;
RTC_HandleTypeDef *Mesh_RTC;
TIM_HandleTypeDef *TASK_TIM;

// DMA Circular Buffer
static uint8_t rx_buffer[RX_SIZE] = {0};
static uint8_t UART_Queue[UART_QUEUE_SIZE] = {0};
static uint16_t queue_head = 0, queue_tail = 0;
static uint16_t rx_buffer_head = 0;
static bool receiving_config_data = false;
static uint8_t command_buffer[4];

static RegisterCache cache = {0};

uint16_t my_id;
uint8_t my_channel;
enum Module_Type my_module;

void LoRa_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, RTC_HandleTypeDef *hrtc, TIM_HandleTypeDef *tim, enum Module_Type module) {

#ifdef DEBUG
	DEBUG_lora_init_timestamp = Get_Timestamp();
#endif

	LoRa_UART = huart1;
	COM_UART = huart2;
	Mesh_RTC = hrtc;
	TASK_TIM = tim;

	my_module = module;
	LoRa_InitDriver(module);

	GPIOx_M0 = M0_GPIO_Port;
	GPIOx_M1 = M1_GPIO_Port;
	GPIOx_AUX = AUX_GPIO_Port;
	GPIOx_LED = LED_GPIO_Port;

	PIN_M0 = M0_Pin;
	PIN_M1 = M1_Pin;
	PIN_LED = LED_Pin;
	PIN_AUX = AUX_Pin;

	LoRa_Start_Receive();
	LoRa_ReadRegister(current_driver.registers->addh, 8); //Reading all registers and parsing the reply as one function call

	// It is recommended to set global LoRa parameters here or in the LoRa_init() from LoRa.c
	// Address should be set up in the main.c, but that is in case that each MCU has a separate project of its own
	// Otherwise just assign an address to one MCU, change it to another in main.c, and continue until you've programmed all the nodes
	// START GLOBAL

#ifdef DEBUG
	LoRa_Set_Encryption(0xCAFE);

	LoRa_Set_NetID(0x00);

	LoRa_Set_SerialPortRate(SPR_9600);
	LoRa_Set_SerialParityBit(SPB_8N1);
	LoRa_Set_AirDataRate(ADR_9_6K);

	LoRa_Set_SubPacketSetting(SPS_128);
	LoRa_Set_RSSIAmbientNoise(false);
	LoRa_Set_TransmittingPower(TP_22DBM);

	LoRa_Set_Channel(19); // 19 for Russia

	LoRa_Set_RSSIEnable(false);
	LoRa_Set_TransmissionMode(TM_FIXED_POINT);
	LoRa_Set_ReplyEnable(false);
	LoRa_Set_LBTEnable(false);
	LoRa_Set_WORTransceiverControl(WOR_RECEIVER);
	LoRa_Set_WORCycle(CYCLE_2000MS);
#endif

	// END GLOBAL

	my_channel = cache.reg2;
	my_id = (cache.addh << 8) + cache.addl;

	HAL_TIM_Base_Start_IT(tim);

#ifdef DEBUG
	printf("Init LoRa Module:\n");
	printf("ADDH =  0x%02X\n", cache.addh);
	printf("ADDL =  0x%02X\n", cache.addl);
	if (current_driver.registers->netid != 0xFF)
		printf("NETID = 0x%02X\n", cache.netid);
	printf("REG0 =  0x%02X\n", cache.reg0);
	printf("REG1 =  0x%02X\n", cache.reg1);
	printf("REG2 =  0x%02X\n", cache.reg2);
	printf("REG3 =  0x%02X\n", cache.reg3);

	printf("ID: %d, Channel: %d\n", my_id, my_channel);

	printf("Init done.\n");

	printf("[T] \t\t\t\tDEBUG: Time initialize LoRa: %" PRIu32 "ms\n", DEBUG_End_Timing(DEBUG_lora_init_timestamp));
#endif
}

void LoRa_InitDriver(enum Module_Type module) {
	if (module == E22_900T22D) {
		current_driver.type = E22_900T22D;
		current_driver.registers = &E22_registers;
		current_driver.modes = &E22_modes;
	} else if (module == E220_900T22D) {
		current_driver.type = E220_900T22D;
		current_driver.registers = &E220_registers;
		current_driver.modes = &E220_modes;
	}
}

void LoRa_ModeSelect(enum Mode mode) {
	GPIO_PinState AUX_state = HAL_GPIO_ReadPin(GPIOx_AUX, PIN_AUX);
	if (AUX_state == GPIO_PIN_SET) {
		HAL_Delay(2); // As per the user manual, switching only after 2ms when the output of AUX is high.
	}

	const ModePin *pins = NULL;

	switch (mode) {
	    case MODE_NORMAL:
	        pins = &current_driver.modes->normal;
	        break;
	    case MODE_WOR:
	    	pins = &current_driver.modes->wor;
	        break;
	    case MODE_CONFIG:
	    	pins = &current_driver.modes->config;
	        break;
	    case MODE_SLEEP:
	    	pins = &current_driver.modes->sleep;
	        break;
	    case MODE_WOR_TRANSMITTING:
	   	    pins = &current_driver.modes->wor_transmitting;
	   	    break;
	    case MODE_WOR_RECEIVING:
	   	    pins = &current_driver.modes->wor_receiving;
	   	    break;
	}

    if (pins) {
    	HAL_GPIO_WritePin(GPIOx_M0, PIN_M0, pins->m0_state);
        HAL_GPIO_WritePin(GPIOx_M1, PIN_M1, pins->m1_state);
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

	if (current_driver.registers->pid == 0xFF) {
		printf("Product info not available on this module\n");
		return;
	}

	LoRa_ReadRegister(current_driver.registers->pid, 7);
}

// Returns to factory default parameters
// Frequency		- 869.125MHz
// Address			- 0x0000
// Channel			- 0x13 = 19 for Russia
// Air data rate	- 2.4kbps
// Baud rate		- 9600
// Parity format	- 8N1
// Power			- 22dbm
void LoRa_ResetFirmware(void) {
	printf("Resetting firmware...\n");

	LoRa_WriteRegister(current_driver.registers->addh, 1, 0x00);
	LoRa_WriteRegister(current_driver.registers->addl, 1, 0x00);
	LoRa_WriteRegister(current_driver.registers->reg0, 1, 0x62);
	LoRa_WriteRegister(current_driver.registers->reg1, 1, 0x00);
	LoRa_WriteRegister(current_driver.registers->reg2, 1, 0x13);
	LoRa_WriteRegister(current_driver.registers->reg3, 1, 0x43);
	LoRa_WriteRegister(current_driver.registers->crypt_h, 1, 0xBE);
	LoRa_WriteRegister(current_driver.registers->crypt_l, 1, 0xEF);

	// Only reset NETID if the module supports it
	if (current_driver.registers->netid != 0xFF) {
		LoRa_WriteRegister(current_driver.registers->netid, 1, 0x00);
	}

	printf("Reset done!\n");
}

// ADDR NETID SETS BEGIN
void LoRa_Set_Address(uint16_t addr) { // Can be set as 0xFFFF for broadcast monitoring
	LoRa_WriteRegister(current_driver.registers->addh, 1, addr >> 8);
	LoRa_WriteRegister(current_driver.registers->addl, 1, addr & 0xFF);
}

void LoRa_Set_NetID(uint8_t netid) {
	if (current_driver.registers->netid != 0xFF) {
		LoRa_WriteRegister(current_driver.registers->netid, 1, netid);
	}
}

// REG0 SETS BEGIN
void LoRa_Set_SerialPortRate(enum Serial_Port_Rate spr) {
	uint8_t register_value = cache.reg0 & 0b00011111;
	uint8_t parameter = register_value | (spr << 5);
	LoRa_WriteRegister(current_driver.registers->reg0, 1, parameter);
}

void LoRa_Set_SerialParityBit(enum Serial_Parity_Bit spb) {
	uint8_t register_value = cache.reg0 & 0b11100111;
	uint8_t parameter = register_value | (spb << 3);
	LoRa_WriteRegister(current_driver.registers->reg0, 1, parameter);
}

void LoRa_Set_AirDataRate(enum Air_Data_Rate adr) {
	uint8_t register_value = cache.reg0 & 0b11111000;
	uint8_t parameter = register_value | adr;
	LoRa_WriteRegister(current_driver.registers->reg0, 1, parameter);
}
// REG0 SETS END

// REG1 SETS BEGIN
void LoRa_Set_SubPacketSetting(enum Sub_Packet_Setting sps) {
	uint8_t register_value = cache.reg1 & 0b00111111;
	uint8_t parameter = register_value | (sps << 6);
	LoRa_WriteRegister(current_driver.registers->reg1, 1, parameter);
}

void LoRa_Set_RSSIAmbientNoise(bool state) {
	uint8_t register_value = cache.reg1 & 0b11011111;
	uint8_t parameter = register_value | (state << 5);
	LoRa_WriteRegister(current_driver.registers->reg1, 1, parameter);
}

void LoRa_Set_TransmittingPower(enum Transmitting_Power tp) {
	uint8_t register_value = cache.reg1 & 0b11111100;
	uint8_t parameter = register_value | tp;
	LoRa_WriteRegister(current_driver.registers->reg1, 1, parameter);
}
// REG1 SETS END

// REG2 SETS BEGIN
void LoRa_Set_Channel(uint8_t channel) {
	LoRa_WriteRegister(current_driver.registers->reg2, 1, channel);
}
// REG2 SETS END

// REG3 SETS BEGIN
void LoRa_Set_RSSIEnable(bool state) {
	uint8_t register_value = cache.reg3 & 0b01111111;
	uint8_t parameter = register_value | (state << 7);
	LoRa_WriteRegister(current_driver.registers->reg3, 1, parameter);
}

void LoRa_Set_TransmissionMode(enum Transmission_Mode tm) {
	uint8_t register_value = cache.reg3 & 0b10111111;
	uint8_t parameter = register_value | (tm << 6);
	LoRa_WriteRegister(current_driver.registers->reg3, 1, parameter);
}

void LoRa_Set_ReplyEnable(bool state) {
	if (my_module != E220_900T22D) {
		uint8_t register_value = cache.reg3 & 0b11011111;
		uint8_t parameter = register_value | (state << 5);
		LoRa_WriteRegister(current_driver.registers->reg3, 1, parameter);
	} else {
		printf("Reply function is not available in this LoRa module\n");
	}
}

void LoRa_Set_LBTEnable(bool state) {
	uint8_t register_value = cache.reg3 & 0b11101111;
	uint8_t parameter = register_value | (state << 4);
	LoRa_WriteRegister(current_driver.registers->reg3, 1, parameter);
}

void LoRa_Set_WORTransceiverControl(enum WOR_Transceiver_Control state) {
	if (my_module != E220_900T22D) {
		uint8_t register_value = cache.reg3 & 0b11110111;
		uint8_t parameter = register_value | (state << 3);
		LoRa_WriteRegister(current_driver.registers->reg3, 1, parameter);
	} else {
		printf("WOR control is not available in this LoRa module. It may be possible to use mode switching for that\n");
	}
}

void LoRa_Set_WORCycle(enum WOR_Cycle cycle) {
	uint8_t register_value = cache.reg3 & 0b11111000;
	uint8_t parameter = register_value | cycle;
	LoRa_WriteRegister(current_driver.registers->reg3, 1, parameter);
}
// REG3 SETS END

void LoRa_Set_Encryption(uint16_t key) {
	LoRa_WriteRegister(current_driver.registers->crypt_h, 1, key >> 8);
	LoRa_WriteRegister(current_driver.registers->crypt_h, 1, key & 0xFF);
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
		RX_Queue_Push(rx_buffer + rx_buffer_head, new_bytes);
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
	while (RX_Queue_Available() > 3) {  // Must be: C1 | STARTING_ADDRESS | LENGTH
		uint8_t *data = RX_Queue_Peek();

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
			RX_Queue_Pop(1);
			continue;
		}

#ifdef DEBUG
		printf("Valid LoRa reply: starting_address=0x%02X, data_length=%d\n",
				starting_address, data_length);
#endif

		if (RX_Queue_Available() < total_reply_size)
			break;

		if (starting_address == current_driver.registers->pid) {
			if (current_driver.registers->pid == 0xFF) {
				RX_Queue_Pop(1);
				continue;
			}

			if (data_length != 7)
				return;
			for (int i = 0; i < 6; i++)
				printf("0x%02X ", data[3 + i]);
			printf("\n");
			RX_Queue_Pop(total_reply_size);
			return;
		}

		for (int i = 0; i < data_length; i++) {
			uint8_t current_data = data[3 + i];
			uint8_t reg_address = starting_address + i;

			if (reg_address == current_driver.registers->addh)
				cache.addh = current_data;
			else if (reg_address == current_driver.registers->addl)
				cache.addl = current_data;
			else if (reg_address == current_driver.registers->netid && current_driver.registers->netid != 0xFF)
				cache.netid = current_data;
			else if (reg_address == current_driver.registers->reg0)
				cache.reg0 = current_data;
			else if (reg_address == current_driver.registers->reg1)
				cache.reg1 = current_data;
			else if (reg_address == current_driver.registers->reg2)
				cache.reg2 = current_data;
			else if (reg_address == current_driver.registers->reg3)
				cache.reg3 = current_data;
		}
		receiving_config_data = false;
		RX_Queue_Pop(total_reply_size);
	}
}

void RX_Queue_Push(uint8_t *data, uint16_t size) {
    if (queue_tail + size <= UART_QUEUE_SIZE) {
        memcpy(&UART_Queue[queue_tail], data, size);
        queue_tail += size;
    } else {
        uint16_t first_part = UART_QUEUE_SIZE - queue_tail;
        memcpy(&UART_Queue[queue_tail], data, first_part);
        memcpy(&UART_Queue[0], data + first_part, size - first_part);
        queue_tail = size - first_part;
    }

    uint16_t available = RX_Queue_Available();
    if (available > UART_QUEUE_SIZE) {
        queue_head = queue_tail;
    }
}

uint16_t RX_Queue_Available(void) {
	if (queue_tail >= queue_head) {
		return queue_tail - queue_head;
	} else {
        return UART_QUEUE_SIZE - queue_head + queue_tail;
	}
}

uint8_t* RX_Queue_Peek(void) {
    if (queue_head == queue_tail) return NULL;
    return &UART_Queue[queue_head];
}

void RX_Queue_Pop(uint16_t size) {
    uint16_t available = RX_Queue_Available();
    size = (size > available) ? available : size;
    queue_head = (queue_head + size) % UART_QUEUE_SIZE;
}

void RX_Queue_Process(void) {
    while (RX_Queue_Available() >= 2) {
        uint8_t *data = RX_Queue_Peek();

        if (data == NULL)
        	break;

        uint8_t ptype = data[0];
        uint8_t payload_length = data[1];
		uint16_t total_packet_size = OPCODE_OFFSET + LENGTH_OFFSET + payload_length + CRC_LEN;

        if (!RX_Queue_Validate_Packet_Length(data, ptype, total_packet_size)) {
			RX_Queue_Pop(1);
			continue;
        }

		if (RX_Queue_Available() < total_packet_size)
			break;

		uint16_t received_crc = (data[total_packet_size - 2] << 8) | data[total_packet_size - 1];


        if (!RX_Queue_Validate_Packet_CRC(data, total_packet_size, received_crc)) {
			RX_Queue_Pop(1);
			continue;
        }


#ifdef DEBUG
        printf("Valid packet: opcode=0x%02X, payload_len=%d\n", ptype, payload_length);
#endif

		Receive_Packet_Handler(data, total_packet_size, ptype);


		RX_Queue_Pop(total_packet_size);

    }
}

bool RX_Queue_Validate_Packet_Length(uint8_t *data, uint8_t ptype, uint16_t total_packet_size) {
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

	return SUCCESS;
}

bool RX_Queue_Validate_Packet_CRC(uint8_t *data, uint16_t total_packet_size, uint16_t received_crc) {
	uint16_t calculated_crc = Calculate_CRC16(data, total_packet_size - CRC_LEN);
	if (received_crc != calculated_crc) {
#ifdef DEBUG
		printf("Calculated CRC = %d, got CRC = %d\n", calculated_crc, received_crc);
#endif
		return FAIL;
	}
	return SUCCESS;
}
