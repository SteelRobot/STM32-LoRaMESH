#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "stm32l1xx_hal.h"

#define ADDH 0x0
#define ADDL 0x1
#define NETID 0x2
#define REG0 0x3
#define REG1 0x4
#define REG2 0x5
#define REG3 0x6
#define CRYPT_H 0x7
#define CRYPT_L 0x8
#define PID 0x80

enum Mode {
	MODE_NORMAL,
	MODE_WOR,
	MODE_CONFIG,
	MODE_SLEEP
};

void LoRa_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2);
void LoRa_ModeSelect(enum Mode mode);
void LoRa_WriteAUXToLED();
void LoRa_ReadRegister(uint8_t address);
void LoRa_WriteRegister(uint8_t address, uint8_t parameter);
void LoRa_SendCommand();
void Lora_Convert(uint8_t *buffer, uint8_t *com_buffer, uint8_t buffer_size);
void LoRa_TestModeTransitions();
void LoRa_ReadProductInfo();

#endif /* INC_LORA_H_ */
