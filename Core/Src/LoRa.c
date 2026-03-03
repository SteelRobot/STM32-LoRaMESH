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
	GPIOx_M0 = M0_GPIO_Port;
	GPIOx_M1 = M1_GPIO_Port;
	GPIOx_AUX = AUX_GPIO_Port;
	GPIOx_LED = LED_GPIO_Port;
	PIN_M0 = M0_Pin;
	PIN_M1 = M1_Pin;
	PIN_LED = LED_Pin;
	PIN_AUX = AUX_Pin;
	LoRa_UART = huart1;
	COM_UART = huart2;
}

// 00 - нормальный режим
// 01 - WOR режим
// 10 - режим конфига
// 11 - режим глубокого сна
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
	HAL_Delay(1000);
}

// Команды имеют формат Cx+Address+Length+Parameter
// Пример "C10501" - чтение регистра 05H
// Было решено дать пользователю читать и модифицировать только 1 регистр за раз
void LoRa_ReadRegister(uint8_t address) {
	uint8_t command_index = 0;
    command_buffer[command_index++] = COMMAND_READ_PREFIX;
    command_buffer[command_index++] = address;
    command_buffer[command_index++] = 0x1;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, SLEEP_TIME);
    HAL_UART_Receive(LoRa_UART, response_buffer, sizeof(response_buffer), SLEEP_TIME);
    Lora_Convert(response_buffer, com_response_buffer, sizeof(response_buffer));
    HAL_UART_Transmit(COM_UART, com_response_buffer, sizeof(com_response_buffer), SLEEP_TIME);
}

void LoRa_WriteRegister(uint8_t address, uint8_t parameter) {
	uint8_t command_index = 0;
    command_buffer[command_index++] = COMMAND_WRITE_PREFIX;
    command_buffer[command_index++] = address;
    command_buffer[command_index++] = 0x1;
    command_buffer[command_index++] = parameter;
	HAL_UART_Transmit(LoRa_UART, command_buffer, command_index, SLEEP_TIME);
    HAL_UART_Receive(LoRa_UART, response_buffer, sizeof(response_buffer), SLEEP_TIME);
    Lora_Convert(response_buffer, com_response_buffer, sizeof(response_buffer));
    HAL_UART_Transmit(COM_UART, com_response_buffer, sizeof(com_response_buffer), SLEEP_TIME);
}

// В регистре PID 7 байт, перед ними модуль также возвращает отправленную на него команду (C18007).
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
    Lora_Convert(product_response_buffer, com_product_response_buffer, sizeof(product_response_buffer));
    HAL_UART_Transmit(COM_UART, preamble, sizeof(preamble), 1000);
    HAL_UART_Transmit(COM_UART, com_product_response_buffer, sizeof(com_product_response_buffer), SLEEP_TIME);
    HAL_Delay(SLEEP_TIME);
}

void Lora_Convert(uint8_t *buffer, uint8_t *com_buffer, uint8_t buffer_size) {
	for (int i = 0; i < buffer_size; i++) {
		com_buffer[i * 3] 		= (buffer[i] >> 4) < 10 ? (buffer[i] >> 4) + '0' : (buffer[i] >> 4) - 10 + 'A';
		com_buffer[i * 3 + 1] 	= (buffer[i] & 0x0F) < 10 ? (buffer[i] & 0x0F) + '0' : (buffer[i] & 0x0F) - 10 + 'A';
		com_buffer[i * 3 + 2]	= ' ';
	}
	com_buffer[buffer_size * 3 - 1] = '\n';
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == AUX_Pin) {
		GPIO_PinState AUX_state = HAL_GPIO_ReadPin(GPIOx_AUX, PIN_AUX);
		HAL_GPIO_WritePin(GPIOx_LED, PIN_LED, AUX_state);
	}
}

void LoRa_TestModeTransitions() {
    // Placeholder function to test mode transitions
    // This function would call the other functions to test transitions (e.g., sleep → receive → transmit)
}
