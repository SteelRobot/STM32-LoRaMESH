#ifndef INC_LORA_H_
#define INC_LORA_H_

#include <stdbool.h>

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

#define LORA_REPLY_OFFSET 3
#define LORA_REPLY_CODE 0xC1

#define SUCCESS	true
#define FAIL false

extern UART_HandleTypeDef *LoRa_UART;
extern UART_HandleTypeDef *COM_UART;
extern RTC_HandleTypeDef *Mesh_RTC;
extern TIM_HandleTypeDef *TASK_TIM;

extern uint16_t my_id;
extern uint8_t my_channel;

// MODE ENUMS BEGIN
enum Mode {
	MODE_NORMAL,
	MODE_WOR,
	MODE_CONFIG,
	MODE_SLEEP
};
// MODE ENUMS END


// REG0 ENUMS BEGIN
enum Serial_Port_Rate {
	SPR_1200,
	SPR_2400,
	SPR_4800,
	SPR_9600,
	SPR_19200,
	SPR_38400,
	SPR_57600,
	SPR_115200
};
enum Serial_Parity_Bit {
	SPB_8N1,
	SPB_8O1,
	SPB_8E1
};
enum Air_Data_Rate {
	ADR_0_3K,
	ADR_1_2K,
	ADR_2_4K,
	ADR_4_8K,
	ADR_9_6K,
	ADR_19_2K,
	ADR_38_4K,
	ADR_62_5K
};
// REG0 ENUMS END


// REG1 ENUMS BEGIN
enum Sub_Packet_Setting {
	SPS_240,
	SPS_128,
	SPS_64,
	SPS_32
};

enum Transmitting_Power {
	TP_22DBM,
	TP_17DBM,
	TP_13DBM,
	TP_10DBM
};
// REG1 ENUMS END


// REG3 ENUMS BEGIN
enum Transmission_Mode {
	TM_TRANSPARENT,
	TM_FIXED_POINT
};

enum WOR_Transceiver_Control {
	WOR_RECEIVER,
	WOR_TRANSMITTER
};

enum WOR_Cycle {
	CYCLE_500MS,
	CYCLE_1000MS,
	CYCLE_1500MS,
	CYCLE_2000MS,
	CYCLE_2500MS,
	CYCLE_3000MS,
	CYCLE_3500MS,
	CYCLE_4000MS
};
// REG3 ENUMS END


void LoRa_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, RTC_HandleTypeDef *hrtc, TIM_HandleTypeDef *tim);
void LoRa_ModeSelect(enum Mode mode);
void LoRa_WriteAUXToLED(void);
void LoRa_ReadRegister(uint8_t starting_address, uint8_t length);
void LoRa_WriteRegister(uint8_t address, uint8_t length, uint8_t parameter);
void LoRa_SendData(uint8_t *buffer, uint8_t buffer_size);
void LoRa_ReadProductInfo(void);
void LoRa_ResetFirmware(void);

void LoRa_Set_Address(uint16_t addr);
void LoRa_Set_NetID(uint8_t netid);

void LoRa_Set_SerialPortRate(enum Serial_Port_Rate spr);
void LoRa_Set_SerialParityBit(enum Serial_Parity_Bit spb);
void LoRa_Set_AirDataRate(enum Air_Data_Rate adr);

void LoRa_Set_SubPacketSetting(enum Sub_Packet_Setting sps);
void LoRa_Set_RSSIAmbientNoise(bool state);
void LoRa_Set_TransmittingPower(enum Transmitting_Power tp);

void LoRa_Set_Channel(uint8_t channel); // 19 for Russia

void LoRa_Set_RSSIEnable(bool state);
void LoRa_Set_TransmissionMode(enum Transmission_Mode tm);
void LoRa_Set_ReplyEnable(bool state);
void LoRa_Set_LBTEnable(bool state);
void LoRa_Set_WORTransceiverControl(enum WOR_Transceiver_Control state);
void LoRa_Set_WORCycle(enum WOR_Cycle cycle);

void LoRa_Set_Encryption(uint16_t key);

void LoRa_Start_Receive(void);
void Process_LoRa_Reply(void);

void RX_Queue_Push(uint8_t *data, uint16_t size);
uint16_t RX_Queue_Available(void);
uint8_t* RX_Queue_Peek(void);
void RX_Queue_Pop(uint16_t size);
void RX_Queue_Process(void);
bool RX_Queue_Validate_Packet(uint8_t *data, uint8_t ptype, uint16_t total_packet_size, uint16_t received_crc);

#endif /* INC_LORA_H_ */
