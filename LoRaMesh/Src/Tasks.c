#include "main.h"
#include "Tasks.h"
#include "LoRa.h"
#include "Mesh.h"

uint16_t ticks = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TASK_TIM->Instance) {
    	ticks++;
    	Process_Tasks();
    }
}

void Process_Tasks(void) {
	//Process incoming packets queue
	RX_Queue_Process();

	// Each 500ms
	if (ticks % 40 == 0) {
		TX_Queue_Process();
	}
}
