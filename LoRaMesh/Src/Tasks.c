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
	Queue_Process();

	// Each 1000ms
	if (ticks % 80 == 0) {
		Pending_Messages_Table_Process();
	}
}
