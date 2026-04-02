#include "main.h"

uint32_t DEBUG_timestamp;

uint32_t Get_Timestamp() {
//	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
//	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
//	currTime.tm_year = currentDate.Year + 100;  // In fact: 2000 + 18 - 1900
//	currTime.tm_mday = currentDate.Date;
//	currTime.tm_mon  = currentDate.Month - 1;
//
//	currTime.tm_hour = currentTime.Hours;
//	currTime.tm_min  = currentTime.Minutes;
//	currTime.tm_sec  = currentTime.Seconds;

	return HAL_GetTick();
}

void rand_delay() {
	HAL_Delay(1);
}

uint32_t Get_Rand(uint32_t x) {
  uint32_t z = (x += 0x9E3779B9u); // 2 to the 32 divided by golden ratio; adding forms a Weyl sequence
  z ^= z >> 16;
  z *= 0x21f0aaadu;
  z ^= z >> 15;
  z *= 0x735a2d97u;
  z ^= z >> 15;
  return z;
}

#ifdef DEBUG
void DEBUG_Start_Timing() {
	DEBUG_timestamp = Get_Timestamp();
}

uint32_t DEBUG_End_Timing() {
	return (Get_Timestamp() - DEBUG_timestamp);
}
#endif
