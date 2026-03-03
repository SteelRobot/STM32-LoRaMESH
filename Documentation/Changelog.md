# Changelog

## 03/03/2026 - day

- Created LoRa.c and LoRa.h
  - Implemented module initialization (Reading pins M0, M1, AUX, COM-UART, LORA-UART2)
  - Implemented reading and writing to registers
  - Implemented reading Product Information
- Created GitHub repository

## 03/03/2026 - night

- Improved on LoRa.c and LoRa.h
  - Implemented default settings reset
  - Implemented ways to change specific parameters, instead of writing plainly to registers
  - Added Enums to LoRa.h to implement previous point
