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

## 04/03/2026 - day

- Improved on LoRa.c and LoRa.h
  - Removed the include of HAL library from library header files (essentially a bugfix)
  - Fixed the implementation of changing parameters of the module, where it used to write over the whole register, instead of just the bits
  - Added functions to set the Address, Channel, NetID, and Encryption key
- Changed project structure
  - Moved library files to a separate source folder
  - Included it in project settings (Not sure if that would get carried over to Git, since .settings folder is excluded from commits)
    - To remedy that, added the instruction into [User Manual](UserManual.md)

## 04/03/2026 - night

- Further LoRa.c improvements
  - Added interrupt-handling to send messages from keyboard
  - Added interrupt-handling to receive messages from other modules
  - Edited a few functions to disable interrupts in them in order to properly receive module's own replies (Not external messages)

## 09/03/2026 - evening

- Separated library further from hardware by putting it as an outside lib
- Added a project for an STM32L4
- Fixed interrupt handling
