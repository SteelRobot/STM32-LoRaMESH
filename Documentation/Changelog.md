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

## 10/03/2026

- Created new dev branch

## 12/03/2026

- Added support for printf function to use Virtual COM port UART
- Started work on the networking protocol

## 24/03/2026

- Switched incoming packets logic handling from interrupts to DMA

## 03/04/2026

- Addition of RTC to use for PRNG later.
- Separation of packet formats into a separate file
- Util.c file added

## 04/04/2026

- Addition of Ping packets. DMA fixes

## 05/04/2026

- STM32L4-specific fixes

## 08/04/2026

- RREQ packet fixes and features (like RREQ expiring)

## 09/04/2026

- Change of how Hello packets are handled

## 10/04/2026

- Addition of queues to process incoming packets
- Addition of a timer to process queue within its callback
- Validation of incoming packets in the queue based on length

## 11/04/2026

- First attempts at creating a queue for outgoing packets

## 29/04/2026

- Remake of outgoing packets queue based on storing raw bytes.
- Addition of ACK packets and handling confirmations of packets
- Addition of a CRC field to further validate packets (if packet type is valid, but CRC is not)
- Refactored Packet_Formats.h file to make it more readable
- Refactored functions to make them consistent with each other
