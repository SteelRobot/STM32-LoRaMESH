# STM32-LoRaMESH

A library for implementing a mesh-network using STM32 microcontrollers with LoRa modules

## LoRa.c and LoRa.h

Responsible for communication between the STM32 and EBYTE's LoRa module via UART.

- Reading registers
- Writing to registers
- Sending responses to the PC over Virtual COM-port

## Mesh.c

- Managing routing table
- Managing outgoing packet queue

## Packet_Handlers.c

- Processing each packet type
- Serializing and de-serializing packets

## Util.c

- Various util functions

## Tasks.c

- Calling functions on timer callback
