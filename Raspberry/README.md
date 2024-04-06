# Semubot 


## Introduction


TODO


## Transmission protocol

Computer to Nucleo: 

PACKET:

* Delimiter - 0x4994
* Command byte - 1 byte
* Motor control - 1 byte each, 4 total
* CRC - TODO

Nucleo to Computer:

PACKET:

* Delimiter - 0x4994
* Encoder position - 2 byte each, 4 total
