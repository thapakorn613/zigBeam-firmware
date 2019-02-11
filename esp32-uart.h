#ifndef espUART_h
#define espUART_h

// #include <stdint.h>
// #include <HardwareSerial.h>

#define SER_WAITING_FOR_1ST_HEADER      1
#define SER_WAITING_FOR_2ND_HEADER      2
#define SER_WAITING_FOR_LENGTH          3
#define SER_WAITING_FOR_CMD             4
#define SER_WAITING_FOR_1ST_CMDID       5
#define SER_WAITING_FOR_2ND_CMDID       6

#define SER_BUFFER_SIZE                 64

#define SERIAL_1ST_HEADER        0x54
#define SERIAL_2ND_HEADER        0xFE


// void picSerialEvent();
// bool newCmdPacketReady();
// void clearCmdReadyFlag();

// extern uint8_t gbl1stCMDBuffer[SER_BUFFER_SIZE];
// extern uint8_t gbl2ndCMDBuffer[SER_BUFFER_SIZE];

// extern bool gblSerialDebugFlag;
// extern int gblSerialState;
// extern int gblSerialCmdLength;
// extern uint8_t inByte;

// extern HardwareSerial Serial;
// extern HardwareSerial extSerial;

#endif
