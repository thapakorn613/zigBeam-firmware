#ifndef __ESP32_GoGoBoard_H__
#define __ESP32_GoGoBoard_H__ 

//? init gogo configure

////////////////////////////////////////////////////////////////
// Pins Definitions
#define ESP_PIC_CONNECT                     0
#define HW_I2C_SDA                          21
#define HW_I2C_SCL                          22

////////////////////////////////////////////////////////////////
// gblDeviceRegisters - Report Packet Type 0
#define REG_INPUT_A1                        1
#define REG_INPUT_JOYSTICK                  9
#define REG_ACTIVE_SERVO                    11
#define REG_SERVO_ANGLE                     12
#define REG_HARDWARE_ID                     17
#define REG_FIRMWARE_ID_GOGO                20
#define REG_FIRMWARE_ID_STM                 21
#define REG_ACTIVE_MOTOR                    22
#define REG_MOTOR_STATUS                    23
#define REG_MOTOR_DIRECTION                 24
#define REG_WIFI_STATUS                     43

////////////////////////////////////////////////////////////////
// ZigBee Extension - Report Packet Type 19
#define REG_NODE_ID                         15
#define REG_NODE_STATUS                     16
#define REG_SHORT_ADDR                      17
#define REG_IEEE_ADDR                       19
#define REG_CO_ZB_STATUS                    27
#define REG_CO_CODE_HIGH                    29
#define REG_CO_CODE_LOW                     30

////////////////////////////////////////////////////////////////
// Variables Definitions
#define REGISTER_SIZE                       64
#define MAX_PAYLOAD_SIZE                    200
#define MAX_OPCODE_SIZE                     1984
#define HARDWARE_ID1                        0x06
#define HARDWARE_ID2                        0x00
#define HARDWARE_ID3                        0x0c
#define FIRMWARE_ID1                        0x01


#define CMD_PACKET                          0
#define FLASH_MEMORY_OPERATION_PACKET       1
#define RASPBERRY_PI_CMD_PACKET             2
#define DATADRIVEN_CMD_PACKET               4
#define HOPHER_CMD_PACKET                   5
#define ZIGBEE_CMD_PACKET                   6
#define PIC_REPORT_PACKET                   17

#define ZIGBEE_PACKET_TYPE                  0
#define RPI_PACKET_TYPE                     2

////////////////////////////////////////////////////////////////
//  Command Definitions
#define CMD_PING                            1
#define CMD_MOTOR_ON_OFF                    2
#define CMD_MOTOR_DIRECTION                 3
#define CMD_MOTOR_RD                        4
#define CMD_SET_POWER                       6
#define CMD_SET_ACTIVE_PORTS                7
#define CMD_TOGGLE_ACTIVE_PORT              8
#define CMD_SET_SERVO_ANGLE                 9
#define CMD_LED_CONTROL                     10
#define CMD_BEEP                            11
#define CMD_AUTORUN_STATE                   12
#define CMD_LOGO_CONTROL                    13
#define CMD_SET_SERVO_PORTS                 14

#define CMD_SYNC_RTC                        50
#define CMD_READ_RTC                        51
#define CMD_SHOW_SHORT_TEXT                 60
#define CMD_SHOW_LONG_TEXT                  61
#define CMD_CLEAR_SCREEN                    62

#define CMD_VOICE_PLAY_PAUSE                70
#define CMD_VOICE_NEXT_TRACK                71
#define CMD_VOICE_PREV_TRACK                72
#define CMD_VOICE_GOTO_TRACK                73
#define CMD_VOICE_ERASE_ALL_TRACKS          74

#define CMD_REBOOT                          100

#define LOGO_SET_MEMORY_POINTER             1
#define FLASH_SET_MEMORY_POINTER            2
#define MEM_WRITE_BYTES                     3
#define MEM_READ_BYTES                      4

#define CMD_WIFI_CONFIG                     1

#define ZCMD_PERMITJOIN                     1
#define ZCMD_IDENTIFYQ                      2
#define ZCMD_COMMAND                        3
#define ZCMD_ACK                            4

///////////////////////////////////////////////////////////////
// Network Definitions
#define NETWORK_REPORT_PACKET               18
#define NWK_SWITCH_DURATION                 300 // 300 * 100ms -> 30s until check network again

#define API_DATALOG                         1
#define API_IFTTT                           2
#define API_CUSTOM                          3

#define NWK_GET                             1
#define NWK_POST                            2
#define NWK_UPDATE                          3
#define NWK_DELETE                          4

///////////////////////////////////////////////////////////////
// MISC Definitions
#define BEEP_DURATION                       4

///////////////////////////////////////////////////////////////
// Serial process -- PIC -> ESP32
#define ENDPOINT_ID                         0
#define CATEGORY_ID                         1
#define CMD_ID                              2

///////////////////////////////////////////////////////////////
// Serial process -- PIC -> ESP32
#define REPORTPKT_HEADER_LENGTH             3
#define PASSTHROUGHPKT_HEADER_LENGTH        4

///////////////////////////////////////////////////////////////
// ZigBee CMD Definitions
#define RES_DEVICE_ANNOUCE                  1
#define RES_READATTR                        2
#define RES_SERIAL                          8
#define RES_REPORT                          9
#define RES_IEEE_REQ                        10
#define RES_NWK_REQ                         11
#define RES_CO_CODE                         12

///////////////////////////////////////////////////////////////
// OLED display Command
#define INIT_DISPLAY                        0
#define CLEAR_DISPLAY                       1
#define BEGIN_DOWNLOAD_DISPLAY              2
#define END_DOWNLOAD_DISPLAY                3
#define STOP_RUN_DISPLAY                    4
#define STOP_SENSOR_DISPLAY                 5

// main display pages
#define MAIN_DISPLAY                        0
#define SENSOR_DISPLAY_1                    1
#define SENSOR_DISPLAY_2                    2
#define SENSOR_DISPLAY_3                    3
#define SENSOR_DISPLAY_4                    4
#define PROXIMITY_DISPLAY                   5
#define GESTURE_DISPLAY                     6
#define LIGHTCOLOUR_DISPLAY                 7
#define SETTING_DISPLAY                     8

// setting display pages
#define SET_SETTING_NUMBER                  2
#define SET_UTILIZE_GESTURE                 0
#define SET_SENSOR_RESOLUTION               1

///////////////////////////////////////////////////////////////
// Sensors Definitions
#define sensorCount                         4

#define bit_test(Var, bit) (Var & (0x01 << bit))

#endif