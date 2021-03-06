#include <Wire.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>

#include "driver/uart.h" //? this include file from tools/sdk/include/*
#include "esp32-hal-uart.c"

#include "Arduino.h"
#include "StackArray.h"
#include "esp32-firmware.h"
#include "esp32-uart.h"
#include "REG_SDM120.h"

TaskHandle_t statusLED;

const int ledPin = 2;
int ledStatus = 0;
int incomingByte = 0;

int gblSerialState = SER_WAITING_FOR_1ST_HEADER;
bool gblUseFirstCmdBuffer = 1;
int gblSerialCmdCounter = 0;
int gblSerialCmdChecksum = 0;
bool gblNewCmdReady = 0;
int gblSerialIdleCounter = 0;

// setup timer interrupt for send serial
hw_timer_t *Timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

uint8_t gbl1stCMDBuffer[SER_BUFFER_SIZE];
uint8_t gbl2ndCMDBuffer[SER_BUFFER_SIZE];

int gblExtSerialState = SER_WAITING_FOR_1ST_HEADER;
bool gblUseFirstExtCmdBuffer = 1;
int gblExtSerialCmdCounter = 0;
bool gblNewExtCmdReady = 0;
uint8_t cmd_highByte = 0;
uint8_t cmd_lowByte = 0;

uint16_t shortAddrDevice = 0;
uint16_t source_addr = 0;

uint8_t gbl1stExtCMDBuffer[SER_BUFFER_SIZE];
uint8_t gbl2ndExtCMDBuffer[SER_BUFFER_SIZE];

// dynamic incoming packet due to GoGoBoard protocol
int inByteLength = 0;
int inExtLength = 0;

uint8_t inByte;
uint8_t inByte_rx;

String Buffer = "PERMITJOIN 255\n";
// setup uart handler
uart_t *uart_0 = &_uart_bus_array[UART_NUM_0];
uart_t *uart_2 = &_uart_bus_array[UART_NUM_2];

bool gblIsCoordinatorZB = false;
volatile int gblWaitForZigBeeResponse = 0; // 1 = waiting , 2 = timeout
volatile int gblWaitZBResponseCounter = 0;

int gblNetworkControl = 0; // 1 = datalog, 2 = ifttt, 3 = customURL
char gblNetworkPacket[MAX_PAYLOAD_SIZE] = {0};

int gblBeepDurationCounter = 0;
volatile bool gblNeedToStartABeep = false;
volatile bool gblTimeToStartBeep = false;
volatile bool gblTimeToStopBeep = false;

int gblAutoSwitchNWKCounter = 0;
volatile bool gblTimeToCheckNWK = false;
volatile bool gblTimeToSwitchNWK = false;

char *cmdToCC;
StackArray<uint8_t> gblBuffer;
StackArray<uint8_t> gblInputStack;

HardwareSerial zbSerial(2);

ModbusMaster node;

//! RS483 part
float HexTofloat(uint32_t x)
{
    return (*(float *)&x);
}

uint32_t FloatTohex(float x)
{
    return (*(uint32_t *)&x);
}
//------------------------------------------------

float Read_Meter_float(char addr, uint16_t REG)
{
    float i = 0;
    uint8_t j, result;
    uint16_t data[2];
    uint32_t value = 0;
    node.begin(addr, Serial);
    result = node.readInputRegisters(REG, 2); ///< Modbus function 0x04 Read Input Registers
    delay(500);
    if (result == node.ku8MBSuccess)
    {
        for (j = 0; j < 2; j++)
        {
            data[j] = node.getResponseBuffer(j);
        }
        value = data[0];
        value = value << 16;
        value = value + data[1];
        i = HexTofloat(value);
        //Serial.println("Connec modbus Ok.");
        return i;
    }
    else
    {
        Serial.print("Connec modbus fail. REG >>> ");
        Serial.println(REG, HEX); // Debug
        delay(1000);
        return 0;
    }
}

void GET_METER()
{                // Update read all data
    delay(1000); // เคลียบัสว่าง
    for (char i = 0; i < Total_of_Reg; i++)
    {
        DATA_METER[i] = Read_Meter_float(ID_meter, Reg_addr[i]); //แสกนหลายตัวตามค่า ID_METER_ALL=X
    }
}
//! End RS485 Part

int count = 0;

void toggleStatusLED()
{
    digitalWrite(ledPin, ledStatus); // turn on the LED
    ledStatus ^= 1;
}

void beep()
{
    //? Disable beep from here
    // this flag causes timer0 to sound the beeper
    portENTER_CRITICAL(&timerMux0);
    gblNeedToStartABeep = true;
    portEXIT_CRITICAL(&timerMux0);
}
void reportCmd()
{
    Serial.print("Report");
}

// * //////////////////////////////////////////////////////
// *    10ms timer1 interrupt
void IRAM_ATTR miscTimer()
{
    portENTER_CRITICAL_ISR(&timerMux0);

    if (gblNeedToStartABeep)
    {
        if (gblBeepDurationCounter++ == 0)
        {
            gblTimeToStartBeep = true;
        }
        else if (gblBeepDurationCounter == BEEP_DURATION)
        {
            gblTimeToStopBeep = true;

            gblBeepDurationCounter = 0;
            gblNeedToStartABeep = false;
        }
    }
    portEXIT_CRITICAL_ISR(&timerMux0);
}

void receiveZBPkt()
{
    uint8_t *inZBPkt; // point to CMD buffer

    if (newExtCmdPacketReady())
    {
        // Serial.print("int handler : ");
        inZBPkt = (newExtCmdPacketReady() == 1) ? gbl1stExtCMDBuffer : gbl2ndExtCMDBuffer;
        // inZBPkt เก็บตั้งแต่ไบร์ทที่ 6 ไป

        clearExtCmdReadyFlag();

        uint8_t cmd = (cmd_highByte << 8) + (cmd_lowByte);
        // //! actually cmdID = inZBPkt[0] and inZBPkt[1] but high byte always 0
        if (cmd_highByte == ZIGBEE_PACKET_TYPE)
        {
            processZBPkt(cmd, inZBPkt);
        }
    }
}

void processZBPkt(uint8_t cmd, uint8_t *inZBPkt)
{
    // โยนขึ้นเน็ต
    Serial.print("Command Id : ");
    Serial.println(cmd);
    for (int i = 0; i < inExtLength; i++)
    {
        Serial.print(" ");
        Serial.print(inZBPkt[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");

    switch (cmd)
    {
    case RES_DEVICE_ANNOUCE:
        //? inZBPkt[8] + inZBPkt[9] = ShortAddr
        shortAddrDevice = (inZBPkt[8] << 8) + (inZBPkt[9]);
        Serial.print("shortAddrDevice : ");
        Serial.println(shortAddrDevice);
        // xSemaphoreTake(regSemaphore, portMAX_DELAY);
        // gblDeviceRegister[REG_NODE_STATUS] = 1;
        // memcpy(gblDeviceRegister + REG_SHORT_ADDR, inZBPkt + 8, 2);
        // memcpy(gblDeviceRegister + REG_IEEE_ADDR, inZBPkt, 8);
        // xSemaphoreGive(regSemaphore);
        break;

    case RES_READATTR:
        // if (gblWaitForZigBeeResponse == 1) // still waiting for response
        // {
        //     // check data type
        //     uint8_t data_type = inZBPkt[7];
        //     if (data_type == 0x10) // boolean
        //     {
        //         gblStackPush(inZBPkt[8]);
        //     }
        //     else
        //     {
        //         if (data_type == 0x20) // uint8
        //         {
        //             gblStackPush((uint8_t)inZBPkt[10]);
        //         }
        //         else if (data_type == 0x29) // signed int16
        //         {
        //             gblStackPush((int)((inZBPkt[10] << 8) + (inZBPkt[11])));
        //         }
        //         else if (data_type == 0x21) // uint16
        //         {
        //             gblStackPush((uint)((inZBPkt[10] << 8) + (inZBPkt[11])));
        //         }
        //         else if (data_type == 0x30) // 8-bit enumeration
        //         {
        //             gblStackPush(inZBPkt[10]);
        //         }
        //         else
        //         {
        //             gblStackPush(0);
        //         }
        //     }
        //     portENTER_CRITICAL(&timerMux2);
        //     gblWaitForZigBeeResponse = 0; // reset wait flag
        //     portEXIT_CRITICAL(&timerMux2);
        // }
        break;

    case RES_SERIAL:
        // send serial success
        // if (inZBPkt[0])
        // {
        //     gblIsCoordinatorZB = true;
        //     // xSemaphoreTake(regSemaphore, portMAX_DELAY);
        //     // gblDeviceRegister[REG_CO_ZB_STATUS] = 1;
        //     // xSemaphoreGive(regSemaphore);
        // }
        break;

    case RES_REPORT: // zigbee network
        source_addr = uint16_t(inZBPkt[0] << 8) + inZBPkt[1];
        // // // reportEndPoint = inZBPkt[2];
        // // // reportClusterID = (inZBPkt[3] << 8) + inZBPkt[4];
        // // // reportRegisterCount = inZBPkt[5];
        Serial.print("source_addr : ");
        Serial.println(source_addr);

        // Serial.print(" , reportEndPoint : ");
        // Serial.print(reportEndPoint);

        // Serial.print(" , reportClusterID : ");
        // Serial.print(reportClusterID);

        // Serial.print(" , reportRegisterCount : ");
        // Serial.println(reportRegisterCount);
        // if (inZBPkt[6] == NETWORK_REPORT_PACKET) // check packet type network report
        // {

        //     // int data_len;
        //     // data_len = (inZBPkt[9] << 8) + inZBPkt[10] + 4; // 2 payload size plus destination API & rest method

        //     // //memcpy(gblNetworkPacket, inZBPkt + 7, data_len);

        //     // // set flag destination API to trig nwk process
        //     // gblNetworkControl = inZBPkt[7];
        //     beep(); // debug receive network packet
        // }
        break;

    case RES_IEEE_REQ:
        // for (int i = 0; i < inExtLength; i++)
        //     Serial.println(inZBPkt[i]);
        break;

    case RES_NWK_REQ:
        // for (int i = 0; i < inExtLength; i++)
        //     Serial.println(inZBPkt[i]);
        break;

    case RES_CO_CODE:
        // xSemaphoreTake(regSemaphore, portMAX_DELAY);
        // gblDeviceRegister[REG_CO_CODE_HIGH] = inZBPkt[0];
        // gblDeviceRegister[REG_CO_CODE_LOW] = inZBPkt[1];
        // xSemaphoreGive(regSemaphore);
        break;
    }
}

// * //////////////////////////////////////////////////////////////
// *    Serial process

void setInterruptHandlerFunction(uart_t *uart, void (*intr_handler_fn)(void *))
{
    uartDisableInterrupt(uart);

    UART_MUTEX_LOCK();
    uart->dev->conf1.rxfifo_full_thrhd = 112;
    uart->dev->conf1.rx_tout_thrhd = 2;
    uart->dev->conf1.rx_tout_en = 1;
    uart->dev->int_ena.rxfifo_full = 1;
    uart->dev->int_ena.frm_err = 1;
    uart->dev->int_ena.rxfifo_tout = 1;
    uart->dev->int_clr.val = 0xffffffff;

    esp_intr_alloc(UART_INTR_SOURCE(uart->num), (int)ESP_INTR_FLAG_IRAM, intr_handler_fn, NULL, &uart->intr_handle);
    UART_MUTEX_UNLOCK();
}

static void IRAM_ATTR zbSerialEvent(void *parameter) //! this will not include checksum in receive buffer (gbl1stCMDBuffer)
{
    uint8_t inByte;
    if (uart_2->intr_handle != NULL)
    {
        uart_2->dev->int_clr.rxfifo_full = 1;
        uart_2->dev->int_clr.frm_err = 1;
        uart_2->dev->int_clr.rxfifo_tout = 1;

        while (uart_2->dev->status.rxfifo_cnt || (uart_2->dev->mem_rx_status.wr_addr != uart_2->dev->mem_rx_status.rd_addr))
        {
            inByte = uart_2->dev->fifo.rw_byte;
            if (inByte == SERIAL_1ST_HEADER && gblExtSerialState == SER_WAITING_FOR_1ST_HEADER)
            {
                gblExtSerialState = SER_WAITING_FOR_2ND_HEADER;
            }
            else if (gblExtSerialState == SER_WAITING_FOR_2ND_HEADER)
            {
                if (inByte == SERIAL_2ND_HEADER)
                {
                    gblExtSerialState = SER_WAITING_FOR_1ST_CMDID;
                }
                else
                {
                    gblExtSerialState = SER_WAITING_FOR_1ST_HEADER;
                }
            }
            else if (gblExtSerialState == SER_WAITING_FOR_1ST_CMDID)
            {
                cmd_highByte = inByte;
                gblExtSerialState = SER_WAITING_FOR_2ND_CMDID;
            }
            else if (gblExtSerialState == SER_WAITING_FOR_2ND_CMDID)
            {
                cmd_lowByte = inByte;
                gblExtSerialState = SER_WAITING_FOR_LENGTH;
            }
            else if (gblExtSerialState == SER_WAITING_FOR_LENGTH)
            {
                inExtLength = inByte;
                //Serial.print("len: ");
                //Serial.println(inExtLength);
                gblExtSerialCmdCounter = 0;
                gblExtSerialState = SER_WAITING_FOR_CMD;
            }
            else if (gblExtSerialState == SER_WAITING_FOR_CMD)
            {
                //Serial.println(inByte);
                if (gblUseFirstExtCmdBuffer)
                {
                    gbl1stExtCMDBuffer[gblExtSerialCmdCounter++] = inByte;
                }
                else
                {
                    gbl2ndExtCMDBuffer[gblExtSerialCmdCounter++] = inByte;
                }

                if (gblExtSerialCmdCounter == inExtLength)
                {
                    //Serial.println("end of packet");
                    gblNewExtCmdReady = 1;
                    gblExtSerialState = SER_WAITING_FOR_1ST_HEADER;
                    gblUseFirstExtCmdBuffer = !gblUseFirstExtCmdBuffer;
                }
            }
        }
    }
}

int newExtCmdPacketReady()
{
    if (!gblNewExtCmdReady)
    {
        return 0;
    }
    else
    {
        return gblUseFirstExtCmdBuffer ? 2 : 1;
    }
}

void clearExtCmdReadyFlag()
{
    gblNewExtCmdReady = 0;
}

//* /////////////////////////////////////////////////////////
//* --- Gesture Task -- keep tracking data from APDS9960 ---
//* /////////////////////////////////////////////////////////

void doStatusLEDCoreStuff(void *parameter)
{
    pinMode(ledPin, OUTPUT);

    for (;;)
    {
        toggleStatusLED();
        delay(500);
    }
}

void setup()
{
    // //* RS485 serial communication
    Serial.begin(115200);

    // //* CC2530 zigbee communication
    // // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
    zbSerial.begin(115200);
    setInterruptHandlerFunction(uart_2, zbSerialEvent); //
    uart_enable_rx_intr(UART_NUM_2);
    delay(10);

    // Serial.begin(2400);
    // Serial.println();
    // Serial.println();
    // Serial.println(F("****************RS485 RTU SDM120*******************"));

    //* start run led status task in core 0
    xTaskCreatePinnedToCore(
        doStatusLEDCoreStuff, /* Function to implement the task */
        "StatusLEDTask",      /* Name of the task */
        1024,                 /* Stack size in words */
        NULL,                 /* Task input parameter */
        1,                    /* Priority of the task */
        &statusLED,           /* Task handle. */
        0);                   /* Core where the task should run */

    delay(100); // wait for gesture task started
}

void loop()
{ //Choose Serial1 or Serial2 as required
    // while (MySerial.available())
    // {
    //     inByte = MySerial.read();
    //     if (inByte == SERIAL_1ST_HEADER)
    //     {
    //         count = 0;
    //     }
    //     Buffer[count] = &inByte;
    //     Serial.write(inByte);
    //     count++;
    //     delay(100);
    // }
    // while (Serial.available())
    // {
    //     inByte_rx = Serial.read();
    //     MySerial.write(inByte_rx);
    //     delay(100);
    // }

    receiveZBPkt();

    while (Serial.available())
    {
        // if(Serial.read() == '1'){
        //     int temp = zbSerial.write("PERMITJOIN 255");
        //     zbSerial.write(temp);
        // }
        // else
        // {
        zbSerial.write(Serial.read());
        /* code */
        // }
    }

    // GET_METER();
    // Serial.println();
    // Serial.print("Total Active Energy = ");
    // Serial.print(DATA_METER[5]);
    // Serial.println(" kWh");
    // delay(5000);
}
