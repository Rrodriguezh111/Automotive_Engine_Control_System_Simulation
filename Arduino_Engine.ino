/*
Author: Ramon Rodriguez
Date: 7/23/2024
Description: This Arduino project simulates an automotive engine control system using the CAN (Controller Area Network) protocol.
             The system reads various potentiometer inputs to represent different engine parameters and sends these values 
             over the CAN bus to simulate a real engine's data. The parameters include speed, engine RPM, coolant temperature, 
             fuel level, throttle position, oil pressure, transmission gear, and battery voltage. 
             This simulation can be used to test automotive dashboards or other CAN-enabled devices without needing a real engine.

Libraries: 
- SPI.h: Provides the SPI communication functionality.
- mcp2515_can.h: Provides the CAN communication functionality using the MCP2515 CAN controller.

Hardware:
- Arduino mega2560
- MCP2515 CAN module
- Potentiometers connected to analog pins A0 to A6 to simulate sensor inputs
- Digital pins 22 to 25 to simulate gear inputs
 
Configuration:
 - CAN_2515: Define this macro to use the MCP2515 CAN controller.
 - SEEED_WIO_TERMINAL and CAN_2518FD: Check these definitions to conditionally include necessary libraries and objects for Wio Terminal and MCP2518FD.
 - SPI_CS_PIN and CAN_INT_PIN: Define the SPI Chip Select and CAN interrupt pins.
 
 CAN Messages:
 - Speed: ID 0x18FFF15A, range 0-120 mph
 - Engine RPM: ID 0x18FFF25A, range 0-8000 RPM
 - Coolant Temperature: ID 0x18FFF35A, range -40 to 215°C (with an offset of -40)
 - Fuel Level: ID 0x18FFF45A, range 0-100%
 - Throttle Position: ID 0x18FFF55A, range 0-100%
 - Oil Pressure: ID 0x18FFF65A, range 0-100 psi
 - Transmission Gear: ID 0x18FFF75A, range P, R, N, D
 - Battery Voltage: ID 0x18FFF85A, range 0-24V
 
 Usage:
 - The setup function initializes the CAN bus and configures the input pins.
 - The loop function periodically reads the sensor values, scales them appropriately, 
   formats them into CAN messages, and sends them over the CAN bus.
 - Each parameter has its own send function that reads the corresponding potentiometer value, 
   scales it, formats it, and sends it as a CAN message.
*/
#include <SPI.h>      /*Include the SPI library for communication with the CAN module*/

#include <EEPROM.H>   /*Include the EEPROM Library for persistent value*/

/*Define CAN_2515 to conditionally include the necessary libraries and objects*/
#define CAN_2515

/*Check if SEEED_WIO_TERMINAL and CAN_2518FD are defined*/
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
const int SPI_CS_PIN  = BCM8;  /*Set SPI Chip Select pin for Wio Terminal*/
const int CAN_INT_PIN = BCM25; /*Set CAN interrupt pin for Wio Terminal*/
#else
const int SPI_CS_PIN = 9;  /*Set default SPI Chip Select pin*/
const int CAN_INT_PIN = 2; /*Set default CAN interrupt pin*/
#endif

/*#define UPDATES_PER_SECOND 100*/ /*Define update frequency, commented out*/

#ifdef CAN_2518FD
#include "mcp2518fd_can.h"    /*Include library for MCP2518FD CAN module*/
mcp2518fd CAN(SPI_CS_PIN);    /*Instantiate CAN object with the CS pin*/
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"      /*Include library for MCP2515 CAN module*/
mcp2515_can CAN(SPI_CS_PIN);  /*Instantiate CAN object with the CS pin*/
#endif

int SpeedPotentiometerPin = A0;                   /*Analog pin for Speed Potentiometer*/
int SpeedPotentiometerValue = 0;                  /*Variable to store potentiometer value*/
int EngineRPMPotentiometerPin = A1;               /*Analog pin for Engine RPM Potentiometer*/
int EngineRPMPotentiometerValue = 0;              /*Variable to store potentiometer value*/
int CoolantTemperaturePotentiometerPin = A2;      /*Analog pin for Coolant Temperature Potentiometer*/
int CoolantTemperaturePotentiometerValue = 0;     /*Variable to store potentiometer value*/
int FuelLevelPotentiometerPin = A3;               /*Analog pin for Fuel Level Potentiometer*/
int FuelLevelPotentiometerValue = 0;              /*Variable to store potentiometer value*/
int ThrottlePositionPotentiometerPin = A4;        /*Analog pin for Throttle Position Potentiometer*/
int ThrottlePositionPotentiometerValue = 0;       /*Variable to store potentiometer value*/
int OilPressurePotentiometerPin = A5;             /*Analog pin for Oil Pressure Potentiometer*/
int OilPressurePotentiometerValue = 0;            /*Variable to store potentiometer value*/
int BatteryVoltagePotentiometerPin = A6;          /*Analog pin for Batter Voltage Potentiometer*/
int BatteryVoltagePotentiometerValue = 0;         /*Variable to store potentiometer value*/


int ParkGear = 22;      /*Digital pin for Park Gear*/
int ReverseGear = 23;   /*Digital pin for Reverse Gear*/
int NeutralGear = 24;   /* Digital pin for Neutral Gear*/
int DriveGear = 25;     /*Digital pin for Drive Gear*/

/*Define byte arrays for different CAN messages*/
unsigned char SpeedData[8];              /*Speed Message Byte*/
unsigned char EngineRPMData[8];          /*Engine RPM Message Byte*/
unsigned char CoolantTemperatureData[8]; /*Coolant Temperature Message Byte*/
unsigned char FuelLevelData[8];          /*Fuel Level Message Byte*/
unsigned char ThrottlePositionData[8];   /*Throttle Position Message Byte*/
unsigned char OilPressureData[8];        /*Oil Pressure Message Byte*/
unsigned char TransmissionGearData[8];   /*Transmission Gear Message Byte*/
unsigned char BatteryVoltageData[8];     /*Battery Voltage Message Byte*/ 
unsigned char MileageData[8];            /*Mileage Message Byte*/

/*Define CAN J1939 message IDs*/
long unsigned int Speed = 0x18FFF15A;                 /*Speed Message ID*/
long unsigned int EngineRPM = 0x18FFF25A;             /*Engine RPM Message ID*/
long unsigned int CoolantTemperature = 0x18FFF35A;    /*Coolant Temperature Message ID*/
long unsigned int FuelLevel = 0x18FFF45A;             /*Fuel Level Message ID*/
long unsigned int ThrottlePosition = 0x18FFF55A;      /*Throttle Position Message ID*/
long unsigned int OilPressure = 0x18FFF65A;           /*Oil Pressure Message ID*/
long unsigned int TransmissionGear = 0x18FFF75A;      /*Transmission Gear Message ID*/
long unsigned int BatteryVoltage = 0x18FFF85A;        /*Battery Voltage Message Byte*/


unsigned char len = 0; /*Define length variable*/

unsigned long previousMillisecond = 0;
const long TransmitRate = 100;  /*Interval in milliseconds for transmission*/

void setup() 
{
    SERIAL_PORT_MONITOR.begin(115200);   /*Initialize serial communication at 115200 baud rate*/
    analogReference(DEFAULT);            /*Set the reference voltage for analog inputs*/
    while(!Serial){};                    /*Wait for the serial port to connect*/
    while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_16MHz)) /*Initialize CAN bus with 250 kbps and 16 MHz clock*/
    {             
        SERIAL_PORT_MONITOR.println("CAN init fail, retry..."); /*Print failure message if CAN initialization fails*/
        delay(100); /*Wait 100 ms before retrying*/
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!"); /*Print success message if CAN initializes correctly*/

    pinMode(ParkGear, INPUT_PULLUP);    /*Set Park Gear pin as input with pull-up resistor*/
    pinMode(ReverseGear, INPUT_PULLUP); /*Set Reverse Gear pin as input with pull-up resistor*/
    pinMode(NeutralGear, INPUT_PULLUP); /*Set Neutral Gear pin as input with pull-up resistor*/
    pinMode(DriveGear, INPUT_PULLUP);   /*Set Drive Gear pin as input with pull-up resistor*/
}

void sendSpeedMessage() 
{
    /*Calculate the Speed Potentiometer Value*/
    SpeedPotentiometerValue = analogRead(SpeedPotentiometerPin); /*Read the potentiometer value*/
    float SpeedScaleFactor = 120.0 / 1023; /*Calculate the scale factor*/
    int SpeedOutputValue = SpeedPotentiometerValue * SpeedScaleFactor; /*Scale the potentiometer value*/
    uint16_t SpeedValue = SpeedOutputValue;  /*Store the scaled value as 16-bit*/

    /*Extract lower and upper bytes from the 16-bit value*/
    uint8_t SpeedLowerByte = SpeedValue & 0xFF;       /*Extract lower byte*/
    uint8_t SpeedUpperByte = (SpeedValue >> 8) & 0xFF; /*Extract upper byte*/

    /*Speed Message Byte Message Byte*/
    SpeedData[0] = SpeedLowerByte;
    SpeedData[1] = SpeedUpperByte;
    SpeedData[2] = 0X00; /*Unused byte*/
    SpeedData[3] = 0X00; /*Unused byte*/
    SpeedData[4] = 0X00; /*Unused byte*/
    SpeedData[5] = 0X00; /*Unused byte*/
    SpeedData[6] = 0X00; /*Unused byte*/
    SpeedData[7] = 0X00; /*Unused byte*/

    /*Send the CAN message*/
    CAN.sendMsgBuf(Speed, 1, 8, SpeedData);
}

void sendEngineRPMMessage() 
{
    /*Calculate the Engine RPM Potentiometer Value*/
    EngineRPMPotentiometerValue = analogRead(EngineRPMPotentiometerPin); /*Read the potentiometer value*/
    float EngineRPMScaleFactor = 8000.0 / 1023; /*Calculate the scale factor*/
    int EngineRPMOutputValue = EngineRPMPotentiometerValue * EngineRPMScaleFactor; /*Scale the potentiometer value*/
    uint16_t EngineRPMValue = EngineRPMOutputValue;  /*Store the scaled value as 16-bit*/  

    /*Extract lower and upper bytes from the 16-bit value*/
    uint8_t EngineRPMLowerByte = EngineRPMValue & 0xFF;       /*Extract lower byte*/
    uint8_t EngineRPMUpperByte = (EngineRPMValue >> 8) & 0xFF; /*Extract upper byte*/

    /*Engine RPM Message Byte*/
    EngineRPMData[0] = EngineRPMLowerByte;
    EngineRPMData[1] = EngineRPMUpperByte;
    EngineRPMData[2] = 0X00; /*Unused byte*/
    EngineRPMData[3] = 0X00; /*Unused byte*/
    EngineRPMData[4] = 0X00; /*Unused byte*/
    EngineRPMData[5] = 0X00; /*Unused byte*/
    EngineRPMData[6] = 0X00; /*Unused byte*/
    EngineRPMData[7] = 0X00; /*Unused byte*/

    /*Send the CAN message*/
    CAN.sendMsgBuf(EngineRPM, 1, 8, EngineRPMData);    
}

void sendCoolantTemperatureMessage() 
{
    CoolantTemperaturePotentiometerValue = analogRead(CoolantTemperaturePotentiometerPin); /*Read the potentiometer value*/
    float CoolantTemperatureScaleFactor = 215.0 / 1023; /*Calculate the scale factor*/
    int CoolantTemperatureOutputValue = CoolantTemperaturePotentiometerValue * CoolantTemperatureScaleFactor; /*Scale the potentiometer value*/
    uint16_t CoolantTemperatureValue = CoolantTemperatureOutputValue;  /*Store the scaled value as 16-bit*/ 

    /*Coolant Temperature Message Byte*/
    CoolantTemperatureData[0] = CoolantTemperatureOutputValue + 40;
    CoolantTemperatureData[1] = 0X00; /*Unused byte*/
    CoolantTemperatureData[2] = 0X00; /*Unused byte*/
    CoolantTemperatureData[3] = 0X00; /*Unused byte*/
    CoolantTemperatureData[4] = 0X00; /*Unused byte*/
    CoolantTemperatureData[5] = 0X00; /*Unused byte*/
    CoolantTemperatureData[6] = 0X00; /*Unused byte*/
    CoolantTemperatureData[7] = 0X00; /*Unused byte*/

    /*Send the CAN message*/
    CAN.sendMsgBuf(CoolantTemperature, 1, 8, CoolantTemperatureData);
}

void sendFuelLevelMessage() 
{
    FuelLevelPotentiometerValue = analogRead(FuelLevelPotentiometerPin); /*Read the potentiometer value*/
    float FuelLevelScaleFactor = 100.0 / 1023; /*Calculate the scale factor*/
    int FuelLevelOutputValue = FuelLevelPotentiometerValue * FuelLevelScaleFactor; /*Scale the potentiometer value*/
    uint16_t FuelLevelValue = FuelLevelOutputValue;  /*Store the scaled value as 16-bit*/

    /*Fuel Level Message Byte*/
    FuelLevelData[0] = FuelLevelValue;
    FuelLevelData[1] = 0X00; /*Unused byte*/
    FuelLevelData[2] = 0X00; /*Unused byte*/
    FuelLevelData[3] = 0X00; /*Unused byte*/
    FuelLevelData[4] = 0X00; /*Unused byte*/
    FuelLevelData[5] = 0X00; /*Unused byte*/
    FuelLevelData[6] = 0X00; /*Unused byte*/
    FuelLevelData[7] = 0X00; /*Unused byte*/

    /*Send the CAN message*/
    CAN.sendMsgBuf(FuelLevel, 1, 8, FuelLevelData);
}

void sendTransmissionGearMessage() 
{
    uint8_t ParkGearB = digitalRead(ParkGear);
    uint8_t ReverseGearB = digitalRead(ReverseGear);
    uint8_t NeutralGearB = digitalRead(NeutralGear);
    uint8_t DriveGearB = digitalRead(DriveGear);

    /*Transsmission Gear Message Byte*/
    if (ParkGearB == 0) 
    {
        TransmissionGearData[0] |= (1 << 0);    /*Set bit 0 if ParkGearB is 0*/
    } 
    else 
    {
        TransmissionGearData[0] &= ~(1 << 0);   /*Clear bit 0 otherwise*/
    }

    if (ReverseGearB == 0) 
    {
        TransmissionGearData[0] |= (1 << 1);    /*Set bit 1 if ReverseGearB is 0*/
    } 
    else 
    {
        TransmissionGearData[0] &= ~(1 << 1);   /*Clear bit 1 otherwise*/
    }

    if (NeutralGearB == 0) 
    {
        TransmissionGearData[0] |= (1 << 2);    /*Set bit 0 if NeutralGearB is 0*/
    } 
    else 
    {
        TransmissionGearData[0] &= ~(1 << 2);   /*Clear bit 0 otherwise*/
    }

    if (DriveGearB == 0) 
    {
        TransmissionGearData[0] |= (1 << 3); /*Set bit 1 if DriveGearB is 0*/
    } 
    else 
    {
        TransmissionGearData[0] &= ~(1 << 3);     /*Clear bit 1 otherwise*/
	  } 

    TransmissionGearData[1] = 0X00; /*Unused byte*/
    TransmissionGearData[2] = 0X00; /*Unused byte*/ 
    TransmissionGearData[3] = 0X00; /*Unused byte*/
    TransmissionGearData[4] = 0X00; /*Unused byte*/
    TransmissionGearData[5] = 0X00; /*Unused byte*/
    TransmissionGearData[6] = 0X00; /*Unused byte*/
    TransmissionGearData[7] = 0X00; /*Unused byte*/

    /*Send the CAN message*/
    CAN.sendMsgBuf(TransmissionGear, 1, 8, TransmissionGearData);
}

void sendBatteryVoltage()
{
    /*Calculate the Battery Voltage Potentiometer Value*/
    BatteryVoltagePotentiometerValue = analogRead(BatteryVoltagePotentiometerPin); /*Read the potentiometer value*/
    float BatteryVoltageScaleFactor = 24.0 / 1023; /*Calculate the scale factor*/
    int BatteryVoltageOutputValue = BatteryVoltagePotentiometerValue * BatteryVoltageScaleFactor; /*Scale the potentiometer value*/
    uint16_t BatteryVoltageValue = BatteryVoltageOutputValue;  /*Store the scaled value as 16-bit*/  

    /*Extract lower and upper bytes from the 16-bit value*/
    uint8_t BatteryVoltageLowerByte = BatteryVoltageValue & 0xFF;       /*Extract lower byte*/
    uint8_t BatteryVoltageUpperByte = (BatteryVoltageValue >> 8) & 0xFF; /*Extract upper byte*/

    BatteryVoltageData[0] = BatteryVoltageLowerByte;
    BatteryVoltageData[1] = BatteryVoltageUpperByte;
    BatteryVoltageData[2] = 0x00; /*Unused byte*/
    BatteryVoltageData[3] = 0x00; /*Unused byte*/
    BatteryVoltageData[4] = 0x00; /*Unused byte*/ 
    BatteryVoltageData[5] = 0x00; /*Unused byte*/
    BatteryVoltageData[6] = 0x00; /*Unused byte*/
    BatteryVoltageData[7] = 0x00; /*Unused byte*/

    /*Send the CAN message*/
    CAN.sendMsgBuf(BatteryVoltage, 1, 8, BatteryVoltageData);       
}

void loop() 
{

    unsigned long currentMillisecond = millis();  /*Get the Current time*/

    if (currentMillisecond - previousMillisecond >= TransmitRate)
    {
      previousMillisecond = currentMillisecond;    

      /*Send CAN messages for each parameter*/
      sendSpeedMessage();               /*Send the main motor drive message*/
      sendEngineRPMMessage();           /*Send the control signal message*/
      sendCoolantTemperatureMessage();  /*Send the trolley status message*/
      sendFuelLevelMessage();           /*Send the drive DC link status message*/
      sendTransmissionGearMessage();    /*Send the transmission gear message*/
      sendBatteryVoltage();             /*Send the battery voltage message*/
    }

}
