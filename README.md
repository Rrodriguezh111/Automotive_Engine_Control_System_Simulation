# Automotive Engine Control System Simulation

## Description

This Arduino project simulates an automotive engine control system using the CAN (Controller Area Network) protocol. It reads various potentiometer inputs to represent different engine parameters and sends these values over the CAN bus to simulate real engine data. The parameters include:

- Speed
- Engine RPM
- Coolant Temperature
- Fuel Level
- Throttle Position
- Oil Pressure
- Transmission Gear
- Battery Voltage

This simulation is useful for testing automotive dashboards or other CAN-enabled devices without requiring an actual engine.

## Libraries

- **SPI.h:** Provides SPI communication functionality.
- **mcp2515_can.h:** Provides CAN communication functionality using the MCP2515 CAN controller.

## Hardware

- **Arduino Mega 2560**
- **MCP2515 CAN Module**
- **Potentiometers:** Connected to analog pins A0 to A6 to simulate sensor inputs.
- **Digital Pins:** Pins 22 to 25 simulate gear inputs.

## Configuration

- **CAN_2515:** Define this macro to use the MCP2515 CAN controller.
- **SEEED_WIO_TERMINAL and CAN_2518FD:** Check these definitions to conditionally include necessary libraries and objects for the Wio Terminal and MCP2518FD.
- **SPI_CS_PIN and CAN_INT_PIN:** Define the SPI Chip Select and CAN interrupt pins according to your hardware setup.

## CAN Messages

- **Speed:** 
  - **ID:** `0x18FFF15A`
  - **Range:** 0-120 mph

- **Engine RPM:** 
  - **ID:** `0x18FFF25A`
  - **Range:** 0-8000 RPM

- **Coolant Temperature:** 
  - **ID:** `0x18FFF35A`
  - **Range:** -40 to 215°C (with an offset of -40)

- **Fuel Level:** 
  - **ID:** `0x18FFF45A`
  - **Range:** 0-100%

- **Throttle Position:** 
  - **ID:** `0x18FFF55A`
  - **Range:** 0-100%

- **Oil Pressure:** 
  - **ID:** `0x18FFF65A`
  - **Range:** 0-100 psi

- **Transmission Gear:** 
  - **ID:** `0x18FFF75A`
  - **Range:** Park (P), Reverse (R), Neutral (N), Drive (D)

- **Battery Voltage:** 
  - **ID:** `0x18FFF85A`
  - **Range:** 0-24V

## Usage

1. **Setup:**
   - Initializes the CAN bus and configures the input pins.

2. **Loop:**
   - Periodically reads the sensor values from the potentiometers.
   - Scales the values appropriately.
   - Formats the values into CAN messages.
   - Sends the CAN messages over the CAN bus.

3. **Functions:**
   - Each parameter has a dedicated function that reads the corresponding potentiometer value, scales it, formats it, and sends it as a CAN message.
