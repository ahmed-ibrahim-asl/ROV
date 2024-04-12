# Wireless Controlled ROV

This project demonstrates a wireless controlled ROV (Remotely Operated Vehicle) using ESP32 and NRF24L01+ modules. The project is split into two main components: the transmitter and the receiver, which handle WiFi and radio communication respectively to control the movement of the ROV's motors based on the received commands.

## Features

- **WiFi Communication**: Utilizes ESP32 to create a WiFi access point for sending control commands through a web interface.
- **Radio Communication**: Uses NRF24L01+ for robust radio communication to transmit control commands and receive acknowledgments.
- **Motor Control**: Controls four motors with directional and speed adjustments based on received commands.
- **Battery Monitoring**: Includes ADC measurements for real-time battery voltage monitoring and power management.

## Hardware Requirements

- 2x ESP32 Development Board
- 2x NRF24L01+ Radio Module
- Motor Driver (compatible with ESP32 PWM outputs)
- Motors x4
- Power Source (batteries)
- Miscellaneous: wires, breadboard, etc.

## Software Requirements

- Arduino IDE
- ESP32 Board definitions for the Arduino IDE
- RadioHead library for NRF24L01+
