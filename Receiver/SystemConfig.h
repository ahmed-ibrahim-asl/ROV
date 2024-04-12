#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// Motor direction pin definitions
#define MOTOR_1_PIN_DIR 14
#define MOTOR_2_PIN_DIR 22
#define MOTOR_3_PIN_DIR 26
#define MOTOR_4_PIN_DIR 33

// Motor speed control pin definitions
#define MOTOR_1_PIN_SPEED 13
#define MOTOR_2_PIN_SPEED 21
#define MOTOR_3_PIN_SPEED 25
#define MOTOR_4_PIN_SPEED 32

#define MOTOR_1_OFFSET 1
#define MOTOR_2_OFFSET 1
#define MOTOR_3_OFFSET 1
#define MOTOR_4_OFFSET 1

/*
 * MOTOR_1_PIN_DIR  Forward --> HIGH  (4)
 * MOTOR_2_PIN_DIR  Forward --> HIGH  (3)
 * MOTOR_3_PIN_DIR  Forward --> HIGH  (2)
 * MOTOR_4_PIN_DIR  Forward --> HIGH  (1)
 */

// NRF24L01+ Radio pin definitions
#define NRF_CS_PIN 4
#define NRF_CE_PIN 5

// Battery monitoring pin
#define BATTERY_PIN 15

#endif
