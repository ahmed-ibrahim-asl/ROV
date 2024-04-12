
/*************** Include  Section ***************/

// RH_NRF24.h - Library for handling NRF24L01+ radio communication, Name of library Radio Head.
#include <RH_NRF24.h>
#include "SystemConfig.h"
#include <SPI.h>
/************************************************/

/**************** Macros Section ****************/
#define TIMEOUT_PERIOD_MS 7000
#define MESSAGE_LEN 5

#define resolution 8
#define PWM_OPERATING_FREQ 5000
#define PWM_RESOLUTION_BITS 8

#define serialSpeed 9600

// adc macros

#define BATTERY_PIN 15

#define REF_VOLTAGE 3.3

// Maximum value for 12-bit ADC, 2^12 = 4095
#define NO_STEPS_ADC 4095

// here we specify number of repeittion of taking readings of battery
#define NUM_READINGS 30

// Calibration offset added to the averaged voltage
#define CALIBRATION_OFFSET 0.14
//////////////////////////////////////

// Radio module configuration //

// Channel for radio communication
// Specify the channel number between (0-125)
#define RADIO_CHANNEL 3 // Example channel number

// Data rate options for radio communication
// Choose one of the following options based on your requirements:
// RH_NRF24::DataRate2Mbps:   Provides higher data transfer rates but shorter range.
// RH_NRF24::DataRate1Mbps:   Balances data rate and range, suitable for most applications.
// RH_NRF24::DataRate250kbps: Provides longer range at the expense of data transfer speed.
#define RADIO_DATA_RATE RH_NRF24::DataRate2Mbps

// Transmit power options for radio communication
// Choose one of the following options based on your transmission range requirements:
// RH_NRF24::TransmitPower0dBm: Suitable for short-range communication within the same room or building.
// RH_NRF24::TransmitPowerm6dBm: Provides slightly better range than 0dBm.
// RH_NRF24::TransmitPowerm12dBm: Balanced option for moderate range and power consumption.
// RH_NRF24::TransmitPowerm18dBm: Maximizes transmission range, suitable for long-distance communication.
#define RADIO_TRANSMIT_POWER RH_NRF24::TransmitPowerm12dBm
/************************************************/

RH_NRF24 radio(NRF_CE_PIN, NRF_CS_PIN);

// millis helpers
unsigned long currentTime = 0;
unsigned long previousTime = 0;

// Variables
uint8_t batteryPercentage = 0; // Initial battery percentage
double batteryVoltage = 0;     // To store the average voltage, localized to loop

uint8_t rx_buffer[MESSAGE_LEN];
uint8_t tx_buffer[MESSAGE_LEN];
uint8_t msgLength = 0;

void setup()
{

  /************** Initialize  Serial **************/
  Serial.begin(serialSpeed);
  Serial.println("Serial Initialized successfully");
  /************************************************/

  /************ RF Communication Setup ************/
  if (!radio.init())
  {
    Serial.println("radio hardware not responding!");
    // hold program in infinite loop to prevent subsequent errors
    while (1)
    {
    }
  }
  if (!radio.setChannel(RADIO_CHANNEL))
  {
    Serial.println("setChannel failed");
  }
  if (!radio.setRF(RADIO_DATA_RATE, RADIO_TRANSMIT_POWER))
  {
    Serial.println("setRF failed");
  }
  Serial.println("Receiver started");
  /************************************************/

  // PWM setup for motors
  for (short index_interator = 0; index_interator < 4; index_interator++)
  {
    // this function responsible for configuring the PWM hardware
    ledcSetup(ledChannel[index_interator], PWM_OPERATING_FREQ, PWM_RESOLUTION_BITS);
  }

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MOTOR_1_PIN_SPEED, 0);
  ledcAttachPin(MOTOR_2_PIN_SPEED, 1);
  ledcAttachPin(MOTOR_3_PIN_SPEED, 2);
  ledcAttachPin(MOTOR_4_PIN_SPEED, 3);

  // Direction pins setup
  pinMode(MOTOR_1_PIN_DIR, OUTPUT);
  pinMode(MOTOR_2_PIN_DIR, OUTPUT);
  pinMode(MOTOR_3_PIN_DIR, OUTPUT);
  pinMode(MOTOR_4_PIN_DIR, OUTPUT);
}

void loop()
{
  //////////////////////////////////////////////////////////////////////////////////////////
  for (short iterator_index = 0; iterator_index < NUM_READINGS; iterator_index++)
  {
    batteryVoltage += analogRead(BATTERY_PIN) * (REF_VOLTAGE / NO_STEPS_ADC);
  }

  batteryVoltage /= NUM_READINGS;

  batteryPercentage = map(int(batteryVoltage * 100) + CALIBRATION_OFFSET, 97, 116, 0, 100);
  //////////////////////////////////////////////////////////////////////////////////////////

  // Check for incoming radio messages:
  if (radio.recv(rx_buffer, &msgLength))
  {
    //! Sending Acknowledgment
    // Send battery level as response
    uint8_t responseData[] = {batteryPercentage};
    radio.send(responseData, sizeof(responseData));
    radio.waitPacketSent();

    /////////////////////////////////////
    Serial.print("MOTOR1: ");
    Serial.println(rx_buffer[0]);

    Serial.print("MOTOR2: ");
    Serial.println(rx_buffer[1]);

    Serial.print("MOTOR3: ");
    Serial.println(rx_buffer[2]);

    Serial.print("MOTOR4: ");
    Serial.println(rx_buffer[3]);

    Serial.print("Speed: ");
    Serial.println(rx_buffer[4]);
    Serial.println("--- Data retrieved from device ---");

    previousTime = millis();
  }

  else
  {
    currentTime = millis();

    if (currentTime - previousTime > TIMEOUT_PERIOD_MS)
    {
      previousTime = millis();

      memset(rx_buffer, 0, sizeof(rx_buffer));

      Serial.print("MOTOR1: N/A\n");
      Serial.print("MOTOR2: N/A\n");
      Serial.print("MOTOR3: N/A\n");
      Serial.print("MOTOR4: N/A\n");
      Serial.print("Speed:  N/A\n");
      Serial.println("--- NO Connection ---");
    }
  }

  /**************************** Taking  Action Based on Received Message ****************************/
  if (rx_buffer[0] == 1 && rx_buffer[1] == 1 && rx_buffer[2] == 1 && rx_buffer[3] == 1)
  {
    // Forward
    digitalWrite(MOTOR_1_PIN_DIR, HIGH);
    ledcWrite(0, rx_buffer[4] * MOTOR_1_OFFSET);
    digitalWrite(MOTOR_2_PIN_DIR, HIGH);
    ledcWrite(1, rx_buffer[4] * MOTOR_2_OFFSET);
    digitalWrite(MOTOR_3_PIN_DIR, HIGH);
    ledcWrite(2, rx_buffer[4] * MOTOR_3_OFFSET);
    digitalWrite(MOTOR_4_PIN_DIR, HIGH);
    ledcWrite(3, rx_buffer[4] * MOTOR_4_OFFSET);
  }
  else if (rx_buffer[0] == 0 && rx_buffer[1] == 0 && rx_buffer[2] == 0 && rx_buffer[3] == 0)
  {
    // Backward
    digitalWrite(MOTOR_1_PIN_DIR, LOW);
    ledcWrite(0, rx_buffer[4] * MOTOR_1_OFFSET);
    digitalWrite(MOTOR_2_PIN_DIR, LOW);
    ledcWrite(1, rx_buffer[4] * MOTOR_2_OFFSET);
    digitalWrite(MOTOR_3_PIN_DIR, LOW);
    ledcWrite(2, rx_buffer[4] * MOTOR_3_OFFSET);
    digitalWrite(MOTOR_4_PIN_DIR, LOW);
    ledcWrite(3, rx_buffer[4] * MOTOR_4_OFFSET);
  }
  else if (rx_buffer[0] == 0 && rx_buffer[1] == 0 && rx_buffer[2] == 1 && rx_buffer[3] == 1)
  {
    // Right
    digitalWrite(MOTOR_1_PIN_DIR, LOW);
    ledcWrite(0, rx_buffer[4] * MOTOR_1_OFFSET);
    digitalWrite(MOTOR_2_PIN_DIR, HIGH);
    ledcWrite(1, rx_buffer[4] * MOTOR_2_OFFSET);
    digitalWrite(MOTOR_3_PIN_DIR, LOW);
    ledcWrite(2, rx_buffer[4] * MOTOR_3_OFFSET);
    digitalWrite(MOTOR_4_PIN_DIR, HIGH);
    ledcWrite(3, rx_buffer[4] * MOTOR_4_OFFSET);
  }
  else if (rx_buffer[0] == 1 && rx_buffer[1] == 1 && rx_buffer[2] == 0 && rx_buffer[3] == 0)
  {
    // Left
    digitalWrite(MOTOR_1_PIN_DIR, HIGH);
    ledcWrite(0, rx_buffer[4] * MOTOR_1_OFFSET);
    digitalWrite(MOTOR_2_PIN_DIR, LOW);
    ledcWrite(1, rx_buffer[4] * MOTOR_2_OFFSET);
    digitalWrite(MOTOR_3_PIN_DIR, HIGH);
    ledcWrite(2, rx_buffer[4] * MOTOR_3_OFFSET);
    digitalWrite(MOTOR_4_PIN_DIR, LOW);
    ledcWrite(3, rx_buffer[4] * MOTOR_4_OFFSET);
  }
  /**************************************************************************************************/
}