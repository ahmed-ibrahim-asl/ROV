/*************** Include  Section ***************/

// Name of library Radio Head
#include <RH_NRF24.h>
#include <WiFi.h>
/************************************************/

/********************************************* Macros Section *********************************************/
#define WIFI_SSID "NRF Wifi"
#define WIFI_PASSWORD "123456789"

#define wifiserver_portNumber 80
#define IDE_TIMEOUT 4000

//! For now don't change that
#define MESSAGE_LEN 5

#define NRF_CS_PIN 4
#define NRF_CE_PIN 5

#define serialSpeed 9600

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

#define ACK_WAIT_TIME 1000
/**********************************************************************************************************/

// WiFiServer server(wifiserver_portNumber);
WiFiServer server = wifiserver_portNumber;

// 5 - CE pin, 4 - CSN pin
RH_NRF24 radio(NRF_CE_PIN, NRF_CS_PIN);

// millis helpers
unsigned long currentTime = 0;
unsigned long previousTime = 0;

uint8_t tx_buffer[MESSAGE_LEN];
String rx_buffer = "";

uint8_t msgLength = 0;
bool movement_status_flag = false;
uint8_t current_speed;

void setup(void)
{
    /************** Initialize  Serial **************/
    Serial.begin(serialSpeed);
    Serial.println("Serial Initialized successfully");
    /************************************************/

    /********** Wifi  Server Configuration **********/
    //! This could be made for mobile application

    // congigure ESP32 Acts as Wifi Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    server.begin();

    Serial.print("Your IP:");
    Serial.println(WiFi.softAPIP());
    Serial.print("Your Gateway:");
    Serial.println(WiFi.localIP());
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
    Serial.println("Transmitter started");
    /************************************************/
}

void loop(void)
{
    WiFiClient newClient = server.available();
    if (!newClient)
    {
        // No client is available, we going to send kill signal to ROV
        currentTime = millis();

        if ((currentTime - previousTime) > IDE_TIMEOUT)
        {
            movement_status_flag = false;
            previousTime = millis();

            for (short uint8_t index = 0; index < MESSAGE_LEN; index++)
            {
                tx_buffer[index] = 0;
            }

            radio.send(tx_buffer, sizeof(tx_buffer));
            radio.waitPacketSent();
        }

        // Maximum buffer size before receiving, recv function would set the actual received message length after.
        msgLength = MESSAGE_LEN;

        if (radio.waitAvailableTimeout(ACK_WAIT_TIME))
        {
            // Waiting to see if the receiver respond with ACK

            if (radio.recv(rx_buffer, &len))
            {
                Serial.print("Got Reply: ");
                Serial.println(rx_buffer[0]);
            }
            else
            {
                Serial.println("Recevied Failed");
            }
        }
        else
        {
            Serial.println("No Reply within timeout.");
        }
        // Early Exit
        return;
    }

    previousTime = millis();
    rx_buffer = client.readStringUntil('\r');
    rx_buffer.remove(0, 5);

    rx_buffer.remove(rx_buffer.length() - 9, 9);
    Serial.print("Received Data: ");
    Serial.println(serial_buffer);

    if (rx_buffer == "Connection")
    {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("");
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.println(String(buf[0]));
        client.println("</html>");
        Serial.print("Battery: ");
        Serial.println(String(buf[0]));
    }
    //////////////////////////////////////////////////////////////////////////
    switch (rx_buffer[0])
    {
    case 'F':
        movement_status_flag = true;
        tx_buffer[0] = 1;
        tx_buffer[1] = 1;
        tx_buffer[2] = 1;
        tx_buffer[3] = 1;
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("");
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.println("Forward");
        client.println("</html>");
        break;

    case 'B':
        movement_status_flag = true;
        tx_buffer[0] = 0;
        tx_buffer[1] = 0;
        tx_buffer[2] = 0;
        tx_buffer[3] = 0;
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("");
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.println("Backward");
        client.println("</html>");
        break;

    case 'L':
        movement_status_flag = true;
        tx_buffer[0] = 1;
        tx_buffer[1] = 1;
        tx_buffer[2] = 0;
        tx_buffer[3] = 0;
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("");
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.println("Left");
        client.println("</html>");
        break;

    case 'R':
        movement_status_flag = true;
        tx_buffer[0] = 0;
        tx_buffer[1] = 0;
        tx_buffer[2] = 1;
        tx_buffer[3] = 1;
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("");
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.println("Right");
        client.println("</html>");
        break;

    case 'S':
        movement_status_flag = false;
        tx_buffer[4] = 0;
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("");
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.println("Stop");
        client.println("</html>");
        break;

    case 'X':
        rx_buffer.remove(0, 1);
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("");
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.println(String(rx_buffer.toInt()));
        client.println("</html>");
        Speed = map(rx_buffer.toInt(), 0, 100, 0, 255);
        delay(20);
        break;

    default:
        // Handle unknown commands
        break;
    }
    //////////////////////////////////////////////////////////////////////////

    if (movement_status_flag)
    {
        tx_buffer[4] = current_speed;
    }
    else
    {
        tx_buffer[4] = 0;
    }

    radio.send(tx_buffer, sizeof(tx_buffer));
    radio.waitPacketSent();
    msgLength = MESSAGE_LEN;

    // waiting for acknowledgment
    if (radio.waitAvailableTimeout(ACK_WAIT_TIME))
    {
        if (radio.recv(rx_buffer, &msgLength))
        {
            Serial.print("got reply: ");
            Serial.println(rx_buffer[0]);
        }
        else
        {
            Serial.println("Recv Failed");
        }
    }

    else
    {
        Serial.println("No Reply.");
    }
}