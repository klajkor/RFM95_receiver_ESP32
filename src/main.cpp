/*
    Wiring:
    ESP32       –  RFM95/96

    Gpio14/D14  – RST
    Gpio05/D05  – NSS (Chip Select)
    Gpio18/D18  – SCK
    Gpio23/D23  – MOSI
    Gpio19/D19  – MISO        
    Gpio02/D02  – DIO0 (INT)
    
    VCC         – 3.3V
    GND         – GND

*/

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 2

#define RF95_FREQ 868.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void GPIO_init(void);
void RFM95_RX_init(void);

void setup()
{
  Serial.begin(115200);
  GPIO_init();
  RFM95_RX_init();
}

int16_t packetnum = 0; // packet counter

void loop()
{
  if (rf95.available())
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char *)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");      
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}

void GPIO_init(void)
{
  delay(1000);
  pinMode(RFM95_CS, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.println("GPIO init done");
  delay(1000);
  yield();
  delay(2000);
}

void RFM95_RX_init(void)
{
  Serial.println();

  Serial.println("Gateway Module starting…");

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init())
  {

    Serial.println("LoRa radio init failed");

    while (1)
    {
      yield();
    }
  }

  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ))
  {

    Serial.println("setFrequency failed");

    while (1)
    {
      yield();
    }
  }

  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}
