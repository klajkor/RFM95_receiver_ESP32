/*
    LoRa board Wiring:
    ESP32       –  RFM95/96

    GPIO14/D14  – RST
    GPIO05/D05  – NSS (Chip Select)
    GPIO18/D18  – SCK
    GPIO23/D23  – MOSI
    GPIO19/D19  – MISO        
    GPIO02/D02  – DIO0 (INT)
    
    VCC         – 3.3V
    GND         – GND

    SSD1306 I2C OLED display:
    ESP32  - display

    GPIO21 - SDA
    GPIO22 - SCL

*/

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
//#include <Adafruit_SSD1306.h>

#define OLED_I2C_ADDR 0x3C /* OLED module I2C address */
#define OLED_SDA_GPIO 21
#define OLED_SCL_GPIO 22

//Set up OLED display
SSD1306AsciiWire oled_ssd1306_display;

#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 2

#define RF95_FREQ ( 868.0F )

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void GPIO_init(void);
void RFM95_RX_init(void);
void Ssd1306_Oled_Init(void);

void setup()
{
  Serial.begin(115200);
  GPIO_init();
  Ssd1306_Oled_Init();
  RFM95_RX_init();  
}

int16_t packetnum = 0; // packet counter
int16_t Last_RSSI = 0;

void loop()
{
  if (rf95.available())
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    oled_ssd1306_display.clear();
    oled_ssd1306_display.setCol(0);
    oled_ssd1306_display.setRow(0);
    oled_ssd1306_display.println("Incoming msg");

    if (rf95.recv(buf, &len))
    {
      RH_RF95::printBuffer("Received: ", buf, len);
      Last_RSSI = rf95.lastRssi();
      Serial.print("Got: ");
      Serial.println((char *)buf);
      Serial.print("RSSI: ");
      Serial.println(Last_RSSI, DEC);
      
      oled_ssd1306_display.println((char *)buf);
      oled_ssd1306_display.print("RSSI: ");
      oled_ssd1306_display.println(Last_RSSI, DEC);
      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      oled_ssd1306_display.println("Sent a reply");
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    //Serial.println("No incoming msg");
    yield();
  }  
}

void GPIO_init(void)
{
  delay(500);
  pinMode(RFM95_CS, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.println("GPIO init done");
  delay(500);
  yield();
  delay(500);
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
    oled_ssd1306_display.println("LoRa radio init failed");

    while (1)
    {
      yield();
    }
  }

  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ))
  {

    Serial.println("setFrequency failed");
    oled_ssd1306_display.println("setFrequency failed");

    while (1)
    {
      yield();
    }
  }

  Serial.print("Freq: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  rf95.setSpreadingFactor(7);
  rf95.setSignalBandwidth(125E3);
  rf95.setPayloadCRC(false);
  rf95.setPreambleLength(8);
  oled_ssd1306_display.print("Set Freq to: ");
  oled_ssd1306_display.println(RF95_FREQ);
  Serial.println("RFM95 init done");
  oled_ssd1306_display.println("RFM95 init done");
  delay(100);
}

void Ssd1306_Oled_Init(void) {
  Wire.begin();
  oled_ssd1306_display.begin(&Adafruit128x64, OLED_I2C_ADDR);
  oled_ssd1306_display.clear();
  oled_ssd1306_display.setFont(X11fixed7x14);
  oled_ssd1306_display.setRow(0);
  oled_ssd1306_display.println(F("RFM95W"));
  oled_ssd1306_display.println(F("Receiver"));
  Serial.println("OLED init done");
  yield();
  delay(500);
  yield();
  delay(500);
}