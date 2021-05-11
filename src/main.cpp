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
#include <LoRa.h>

#define OLED_I2C_ADDR 0x3C /* OLED module I2C address */
#define OLED_SDA_GPIO 21
#define OLED_SCL_GPIO 22

//Set up OLED display
SSD1306AsciiWire oled_ssd1306_display;

#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 2

#define LORA_FREQ ( 868E6 ) // For LoRa driver by Sandeep Mistry
#define RF95_FREQ (868.0F)  // For RadioHead RF95 library

/**
 * RadioHead RF95 library use this syncword by default.
 * No member function implemented for changing it.
 * So either you got used to it or start using another library, like Sandeep Mistry's LoRa library.
 * Refer to this brilliant article: https://appcodelabs.com/using-a-software-defined-radio-to-debug-lora-communication-problems
 */
#define LORA_SYNC_WORD ( 0x12 )

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void GPIO_init(void);
void RFM95_RX_init(void);
void Ssd1306_Oled_Init(void);
void RFM95_receiver(void);
void LoRa_init(void);
void LoRa_receiver(void);

void setup()
{
  Serial.begin(115200);
  GPIO_init();
  Ssd1306_Oled_Init();
  RFM95_RX_init();
  //LoRa_init();
}

int16_t packetnum = 0; // packet counter
int16_t Last_RSSI = 0;
int Last_SNR = 0;

void loop()
{
  RFM95_receiver();
  //LoRa_receiver();
  yield();
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
  rf95.setPromiscuous(true);
  oled_ssd1306_display.print("Set Freq to: ");
  oled_ssd1306_display.println(RF95_FREQ);
  Serial.println("RFM95 init done");
  oled_ssd1306_display.println("RFM95 init done");
  delay(100);
}

void Ssd1306_Oled_Init(void)
{
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

void RFM95_receiver(void)
{
  uint8_t msg_headerTo;
  uint8_t msg_headerFrom;
  uint8_t msg_headerId;
  uint8_t msg_headerFlags;
  uint8_t msg_header[5];
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
      //RadioHead library treats the first 4 bytes as part of the header:
      msg_headerTo=rf95.headerTo();
      msg_headerFrom=rf95.headerFrom();
      msg_headerId=rf95.headerId();
      msg_headerFlags=rf95.headerFlags();
      msg_header[0]=msg_headerTo;
      msg_header[1]=msg_headerFrom;
      msg_header[2]=msg_headerId;
      msg_header[3]=msg_headerFlags;
      msg_header[4]=0x00;
      RH_RF95::printBuffer("Received: ", buf, len);
      Last_RSSI = rf95.lastRssi();
      Last_SNR = rf95.lastSNR();
      Serial.print("Got: ");
      Serial.print((char *)msg_header);
      Serial.println((char *)buf);
      Serial.print("RSSI: ");
      Serial.println(Last_RSSI, DEC);
      Serial.print("SNR: ");
      Serial.println(Last_SNR, DEC);

      oled_ssd1306_display.print((char *)msg_header);
      oled_ssd1306_display.println((char *)buf);
      oled_ssd1306_display.print("RSSI: ");
      oled_ssd1306_display.println(Last_RSSI, DEC);
      oled_ssd1306_display.print("SNR: ");
      oled_ssd1306_display.println(Last_SNR, DEC);
      // Send a reply
      //uint8_t data[] = "And hello back to you";
      //rf95.send(data, sizeof(data));
      //rf95.waitPacketSent();
      //Serial.println("Sent a reply");
      //oled_ssd1306_display.println("Sent a reply");
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

void LoRa_init(void)
{
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);

  while (!LoRa.begin(LORA_FREQ))
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  Serial.println("LoRa Initializing OK!");
  oled_ssd1306_display.println("LoRa init done");
  delay(100);
}

void LoRa_receiver(void)
{
  int packetSize;
  packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    oled_ssd1306_display.clear();
    oled_ssd1306_display.setCol(0);
    oled_ssd1306_display.setRow(0);
    // received a packet
    Serial.print("Received packet: ");

    // read packet
    while (LoRa.available())
    {
      String LoRaData = LoRa.readString();
      Serial.println(LoRaData);
      oled_ssd1306_display.println(LoRaData);
    }

    // print RSSI of packet
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    oled_ssd1306_display.println("RSSI: " + String(LoRa.packetRssi()));
    oled_ssd1306_display.println("Snr: " + String(LoRa.packetSnr()));
  }
  yield();
}