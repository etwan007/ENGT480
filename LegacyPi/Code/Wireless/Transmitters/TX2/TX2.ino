//This program will transmit ID number and raw encoder value over ESPnow to a master ESP32
//Import Packages

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

// ===== settings =====
#define ESPNOW_CHANNEL      6 //Set ESP32 comms channel            
#define SEND_HZ             200 //send rate of 200 encoder values per second
#define SEND_PERIOD_US      (1000000UL / SEND_HZ) //find T

#define MT6701_ADDR         0x06 //Base address for reading encoder vals
#define MT6701_ANGLE_MSB    0x03 //Set start reading angle from encoder

#define SENDER_ID           1 //id for each encoder

//Master (receiver) MAC:
uint8_t MASTER_MAC[6] = {0xFC, 0xB4, 0x67, 0xF6, 0x50, 0xF4};

// ===== packet: minimal for motor driving and receiving =====
//just raw angle (plus sender id so master can place it)
typedef struct __attribute__((packed)) {
  uint8_t  sender_id;     // 0..2
  uint16_t raw14;         // 0..16383, or 0xFFFF if bad
} AnglePacket;

//set ESP32 comms channel
static inline void setChannel(uint8_t ch) {
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

//2-byte  i2c reg read from the MT6701 to convert 2 byte read to a 14-bit angle meas
//returns TRUE if successful and FALSE if anything fails
static bool mt6701_readAngle14(uint16_t &angle14) {
  //set read register
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(MT6701_ANGLE_MSB);

  //end the write without releasing the serial bus
  if (Wire.endTransmission(false) != 0) return false;  //repeated start

//read 2 bytes from the address
  int n = Wire.requestFrom(MT6701_ADDR, 2u);
  //return false if a read fail occured 
  if (n != 2) return false;

//top 8 bits of the encoder
  uint8_t msb = Wire.read(); // [13:6]
//bottom 8 bits of the encoder
  uint8_t lsb = Wire.read(); // [5:0] in bits [7:2]
//14 bit value is constructed from the msb and lsb
  angle14 = ((uint16_t)msb << 6) | (lsb >> 2);
  //keep only 14 bits from the combined 2bytes(16bits)
  angle14 &= 0x3FFF;
  return true;
}

//holds the next scheduled timestamp for the next packet send
static uint32_t nextSendUs = 0;

void setup() {
  //begin serial
  Serial.begin(115200);
  delay(200);
//begin i2c
  Wire.begin();
  Wire.setClock(400000);

//begin espnow
  WiFi.mode(WIFI_STA);
  setChannel(ESPNOW_CHANNEL);
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (esp_now_init() != ESP_OK) {
    while (true) delay(1000);
  }

//add the receiver as a peer
  esp_now_peer_info_t peer = {};
  //sets masters MAC
  memcpy(peer.peer_addr, MASTER_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    while (true) delay(1000);
  }
//begin send scheduler as now
  nextSendUs = micros();
}

void loop() {
  //gives current timestamp 
  uint32_t now = micros();
  //is it time to send packet?
  if ((int32_t)(now - nextSendUs) >= 0) {
    //schedule next send time based off of T
    nextSendUs += SEND_PERIOD_US;

//2byte transmit value - This value is considered a BAD read
    uint16_t a = 0xFFFF;
    
    uint16_t angle14 = 0;
    //if i2c read is a success set a the the read 14 bit value
    if (mt6701_readAngle14(angle14)) a = angle14;
    //setup packet
    AnglePacket p;
    p.sender_id = (uint8_t)SENDER_ID;
    p.raw14     = a;
    //sets up the esp packet send 
    
    esp_now_send(MASTER_MAC, (uint8_t*)&p, sizeof(p));
  }
}