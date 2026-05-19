#include <Arduino.h>
#include <SPI.h>

static const int PIN_SCK  = 5;   // change if needed
static const int PIN_MISO = 7;   // change if needed
static const int PIN_MOSI = 6;   // change if needed
static const int PIN_CS   = 4;   // change if needed

SPIClass spi;

uint8_t sdCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
  // Command frame: [0x40|cmd][arg31:24][arg23:16][arg15:8][arg7:0][crc]
  spi.transfer(0x40 | cmd);
  spi.transfer((arg >> 24) & 0xFF);
  spi.transfer((arg >> 16) & 0xFF);
  spi.transfer((arg >> 8) & 0xFF);
  spi.transfer(arg & 0xFF);
  spi.transfer(crc);

  // SD card may return 0xFF for a while before a valid R1 response
  for (int i = 0; i < 16; i++) {
    uint8_t r = spi.transfer(0xFF);
    if (r != 0xFF) return r;
  }
  return 0xFF;
}

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(10);
  }
  delay(1000);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // Start slow for init
  SPISettings sdInitSettings(250000, MSBFIRST, SPI_MODE0);

  Serial.println("Sending idle clocks...");
  spi.beginTransaction(sdInitSettings);
  digitalWrite(PIN_CS, HIGH);

  // >= 74 clocks with MOSI high -> send 10 bytes of 0xFF = 80 clocks
  for (int i = 0; i < 10; i++) {
    spi.transfer(0xFF);
  }

  digitalWrite(PIN_CS, LOW);

  // CMD0 requires valid CRC in SPI mode during init: 0x95
  uint8_t r = sdCommand(0, 0x00000000, 0x95);

  digitalWrite(PIN_CS, HIGH);
  spi.transfer(0xFF); // extra clocks after CS high
  spi.endTransaction();

  Serial.print("CMD0 response = 0x");
  Serial.println(r, HEX);

  if (r == 0x01) {
    Serial.println("SD card is reachable: entered SPI idle state.");
  } else if (r == 0xFF) {
    Serial.println("No response: wiring, CS, MISO, power, or bus mismatch likely.");
  } else {
    Serial.println("Card responded, but not with expected idle response.");
  }
}

void loop() {
}