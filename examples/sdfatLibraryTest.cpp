#include <Arduino.h>

#ifndef DISABLE_FS_H_WARNING
#define DISABLE_FS_H_WARNING
#endif

#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

#define SD_FAT_TYPE 3
#define SPI_SPEED SD_SCK_MHZ(4)

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else
#error Invalid SD_FAT_TYPE
#endif

ArduinoOutStream cout(Serial);

static const int PIN_SCK  = 5;
static const int PIN_MISO = 7;
static const int PIN_MOSI = 6;
static const int PIN_CS   = 4;

// Be explicit about the SPI bus object
SPIClass spi;

void cardOrSpeed() {
  cout << F("Try another SD card or reduce the SPI bus speed.\n");
  cout << F("Edit SPI_SPEED in this program to change it.\n");
}

void reformatMsg() {
  cout << F("Try reformatting the card. For best results use\n");
  cout << F("the SdFormatter program in SdFat/examples or download\n");
  cout << F("and use SDFormatter from sdcard.org.\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    yield();
  }

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  cout << F("\nSPI pins:\n");
  cout << F("MISO: ") << int(PIN_MISO) << endl;
  cout << F("MOSI: ") << int(PIN_MOSI) << endl;
  cout << F("SCK:  ") << int(PIN_SCK) << endl;
  cout << F("SS:   ") << int(PIN_CS) << endl;

  // Optional: give the card a moment
  delay(100);
}

bool firstTry = true;

void loop() {
  if (!firstTry) {
    cout << F("\nRestarting\n");
  }
  firstTry = false;

  // Explicit SdFat config using YOUR SPI object
  SdSpiConfig spiConfig(PIN_CS, SHARED_SPI, SPI_SPEED, &spi);

  if (!sd.begin(spiConfig)) {
    cout << F("\nSD initialization failed.\n");

    if (sd.card() && sd.card()->errorCode()) {
      cout << F("Do not reformat the card!\n");
      cout << F("Is the card correctly inserted?\n");
      cout << F("Is chipSelect set to the correct value?\n");
      cout << F("Does another SPI device need to be disabled?\n");
      cout << F("Is there a wiring/soldering problem?\n");

      cout << F("\nerrorCode: ") << hex << showbase << int(sd.card()->errorCode());
      cout << F(", errorData: ") << int(sd.card()->errorData());
      cout << dec << noshowbase << endl;
    } else {
      cout << F("No detailed SdFat card error available.\n");
    }

    delay(10000);
    return;
  }

  cout << F("\nCard successfully initialized.\n\n");

  uint32_t size = sd.card()->sectorCount();
  if (size == 0) {
    cout << F("Can't determine the card size.\n");
    cardOrSpeed();
    while (true) {
      delay(10);
    }
  }

  uint32_t sizeMB = 0.000512 * size + 0.5;
  cout << F("Card size: ") << sizeMB;
  cout << F(" MB (MB = 1,000,000 bytes)\n\n");

  if (sd.fatType() <= 32) {
    cout << F("Volume is FAT") << int(sd.fatType());
  } else {
    cout << F("Volume is exFAT");
  }
  cout << F(", Cluster size (bytes): ") << sd.vol()->bytesPerCluster();
  cout << endl << endl;

  cout << F("Files found (date time size name):\n");
  sd.ls(LS_R | LS_DATE | LS_SIZE);

  cout << F("\nSuccess!\n");

  while (true) {
    delay(10);
  }
}