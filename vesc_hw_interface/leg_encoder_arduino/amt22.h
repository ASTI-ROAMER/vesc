#ifndef AMT22_H_
#define AMT22_H_

#include <SPI.h>

/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70
#define AMT22_TURNS     0xA0

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14

/* SPI pins */
#define SPI_MOSI        11
#define SPI_MISO        12
#define SPI_SCLK        13


class EncoderAMT22
{
  public:
  EncoderAMT22(){};
  EncoderAMT22(int);

  ~EncoderAMT22(){};

  uint16_t enc_pos;
  int16_t enc_turns;
  int enc_pin;
  int resolution;
  bool last_pos_valid=false;
  bool last_turns_valid=false;

  void setCSLine (uint8_t);
  uint8_t spiWriteRead(uint8_t, uint8_t);
  static bool isValidValue(uint16_t val);
  int getPositionSPI();
  int getPositionMultiTurnsSPI();
};


#endif /* AMT22_H_ */