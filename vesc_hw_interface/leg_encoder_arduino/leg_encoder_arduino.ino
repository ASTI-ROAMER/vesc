/*
 * AMT22_SPI_Sample_Code.ino
 * Company: CUI Inc.
 * Author: Jason Kelly
 * Version: 2.0.1.0
 * Date: August 20, 2019
 * 
 * This sample code can be used with the Arduino Uno to control the AMT22 encoder.
 * It uses SPI to control the encoder and the the Arduino UART to report back to the PC
 * via the Arduino Serial Monitor.
 * For more information or assistance contact CUI Inc for support.
 * 
 * After uploading cHexode to Arduino Uno open the open the Serial Monitor under the Tools 
 * menu and set the baud rate to 115200 to view the serial stream the position from the AMT22.
 * 
 * Arduino Pin Connections
 * SPI Chip Select Enc 0:   Pin  2
 * SPI Chip Select Enc 1:   Pin  3
 * SPI MOSI                 Pin 11
 * SPI MISO                 Pin 12
 * SPI SCLK:                Pin 13
 * 
 * 
 * AMT22 Pin Connections
 * Vdd (5V):                Pin  1
 * SPI SCLK:                Pin  2
 * SPI MOSI:                Pin  3
 * GND:                     Pin  4
 * SPI MISO:                Pin  5
 * SPI Chip Select:         Pin  6
 * 
 * 
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 * 
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KINDwrite,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */


/* Include the SPI library for the arduino boards */
#include <SPI.h>
#include <vector>
#include "amt22.h"
#include "custom_uart.h"

// Encoder poll time
#define SPI_POLL_TIME_MS    1 

/* Serial rates for UART */
#define BAUDRATE        115200

/* SPI CS pins */
#define ENC_0            2
#define ENC_1            3
#define ENC_2            4


const int ENC_ARR_SIZE = 3;
std::vector<EncoderAMT22> encoders(ENC_ARR_SIZE);       // declaration only here, init should be inside setup ()
bool flagNewFrameAvail=false;
unsigned long time_spi_poll, time2;



void setup() 
{
  //Set the modes for the SPI IO
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  // The EncoderAMT22 will handle initializing the CS pins
  
  //Initialize the UART serial connection for debugging
  Serial.begin(BAUDRATE);

  //set the clockrate. Uno clock rate is 16Mhz, divider of 32 gives 500 kHz.
  //500 kHz is a good speed for our test environment
  //SPI.setClockDivider(SPI_CLOCK_DIV2);   // 8 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV4);   // 4 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV8);   // 2 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV64);  // 250 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV128); // 125 kHz
  
  //start SPI bus
  SPI.begin();
  
  // initialize objects here
  encoders[0] = EncoderAMT22(ENC_0);
  encoders[1] = EncoderAMT22(ENC_1);
  encoders[2] = EncoderAMT22(ENC_2);

  time_spi_poll = millis();
  time2 = millis();
  
}



void loop()
{
  if (millis() - time_spi_poll >= SPI_POLL_TIME_MS){
    // Serial.println("j");
    for (int i=0; i < ENC_ARR_SIZE ; i++){
      encoders[i].getPositionSPI();
      // delayMicroseconds(40);               // USEFUL IF multiple encoders slots/object refers to the same encoder pin 
    }
    time_spi_poll = millis();
  } 
  // else if (millis() - time2 >= 1000){
  //   send_all_encoder_pos_status(encoders);
  //   for(int i=0; i < ENC_ARR_SIZE; i++){
  //     Serial.print(encoders[i].enc_pos);
  //     Serial.print(", ");
  //   }
  //   Serial.write(NEWLINE);
  //   time2 = millis();
  // }

  recvWithStartEndMarkers(flagNewFrameAvail);
  parseUartCmd(flagNewFrameAvail, encoders);

}


