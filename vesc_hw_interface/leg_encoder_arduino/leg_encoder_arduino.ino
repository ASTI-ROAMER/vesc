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
#include "crc.h"
#include "buffer.h"

/* Serial rates for UART */
#define BAUDRATE        115200

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
#define ENC_0            2
#define ENC_1            3
#define SPI_MOSI        11
#define SPI_MISO        12
#define SPI_SCLK        13


// UART COMMS STUFF
#define START_BYTE_LEN              1
#define PACKET_BYTE_LEN_LEN         1
#define PAYLOAD_MAX_LEN             255
#define CRC_LEN                     2
#define STOP_BYTE_LEN               132
#define UART_SND_BUFF_MAX_LEN       START_BYTE_LEN + PACKET_BYTE_LEN_LEN + PAYLOAD_MAX_LEN + CRC_LEN + STOP_BYTE_LEN

#define UART_SEND_PACKET_AS_ASCIIHEX  0

static uint8_t payload_buffer_global[PAYLOAD_MAX_LEN] = {0};          // for constructing payload
static uint8_t uart_send_buffer_global[UART_SND_BUFF_MAX_LEN];        // for constructing uart packet


// PAYLOAD IDS
#define GET_ALL_ENC_POS_ID              11
#define GET_ALL_ENC_POS_STATUS_ID         12



// Internal storage for positions
// (0)Left bogie, (1)Left rocker, (2)Right bogie, (3)Right rocker
// uint16_t enc_vals[4]={0};

class EncoderAMT22
{
  public:
  EncoderAMT22(int enc_pin_):enc_pos(0), enc_pin(enc_pin_), enc_turns(0), resolution(RES12), last_pos_valid(false){}

  uint16_t enc_pos;
  int16_t enc_turns;
  int enc_pin;
  int resolution;
  bool last_pos_valid=false;
  bool last_turns_valid=false;

  void setCSLine (uint8_t csLine){
    digitalWrite(enc_pin, csLine);
  }

  uint8_t spiWriteRead(uint8_t sendByte, uint8_t releaseLine){
    //holder for the received over SPI
    uint8_t data;

    //set cs low, cs may already be low but there's no issue calling it again except for extra time
    setCSLine(LOW);

    //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
    //We will implement that time delay here, however the arduino is not the fastest device so the delay
    //is likely inherantly there already
    delayMicroseconds(3);

    //send the command  
    data = SPI.transfer(sendByte);
    delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
    setCSLine(releaseLine); //if releaseLine is high set it high else it stays low
    
    return data;
  }


  static bool isValidValue(uint16_t val){
    bool binaryArray[16];
    //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
    for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (val >> (i));

    //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
    if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
            && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      return true;
    }
    else
    {
      return false;
    }

  }

  int getPositionSPI(){
    uint16_t currentPosition;       //16-bit response from encoder

    //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
    currentPosition = spiWriteRead(AMT22_NOP, false) << 8;   

    //this is the time required between bytes as specified in the datasheet.
    //We will implement that time delay here, however the arduino is not the fastest device so the delay
    //is likely inherantly there already
    delayMicroseconds(3);

    //OR the low byte with the currentPosition variable. release line after second byte
    currentPosition |= spiWriteRead(AMT22_NOP, true);        

    //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
    if (isValidValue(currentPosition))
    {
      currentPosition &= 0x3FFF;          // mask out the check bits
      enc_pos = currentPosition >> 2;     // since we are using 12-bit encoder
      last_pos_valid = true;
      return true;
    }
    else
    {
      // Do not update the position
      last_pos_valid = false;
      return false;
    }

  }

  int getPositionMultiTurnsSPI()
  {
    uint16_t pos, uint_turns;

    //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
    pos = spiWriteRead(AMT22_NOP, false) << 8;   

    //this is the time requitcrcred between bytes as specified in the datasheet.
    //We will implement that time delay here, however the arduino is not the fastest device so the delay
    //is likely inherantly there already
    delayMicroseconds(3);

    //OR the low byte with the currentPosition variable. ****NO: release line after second byte****
    pos |= spiWriteRead(AMT22_TURNS, false);        

    // For turns
    delayMicroseconds(3);
    uint_turns = spiWriteRead(AMT22_NOP, false) << 8;
    delayMicroseconds(3);
    uint_turns |= spiWriteRead(AMT22_NOP, true);


    if(isValidValue(pos)){
      //we got back a good position, so just mask away the checkbits
      pos &= 0x3FFF;
      enc_pos = pos >> 2;     // since we are using 12-bit encoder
      last_pos_valid = true;
    } else{
      last_pos_valid = false;
    }

    // pt.turns14 = pt.turns & 0x3FFF;
    if(isValidValue(uint_turns)){
      uint_turns &= 0x3FFF;             // mask out the check bits
      enc_turns = (int16_t)(static_cast<int16_t>(uint_turns) << 2) / 4;     //https://stackoverflow.com/questions/34075922/convert-raw-14-bit-twos-complement-to-signed-16-bit-integer
      last_turns_valid = true;
    } else{
      last_turns_valid = false;
    }

    return last_pos_valid && last_turns_valid ? true : false ;
  }
};




unsigned long time_spi_poll, time2;

void setup() 
{
  //Set the modes for the SPI IO
  //get first byte which is the high byte, shi
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(ENC_0, OUTPUT);
  // pinMode(ENC_1, OUTPUT);
  
  //Initialize the UART serial connection for debugging
  Serial.begin(BAUDRATE);

  //Get the CS line high which is the default inactive state
  digitalWrite(ENC_0, HIGH);
  // digitalWrite(ENC_1, HIGH);

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
  time_spi_poll = millis();
  
}





// CRC16 newcrc(0x1021, 0x0000, 0, false, false);
// char testbuff[]="123456789";


EncoderAMT22 encoders[4] = {EncoderAMT22(ENC_0), EncoderAMT22(3), EncoderAMT22(4), EncoderAMT22(5)} ;


// ************************************************************************************************* pack processing
void uart_send_packet(unsigned char *data, unsigned int len){
  // immitate packet structure of vesc
  
  if (len == 0 || len > PAYLOAD_MAX_LEN) {
		return;
	}

  int b_ind = 0;

  // use only 2 as start bit (only 255 max length)
  uart_send_buffer_global[b_ind++] = 2;
  uart_send_buffer_global[b_ind++] = len;

  memcpy(uart_send_buffer_global + b_ind, data, len);
  b_ind += len;

  // calculate the crc from the data payload
  unsigned short crc = crc16(data, len);
	uart_send_buffer_global[b_ind++] = (uint8_t)(crc >> 8);
	uart_send_buffer_global[b_ind++] = (uint8_t)(crc & 0xFF);
	uart_send_buffer_global[b_ind++] = 3;

  #if UART_SEND_PACKET_AS_ASCIIHEX == 0
    Serial.write(uart_send_buffer_global, b_ind);
  #else 
    Serial.print("0x");
    for(int i=0; i<b_ind; i++){
      if(uart_send_buffer_global[i] < 0x10) Serial.print("0");    // leading 0
      Serial.print(uart_send_buffer_global[i], HEX);
      Serial.print(" ");
    }
    Serial.write(NEWLINE);
  #endif      // UART_SEND_PACKET_AS_ASCIIHEX
  
  // No need ??? to reset uart send buffer.
}


void send_all_encoder_pos(bool send_to_uart=true){
  int32_t ind=0;
  payload_buffer_global[ind++] = GET_ALL_ENC_POS_ID;

  for (int i=0; i<4; i++){
    buffer_append_uint16(payload_buffer_global, encoders[i].enc_pos, &ind);
  }
  
  if(send_to_uart) uart_send_packet(payload_buffer_global, ind);
}

void send_all_encoder_pos_status(bool send_to_uart=true){
  int32_t ind=0;
  payload_buffer_global[ind++] = GET_ALL_ENC_POS_STATUS_ID;

  for (int i=0; i<4; i++){
    buffer_append_uint16(payload_buffer_global, encoders[i].enc_pos, &ind);
    payload_buffer_global[ind++] = (uint8_t)(encoders[i].last_pos_valid);
  }
  
  if(send_to_uart) uart_send_packet(payload_buffer_global, ind);
  
}




// *************************** UART RECEIVER ************
int checkEqualArrays(const char *ref_arr, const char *buff_arr, int ref_len){
  for(int i=0; i<ref_len; i++){
    if ((unsigned char)(ref_arr[i]) != (unsigned char)(buff_arr[i])){
      return 0;
    }
  }

  return 1;    
}

const byte numChars = UART_SND_BUFF_MAX_LEN;
char uartRcvdChars[numChars];
char tempChars[numChars];
boolean newData = false;
int uart_rcv_len = -1;
const char cmdpos[] = {0x02, 0x01, 0x0b, 0xb1, 0x6b, 0x03};
const char cmdposStat[] = {0x02, 0x01, 0x0c, 0xc1, 0x8c, 0x03};


void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = 0x02;
  char endMarker = 0x03;
  char rc;
  int payload_len = -1;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        uartRcvdChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
      }
      else {
        uartRcvdChars[ndx++] = rc; 
        recvInProgress = false;
        uart_rcv_len = ndx;
        ndx = 0;
        newData = true;
        
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
      uartRcvdChars[ndx] = rc;
      ndx++;
    }
  }

}

void showNewData() {
  
  if (newData == true) {
    Serial.println("NEWdata");
    if(checkEqualArrays(cmdpos, uartRcvdChars, sizeof(cmdpos))){
      Serial.print("UART RCV: ");

      // print as hex ascii 
      for(int i=0; i < uart_rcv_len; i++){
        if(uartRcvdChars[i] < 0x10) Serial.print("0");    // leading 0
        Serial.print(uartRcvdChars[i], HEX);
        Serial.print(" ");
      }
      Serial.write(NEWLINE);
    } else{
      Serial.println("Wrong data!");
    }

    newData = false;
    uart_rcv_len = -1;
  }
}

void parseUartCmd() {
  if (newData == true) {
    if(checkEqualArrays(cmdpos, uartRcvdChars, sizeof(cmdpos))){
      send_all_encoder_pos();
    } if(checkEqualArrays(cmdposStat, uartRcvdChars, sizeof(cmdposStat))){
      send_all_encoder_pos_status();
    }
    // else{
    //   Serial.println("Wrong data!!");
    // }

    newData = false;
    uart_rcv_len = -1;
  }
}


#define SPI_POLL_TIME_MS    1 

void loop()
{
  if (millis() - time_spi_poll >= SPI_POLL_TIME_MS){
    // Serial.println("j");
    for (int i=0; i < 4; i++){
      encoders[i].getPositionSPI();
    }
    time_spi_poll = millis();
  }
  recvWithStartEndMarkers();
  parseUartCmd();
}


// uint8_t attempts;
// void loop() 
// {
  


//   //once we enter this loop we will run forever
//   while(1)
//   {
//     //set attemps counter at 0 so we can try again if we get bad position    
    
//     for (int i=0; i < 4; i++){
//       attempts = 0;
//       encoders[i].getPositionMultiTurnsSPI();

//       //if the position returned was 0xFFFF we know that there was an error calculating the checksum
//       //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
//       while (!(encoders[i].last_pos_valid) && ++attempts < 3)
//       {
//         encoders[i].getPositionMultiTurnsSPI();
//       }

//       if (!encoders[i].last_pos_valid) //position is bad, let the user know how many times we tried
//       {
//         Serial.print("Enc#");

//         Serial.print(i, DEC);
//         Serial.print(" error. Attempts: ");
//         Serial.print(attempts, DEC); //print out the number in decimal format. attempts - 1 is used since we post incremented the loop
//         Serial.write(NEWLINE);

//       }
//       else //position was good, print to serial stream
//       {
        
//         Serial.print("Enc");
//         Serial.print(i, DEC);
//         Serial.print(">> ");
//         Serial.print(encoders[i].enc_pos, DEC); //print the position in decimal format

//         Serial.print("    Hex: 0x");
//         Serial.print(encoders[i].enc_pos, HEX);
//         Serial.print(" BIN: ");
//         Serial.print(encoders[i].enc_pos, BIN);
//         Serial.print("  CRC: ");
//         unsigned short tcrc = crc16(reinterpret_cast<unsigned char *>(&(encoders[i].enc_pos)), 2);
//         Serial.print(tcrc, HEX);
//         Serial.write(NEWLINE);

//         // Serial.write(NEWLINE);
//         // Serial.print("TESTBUFF: ");
//         // Serial.print(testbuff);
//         // Serial.print("  CRC: ");
//         // unsigned short tcrc2 = crc16((unsigned char *)testbuff, 9);
//         // Serial.print(tcrc2, HEX);
//         // Serial.print("   expected crc: 0xE5CC");
//         // Serial.write(NEWLINE);

        

//         // newcrc.restart();
//         // newcrc.add((unsigned char *)testbuff, 9);
//         // unsigned short tcrc3 = newcrc.calc();
//         // Serial.print("  LIBcrc(");
//         // Serial.print(newcrc.count(), DEC);
//         // Serial.print("): ");
//         // Serial.print(tcrc3, HEX);
//         // Serial.write(NEWLINE);
//         // Serial << "E" << i << " >> " << encoders[i].enc_pos << ", turns: " << encoders[i].enc_turns <<"\n";
//       }

//     }
//     send_all_encoder_pos();

//     Serial.print("\n>>>>>\n");
//     //For the purpose of this demo we don't need the position returned that quickly so let's wait a half second between reads
//     //delay() is in millisecondsGET_ALL_ENC_POS_ID
//     delay(500);
//   }
// }

