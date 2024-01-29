#include "custom_uart.h"
#include "buffer.h"
#include "crc.h"
#include "amt22.h"
#include <vector>


static uint8_t payload_buffer_global[PAYLOAD_MAX_LEN] = {0};                // for constructing payload
static uint8_t uart_send_buffer_global[UART_SND_BUFF_MAX_LEN] = {0};        // for constructing uart packet

const byte numChars = UART_SND_BUFF_MAX_LEN;
char uartRcvdChars[numChars];
int uart_rcv_len = -1;
const char cmdpos[] = {0x02, 0x01, 0x0b, 0xb1, 0x6b, 0x03};
const char cmdposStat[] = {0x02, 0x01, 0x0c, 0xc1, 0x8c, 0x03};


// ************** UART Transmit ***************************************
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



// ****************** For assembling the payload for reply **********************************************
void send_all_encoder_pos(std::vector<EncoderAMT22> &encs, bool send_to_uart){
  int32_t ind=0;
  payload_buffer_global[ind++] = GET_ALL_ENC_POS_ID;

  for (const auto &enc: encs){
    buffer_append_uint16(payload_buffer_global, enc.enc_pos, &ind);
  }
  
  if(send_to_uart) uart_send_packet(payload_buffer_global, ind);
}


void send_all_encoder_pos_status(std::vector<EncoderAMT22> &encs, bool send_to_uart){
  int32_t ind=0;
  payload_buffer_global[ind++] = GET_ALL_ENC_POS_STATUS_ID;

  for (const auto &enc: encs){
    buffer_append_uint16(payload_buffer_global, enc.enc_pos, &ind);
    payload_buffer_global[ind++] = (uint8_t)(enc.last_pos_valid);
  }
  
  if(send_to_uart) uart_send_packet(payload_buffer_global, ind);
  
}



// ************** UART Receiver ***************************************
void recvWithStartEndMarkers(bool &flagNewFrameAvail) {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = 0x02;
  char endMarker = 0x03;
  char rc;
  int payload_len = -1;

  while (Serial.available() > 0 && flagNewFrameAvail == false) {
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
        flagNewFrameAvail = true;
        
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
      uartRcvdChars[ndx] = rc;
      ndx++;
    }
  }
}


void parseUartCmd(bool &flagNewFrameAvail, std::vector<EncoderAMT22> &encs) {
  if (flagNewFrameAvail == true) {
    if(checkEqualArrays(cmdpos, uartRcvdChars, sizeof(cmdpos))){
      send_all_encoder_pos(encs);
    } if(checkEqualArrays(cmdposStat, uartRcvdChars, sizeof(cmdposStat))){
      send_all_encoder_pos_status(encs);
    }
    // else{
    //   Serial.println("Wrong data!!");
    // }

    flagNewFrameAvail = false;
    uart_rcv_len = -1;
  }
}

void showNewData(bool &flagNewFrameAvail) {
  
  if (flagNewFrameAvail == true) {
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

    flagNewFrameAvail = false;
    uart_rcv_len = -1;
  }
}





// ****************** utility **********************************************
int checkEqualArrays(const char *ref_arr, const char *buff_arr, int ref_len){
  for(int i=0; i<ref_len; i++){
    if ((unsigned char)(ref_arr[i]) != (unsigned char)(buff_arr[i])){
      return 0;
    }
  }

  return 1;    
}