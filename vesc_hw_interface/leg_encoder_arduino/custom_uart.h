#ifndef CUSTOM_UART_H_
#define CUSTOM_UART_H_


// UART COMMS STUFF
#define START_BYTE_LEN              1
#define PACKET_BYTE_LEN_LEN         1
#define PAYLOAD_MAX_LEN             255
#define CRC_LEN                     2
#define STOP_BYTE_LEN               132
#define UART_SND_BUFF_MAX_LEN       START_BYTE_LEN + PACKET_BYTE_LEN_LEN + PAYLOAD_MAX_LEN + CRC_LEN + STOP_BYTE_LEN

#define UART_SEND_PACKET_AS_ASCIIHEX  0       // !!!!!! SET TO 1 TO SEND PACKETS AS ASCII !!!!!!!


// PAYLOAD IDS
#define GET_ALL_ENC_POS_ID                11
#define GET_ALL_ENC_POS_STATUS_ID         12

// For sending a specific length packet to uart
void uart_send_packet(unsigned char *data, unsigned int len);

void send_all_encoder_pos(std::vector<EncoderAMT22> &encs, bool send_to_uart=true);
void send_all_encoder_pos_status(std::vector<EncoderAMT22> &encs, bool send_to_uart=true);

// Reads values from UART and stores it, sets the given flag to true if the it encounters start and end markers
void recvWithStartEndMarkers(bool &flagNewFrameAvail) ;

// Checks if the given flag if there is data to be parsed and matches it to an action
void parseUartCmd(bool &flagNewFrameAvail, std::vector<EncoderAMT22> &encs);

// For debugging, reprints what we received
void showNewData(bool &flagNewFrameAvail);

// Compares 2 arrays if they are equal, up to the length
int checkEqualArrays(const char *ref_arr, const char *buff_arr, int ref_len);

#endif /* CUSTOM_UART_H_ */