#include "amt22.h"


EncoderAMT22::EncoderAMT22(int enc_pin_):enc_pos(0), enc_pin(enc_pin_), enc_turns(0), resolution(RES12), last_pos_valid(false){
  pinMode(enc_pin_, OUTPUT);
  //Get the CS line high which is the default inactive state
  digitalWrite(enc_pin_, HIGH);
}

void EncoderAMT22::setCSLine (uint8_t csLine){
  digitalWrite(enc_pin, csLine);
}

uint8_t EncoderAMT22::spiWriteRead(uint8_t sendByte, uint8_t releaseLine){
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


bool EncoderAMT22::isValidValue(uint16_t val){
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

int EncoderAMT22::getPositionSPI(){
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
    // Serial.print("ok:");
    // Serial.print(enc_pos);
    // Serial.print(" raw: ");
    // Serial.println(currentPosition);
    return true;
  }
  else
  {
    // Do not update the position
    last_pos_valid = false;
    return false;
  }

}

int EncoderAMT22::getPositionMultiTurnsSPI()
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


