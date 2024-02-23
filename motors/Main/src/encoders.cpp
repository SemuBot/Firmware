#include "encoders.h"

uint16_t encoderPosition = 0;

void setupEncoders(Encoder* encoders){
  pinMode(ENCODER_SCK, OUTPUT);
  pinMode(ENCODER_SDO, INPUT);
  for (int i = 0; i < NUM_ENCODERS; i++) {
    pinMode(encoders[i].cs_pin,OUTPUT);
  Serial.println("Encoder setup Config finished, starting");
  }
}

void getEncoderPosition(Encoder* encoders){
  for (int i = 0; i < NUM_ENCODERS; i++){
    Serial.print("Encoder ");
    Serial.print(i);
    Serial.print(" value: ");
    Serial.println(encoders[i].data);
  }
}


void encoderUpdate(Encoder* encoders){
  for (int i = 0; i < NUM_ENCODERS; i++) {
    uint8_t attempts; //we can use this for making sure position is valid

    attempts = 0;
    encoders[i].data = getPositionSSI(res12, encoders[i]); 
    while (encoders[i].data == 0xFFFF && attempts++ < 3)
    {
      delay(1);
      encoders[i].data = getPositionSSI(res12, encoders[i]); //try again
    }
    delay(100);
  }
}



/*
 * Use this function to understand how the SSI protocol works first, then you can use the function
 * below that has better efficiency.
 */
uint16_t getPositionSSI(uint8_t resolution, Encoder &encoder)
{
  uint8_t i, j; //we'll use these incrementers
  uint16_t currentPosition;
  uint8_t _clockCounts = resolution + 2; //the AMT23 includes 2 additional bits in the response that are used as checkbits
  bool binaryArray[_clockCounts]; //we'll read each bit one at a time and put in array. SSI comes out reversed so this helps reorder
  bool bitHolder; //this variable holds the current bit in our read loop
  bool checkBit0, checkBit1; //the frist two bits in the position response are checkbits used to check the validity of the position response

  //drop cs low and wait the minimum required time. This is done with NOPs
  digitalWrite(encoder.cs_pin, LOW);
  for (i = 0; i < 5; i++) NOP;

  //We will clock the encoder the number of times (resolution + 2), incrementing with 'j'
  //note that this method of bit-banging doesn't give a reliable clock speed.
  //in applications where datarate is important, the Arduino is not the best solution unless you
  //can find a way to make the SPI interface work for this protocol format.
  for (j = 0; j < _clockCounts; j++)
  {
    //first we lower the clock line and wait until the pin state has fully changed
    digitalWrite(ENCODER_SCK, LOW);
    for (i = 0; i < 10; i++) NOP;

    //now we go high with the clock. no need to wait with NOPs because the pin read we'll do next times sufficient time
    digitalWrite(ENCODER_SCK, HIGH);
    
    //Grab the data off of the SDO line and place it into the binary array
    binaryArray[j] = digitalRead(ENCODER_SDO);
  }
  //release cs line, position has been fully received
  digitalWrite(encoder.cs_pin, HIGH);

  //now we'll reverse the order of the binary array so that the bit ordering matches binary
  for (i = 0, j = _clockCounts - 1; i < (_clockCounts / 2); i++, j--)
  {
    bitHolder = binaryArray[i];
    binaryArray[i] = binaryArray[j];
    binaryArray[j] = bitHolder;
  }

  //create uint16_t from binary array by masking and bit shifting
  for (i = 0; i < _clockCounts - 2; i++) currentPosition |= binaryArray[i] << i;

  //grab check bits in highest bit slots
  checkBit1 = binaryArray[_clockCounts - 1];
  checkBit0 = binaryArray[_clockCounts - 2];

  //use the checkbit equation from the ATM23 datasheet
  if (resolution == res12) //if we're in 12-bit
  {
    if (!(checkBit1 == !(binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
        && (checkBit0 == !(binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      currentPosition = 0xFFFF; //bad pos, return 0xFFFF which is not a valid value
    }
  }
  return currentPosition;
}
