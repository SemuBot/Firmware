#include "encoders.h"

uint16_t encoderPosition = 0;

void encoderSetup(){
    pinMode(SSI_SCK, OUTPUT);
    pinMode(SSI_CS, OUTPUT);
    pinMode(SSI_SDO, INPUT);
}

uint16_t getEncoderPosition() {
    return encoderPosition;
}

void encoderUpdate(){
    uint8_t attempts; //we can use this for making sure position is valid
    //set attemps counter at 0 so we can try again if we get bad position    
    attempts = 0;
    encoderPosition = getPositionSSI(res12); 
    while (encoderPosition == 0xFFFF && attempts++ < 3)
    {
      delay(1);
      encoderPosition = getPositionSSI(res12); //try again
    }
    delay(500);
}



/*
 * Use this function to understand how the SSI protocol works first, then you can use the function
 * below that has better efficiency.
 */
uint16_t getPositionSSI(uint8_t resolution)
{
  uint8_t i, j; //we'll use these incrementers
  uint16_t currentPosition;
  uint8_t _clockCounts = resolution + 2; //the AMT23 includes 2 additional bits in the response that are used as checkbits
  bool binaryArray[_clockCounts]; //we'll read each bit one at a time and put in array. SSI comes out reversed so this helps reorder
  bool bitHolder; //this variable holds the current bit in our read loop
  bool checkBit0, checkBit1; //the frist two bits in the position response are checkbits used to check the validity of the position response

  //drop cs low and wait the minimum required time. This is done with NOPs
  digitalWrite(SSI_CS, LOW);
  for (i = 0; i < 5; i++) NOP;

  //We will clock the encoder the number of times (resolution + 2), incrementing with 'j'
  //note that this method of bit-banging doesn't give a reliable clock speed.
  //in applications where datarate is important, the Arduino is not the best solution unless you
  //can find a way to make the SPI interface work for this protocol format.
  for (j = 0; j < _clockCounts; j++)
  {
    //first we lower the clock line and wait until the pin state has fully changed
    digitalWrite(SSI_SCK, LOW);
    for (i = 0; i < 10; i++) NOP;

    //now we go high with the clock. no need to wait with NOPs because the pin read we'll do next times sufficient time
    digitalWrite(SSI_SCK, HIGH);
    
    //Grab the data off of the SDO line and place it into the binary array
    binaryArray[j] = digitalRead(SSI_SDO);
  }
  //release cs line, position has been fully received
  digitalWrite(SSI_CS, HIGH);

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

/*
 * This function is a more efficient version of the getPositionSSI() function
 * that strips out some of the bit reordering and improves the speed of calculating the checkbit.
 * Once the SSI protocol is fully understood, this function will be more useful in an end application.
 */
uint16_t getPositionSSI_efficient(uint8_t resolution)
{
  uint8_t i, j; //we'll use these incrementers
  uint8_t odd, even; //bit parity counters
  uint16_t currentPosition = 0;
  uint8_t _clockCounts = resolution + 2; //the AMT23 includes 2 additional bits in the response that are used as checkbits
  uint8_t checkBit1, checkBit0; //the frist two bits in the position response are checkbits used to check the validity of the position response

  //drop cs low and wait the minimum required time. This is done with NOPs
  digitalWrite(SSI_CS, LOW);
  for (i = 0; i < 5; i++) NOP;

  //We will clock the encoder the number of times (resolution + 2), incrementing with 'j'
  //note that this method of bit-banging doesn't give a reliable clock speed.
  //in applications where datarate is important, the Arduino is not the best solution unless you
  //can find a way to make the SPI interface work for this protocol format.
  for (j = 0; j < _clockCounts; j++)
  {
    //first we lower the clock line and wait until the pin state has fully changed
    digitalWrite(SSI_SCK, LOW);
    for (i = 0; i < 10; i++) NOP;

    //now we go high with the clock. no need to wait with NOPs because the pin read we'll do next times sufficient time
    digitalWrite(SSI_SCK, HIGH);
    
    //throw the pin value into the position, note that it's reversing it as well
    currentPosition |= (digitalRead(SSI_SDO) << (_clockCounts - j - 1));
  }
  //release cs line, position has been fully received
  digitalWrite(SSI_CS, HIGH);


  //grab the highest two bits and put them into the checkbit holders
  checkBit1 = (currentPosition >> (_clockCounts - 1)) & 0x01;
  checkBit0 = (currentPosition >> (_clockCounts - 2)) & 0x01;

  //at this point currentPosition still holds the checkbits. So if we're in 14 bit mode, there's 16 bits
  //we only move up 14 bits here (resolution) because we're only tallying up the 1's in the position
  //we're counting the bits in even slots and the ones in odd slots
  for (uint8_t i = 0; i < resolution; i++) (i % 2 == 0) ? even += ((currentPosition >> i) & 0x01) : odd += ((currentPosition >> i) & 0x01);

  //check the counts against the checkbits
  if ((checkBit1 == odd % 2) || (checkBit0 == even % 2)) currentPosition = 0xFFFF;
  else 
  {
    //this isn't the 'fastest' since we're introducting an if/else but doing
    // currentPosition &= 2^resolution; doesn't work because arduino has a problem with
    // powers.
    if (resolution == res12) currentPosition &= 0xFFF;
  }

  return currentPosition;
}
