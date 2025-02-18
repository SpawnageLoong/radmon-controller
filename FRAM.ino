/* 
 * FRAM.ino
 * 
 * Copyright (C) 2025  Richard Loong
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <SPI.h>
#include "FRAM.h"

//*************************************************************
// Variables
//*************************************************************

int iFramArray[KBYTE/sizeof(int)] = {0};    // 256 * 4byte = 1024bytes = 1KB
int iFramRdArray[KBYTE/sizeof(int)] = {0};

//*************************************************************
// Functions
//*************************************************************

void Clear_FRAM_Data(int iSize)
{
  int temp=0;
  
  Serial.print("\n Start clearing FRAM... \n"); // for debug
  pinMode(FRAM_CS, OUTPUT);
  digitalWrite(FRAM_CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_MAX_SPEED, SPI_DATA_ORDER, SPI_DATA_MODE));

  for(int i=FRAM_BASE_ADDRESS; i<iSize; i++)
  {
    FRAMWrite32( FRAM_CS, i, 0);
    delayMicroseconds(1);
  }
  SPI.endTransaction();

  //check if FRAM is cleared successfully
  Serial.print("\n FRAM check started! \n");
  SPI.beginTransaction(SPISettings(SPI_MAX_SPEED, SPI_DATA_ORDER, SPI_DATA_MODE));

  for(int i=FRAM_BASE_ADDRESS; i<iSize; i++)
  {
    temp = temp + FRAMRead32( FRAM_CS, i);
    delayMicroseconds(1);
  }
  SPI.endTransaction();

  if (temp != 0) Serial.println("\n FRAM is not cleared!");
  else Serial.println("\n FRAM cleared! ");
  //checking FRAM ended

}


/**
 * \brief           Prints a given length of values stored in FRAM starting at a given memory address
 * \param[in]       address: Starting memory address
 * \param[in]       iSize: Length of values (in bytes) to print
 */
void Dump_FRAM_Data(int address, int iSize)
{
  int iInByteCount = 0;
  int i8Data = 0;
  int temp=0;

  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  int endAddress = 0x7FFF;
  if (address + iSize - 1 < endAddress) {
    endAddress = address + iSize - 1;
  }
  
  Serial.println(" Dump_FRAM_Data "); // for debug
  pinMode(FRAM_CS, OUTPUT);
  digitalWrite(FRAM_CS, HIGH);
  SPI.begin();

  SPI.beginTransaction(SPISettings(SPI_MAX_SPEED, SPI_DATA_ORDER, SPI_DATA_MODE));

  for(int i=address; i < endAddress; i++)
  {
    
    if (i%32 == 0)
    {
      Serial.println();
      sprintf(cbuf, " Addr 0x%08X : ", i);
      Serial.print(cbuf);
    }
    
    // read from address
    i8Data <<= 8;
    i8Data |= FRAMRead32( FRAM_CS, i);
    delayMicroseconds(1);
    iInByteCount++;
    // every read() command return 8bits data
    // 4 read means read back 32bits data.
    if (iInByteCount == 4)
    {
        // Reverse the bytes:
        // 32bits data send to SD card is 0x12345678 with 0x78 send first
        // data read back is 0x78563412, so need to reverse it
        int i8Rev=0;
        for (int i = 0; i < 4; i++)
        {
            i8Rev = (i8Rev << 8) | ((i8Data >> (i * 8)) & 0xFF);
        }

        sprintf(cbuf, " 0x%08X", i8Rev);
        Serial.print(cbuf);

        iInByteCount = 0;
        i8Data = 0;
    }
  }

  SPI.endTransaction();

  return;
}


/**
 * \brief           Dumps a given length of data from FRAM to the CAN interface
 * \param[in]       address: Starting memory address
 * \param[in]       iSize: Length of data (in bytes) to dump
 * \details         
 */
 void CAN_Dump_FRAM(uint16_t address, uint16_t iSize)
 {
  uint8_t iInByteCount = 0;
  uint8_t i8Data = 0;

  // CAN Frame data
  uint8_t data[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  // Check that end address stays within 32kB
  uint16_t endAddress = 0x7FFF;
  if (address + iSize - 1 < endAddress) {
    endAddress = address + iSize - 1;
  }
  
  // Prepare SPI for FRAM read
  pinMode(FRAM_CS, OUTPUT);
  digitalWrite(FRAM_CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_MAX_SPEED, SPI_DATA_ORDER, SPI_DATA_MODE));

  for(uint16_t i = address; i <= endAddress; i++)
  {
    // Read from address
    i8Data = FRAMReadByte(FRAM_CS, i);
    delayMicroseconds(1);
    iInByteCount++;

    // Save data starting from the end of the CAN frame
    // Words are reveresed when saving, so write from the end to un-reverse it
    data[sizeof(data) - iInByteCount] = i8Data;
    if (iInByteCount == 4)
    {
      // Split starting address into two 8-bit values
      uint16_t startAddress = i - 3;
      uint16_t addressL = startAddress & 0x00FF;
      uint16_t addressH = (startAddress & 0xFF00) >> 8;
      data[1] = addressH;
      data[2] = addressL;
      CAN.beginPacket(CAN_SEND_ID);
      CAN.write(data, sizeof(data));
      CAN.endPacket();
      iInByteCount = 0;
      for (uint8_t j = 1; j < 8; j++)
      {
        data[j] = 0;
      }
    }
    i8Data = 0;
    if (i == 0x7FFE) {
      break;
    }
  }

  SPI.endTransaction();

  return;
 }


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int FRAMRead32(int CS_pin, int address, int N_BYTE) {

  unsigned int temp_data[N_BYTE]={0};
  unsigned int result = 0;

  // take the SS pin low to select the chip:
  digitalWrite(CS_pin, LOW); 
  delayMicroseconds(1);

  //  send in the address and value via SPI:
  SPI.transfer(FRAM_READ); //op-code:Read Memory Code
  SPI.transfer(((address & 0x7F00) >> 8));
  SPI.transfer((address & 0x00FF));
  
  for(int i=0; i< N_BYTE; i++){

    temp_data[i] = SPI.transfer(0x00);

  }

  // take the SS pin high to de-select the chip:
  delayMicroseconds(1);
  digitalWrite(CS_pin, HIGH); 

  //calculate 8/16/24/32 bit result
  result = (((temp_data[3]) << 24) & (N_BYTE >= 4 ? 0xFFFFFFFF : 0x00000000)) | (((temp_data[2]) << 16) & (N_BYTE >= 3 ? 0xFFFFFFFF : 0x00000000)) | (((temp_data[1]) << 8) & (N_BYTE >= 2 ? 0xFFFFFFFF : 0x00000000)) | temp_data[0];  

  return (result);
  
}


/**
 * \brief           Reads the byte at the given address from FRAM
 * \param[in]       CS_pin: Chip select pin for FRAM SPI
 * \param[in]       address: Address of the byte to be read
 * \param[out]      result: The value read from FRAM
 */
uint8_t FRAMReadByte(int CS_pin, int address) {

  uint8_t temp_data = 0;
  uint8_t result = 0;

  // take the SS pin low to select the chip:
  digitalWrite(CS_pin, LOW); 
  delayMicroseconds(1);

  //  send in the address and value via SPI:
  SPI.transfer(FRAM_READ); //op-code:Read Memory Code
  SPI.transfer(((address & 0x7F00) >> 8));
  SPI.transfer((address & 0x00FF));

  result = SPI.transfer(0x00);

  // take the SS pin high to de-select the chip:
  delayMicroseconds(1);
  digitalWrite(CS_pin, HIGH); 

  //calculate 8/16/24/32 bit result
  //result = (((temp_data[3]) << 24) & (N_BYTE >= 4 ? 0xFFFFFFFF : 0x00000000)) | (((temp_data[2]) << 16) & (N_BYTE >= 3 ? 0xFFFFFFFF : 0x00000000)) | (((temp_data[1]) << 8) & (N_BYTE >= 2 ? 0xFFFFFFFF : 0x00000000)) | temp_data[0];  

  return (result);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void FRAMWrite32(int CS_pin, int address, int value, int N_BYTE) {
  unsigned int temp_data [N_BYTE];

  for (int i=0; i<N_BYTE; i++){
    temp_data [i] = (value) >> (i*8) & 0xFF;
  }

  // take the CS pin low to select the chip:
  digitalWrite(CS_pin, LOW); 
  delayMicroseconds(1);

  //  send in the address and value via SPI:
  SPI.transfer(FRAM_WREN); //op-code:Set Write Enable Latch
  delayMicroseconds(1);
  digitalWrite(CS_pin, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS_pin, LOW); 
  SPI.transfer(FRAM_WRITE); //op-code:Write Memory Code
  SPI.transfer(((address & 0x7F00) >> 8));
  SPI.transfer((address & 0x00FF));

  for(int i = 0; i<N_BYTE; i++){
    SPI.transfer(temp_data [i]);
  }
 
  // take the SS pin high to de-select the chip:
  delayMicroseconds(1);
  digitalWrite(CS_pin, HIGH); 

  return;  
}


/**
 * \brief           
 * \param[in]       iSize: 
 */
void Initialize_FRAM(int iSize, int iEvenWord, int iOddWord)
{
    sprintf(cbuf, "    Initialize_FRAM();  Size: %04dbyte, EvenWords: 0x%08X, OddWords: 0x%08X \n", iSize*sizeof(int), iEvenWord, iOddWord);
    Serial.print(cbuf);

    for (int i=0; i<iSize; i++)
    {
      if (i % 2 == 0) { iFramArray[i] = iEvenWord; }
      else { iFramArray[i] = iOddWord; }
    }

    Store_Data_To_FRAM(FRAM_BASE_ADDRESS, iFramArray, sizeof(iFramArray));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Note: A word is 4 bytes
void Custom_Data_Init_FRAM(int iSize)
{
    sprintf(cbuf, "Custom_Data_Init_FRAM();  Size: %04dbyte \n", iSize*sizeof(int));
    Serial.print(cbuf);

    int epochWord       = 0x672B11D0;
    int gmCountWord     = 0x00000064;
    int errorCountsWord = 0x00040004;
    int dataWord0       = 0x10101010;
    int dataWord1       = 0x0F0F0F0F;

    // 512 Bytes of data is 128 words
    int dataSize        = 128;

    // 1 Frame is 131 words
    for (int i=0; i<iSize; i++)
    {
      iFramArray[i]   = epochWord;
      iFramArray[i+1] = gmCountWord;
      iFramArray[i+2] = errorCountsWord;
      for (int j=3; j<67; j++)
      {
        iFramArray[i+j] = dataWord0;
      }
      for (int j=67; j<131; j++)
      {
        iFramArray[i+j] = dataWord1;
      }
    }

    Store_Data_To_FRAM(FRAM_BASE_ADDRESS, iFramArray, sizeof(iFramArray));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Read_Data_From_FRAM(int iAddress, int *iArray, int iSize)
{
  int iInByteCount = 0;
  int i8Data = 0;

  Serial.print("    -- Read_Data_From_FRAM --");
  Serial.print(" >>  Address : 0x");
  Serial.print(iAddress,HEX);
  Serial.print(" >>  Size : ");
  Serial.println(iSize);

  pinMode(FRAM_CS, OUTPUT);
  digitalWrite(FRAM_CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_MAX_SPEED, SPI_DATA_ORDER, SPI_DATA_MODE));  

  // read back stored PARI_R[NUM_ROWS_IN_1KB]; 
  for(int i=0; i < iSize; i++)
  {
    // read from address
    int iAddr = iAddress + i;
    i8Data <<= 8;
    i8Data |= FRAMRead32( FRAM_CS, iAddr);
    delayMicroseconds(1);
    iInByteCount++;
    // every read() command return 8bits data
    // 4 read means read back 32bits data.
    if (iInByteCount == 4)
    {
        // Reverse the bytes:
        // 32bits data send to SD card is 0x12345678 with 0x78 send first
        // data read back is 0x78563412, so need to reverse it
        int i8Rev=0;
        for (int i = 0; i < 4; i++)
        {
            i8Rev = (i8Rev << 8) | ((i8Data >> (i * 8)) & 0xFF);
        }
        iArray[i/4] = i8Rev;
        iInByteCount = 0;
        i8Data = 0;
    }
  }
  SPI.endTransaction();
}


/**
 * \brief           Write an array of values with known length to a memory address in FRAM
 * \param[in]       iAddress: Memory address to write to
 * \param[in]       iArray: Array of values
 * \param[in]       iSize: Length of array
 * \details         
 */
void Store_Data_To_FRAM(int iAddress, int *iArray, int iSize)
{
  int iData;
  int error = 0;

  Serial.print("    -- Store_Data_To_FRAM --");
  Serial.print(" >> Address : 0x");
  Serial.print(iAddress,HEX); 
  Serial.print(" >> Size : ");
  Serial.println(iSize);

  pinMode(FRAM_CS, OUTPUT);
  digitalWrite(FRAM_CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_MAX_SPEED, SPI_DATA_ORDER, SPI_DATA_MODE));    

  // iArray contain 4bytes (32bits) of data
  for (int i = 0; i < (iSize/sizeof(int)); i++)
  {
    for (int k=0; k<4; k++)
    {
      // for 32bits data 0x12345678, send out will be 0x78, 0x56, 0x34, 0x12
      iData = (iArray[i] >> (k * 8)) & 0xFF;
      // write address
      int iAddr = iAddress + (i * 4) + k ;

      FRAMWrite32( FRAM_CS, iAddr, iData);
      delayMicroseconds(1);
    }
  }

  SPI.endTransaction();
}


