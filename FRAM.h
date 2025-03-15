/* 
 * FRAM.h
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

#ifndef FRAM_h
#define FRAM_h

//*************************************************************
// FRAM OP-CODE
//*************************************************************
#define FRAM_WREN  0x06 // 0000 0110b // Set Write Enable Latch
#define FRAM_WRDI  0x04 // 0000 0100b // Reset Write Enable Latch
#define FRAM_RDSR  0x05 // 0000 0101b // Read Status Register
#define FRAM_WRSR  0x01 // 0000 0001b // Write Status Register
#define FRAM_READ  0x03 // 0000 0011b // Read Memory Code
#define FRAM_WRITE 0x02 // 0000 0010b // Write Memory Code
#define FRAM_RDID  0x9F // 1001 1111b // Read Device ID
#define FRAM_SLEEP 0x06 // 1011 1001b // Sleep Mode


//*************************************************************
// Function declaration
//*************************************************************
void Clear_FRAM_Data(int iSize);
void Fill_FRAM_Data(int iSize);
void Dump_FRAM_Data(int iSize);
void Dump_FRAM_Data_CAN(int iSize);
unsigned int FRAMRead32(int CS_pin, int address, int N_BYTE=1);
uint8_t FRAMReadByte(int CS_pin, int address);
void FRAMWrite32(int CS_pin, int address, int value, int N_BYTE=1);
void Initialize_FRAM(int iSize, int iEvenWord=0xFEDCBA98, int iOddWord=0x12345678);
void Custom_Data_Init_FRAM(int iSize);
void Read_Data_From_FRAM(int iAddress, int *iArray, int iSize);
void Store_Data_To_FRAM(int iAddress, int *iArray, int iSize);



#endif