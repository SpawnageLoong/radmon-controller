/* 
 * Constants.h
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

#ifndef Constants_h
#define Constants_h

//**************************************************************************
// MCP2515 Defines
//**************************************************************************

#define CAN_PIN_CS            7       // CS Pin
#define CAN_PIN_INT           1       // INT Pin (INT[10])
#define CAN_REC_ID            0x010   // Frame ID for CAN packets with ZSOM as destination
#define CAN_SEND_ID           0x011   // Frame ID for CAN packets with ZSOM as origin
#define CAN_DATA_FRAME_ID     0x02    // Type identifier for data frames

//**************************************************************************
// ZSOM/FRAM Defines
//**************************************************************************

#define FRAM_CS             5           // CS Pin for ZSOM FRAM
#define KBYTE               1024        // Number of bytes in a kilobyte
#define NUM_BITS_IN_BYTE    8
#define SIZE_OF_INT         sizeof(int) // 4 bytes
#define NUM_OF_BITS_IN_WORD (NUM_BITS_IN_BYTE * SIZE_OF_INT) // 8 * 4 = 32 bits
#define SYS_BITS            NUM_OF_BITS_IN_WORD

#define SPI_MAX_SPEED   33000000     // 33 MHz SCK (max)
#define SPI_DATA_ORDER  MSBFIRST    // MSBFIRST or LSBFIRST
#define SPI_DATA_MODE   SPI_MODE0   // SPI_MODE0 or SPI_MODE1 or SPI_MODE2 or SPI_MODE3

#define FRAM_SIZE                         (32 * KBYTE)
#define FRAM_BASE_ADDRESS                 ( 0 )
#define FRAM_DATA_SIZE                    ( 1 * KBYTE )

//**************************************************************************
// Geiger Counter Defines
//**************************************************************************

#define GM_PIN_INT              4     // INT Pin (INT[11])
#define GM_PIN_OUT              3     // LED Output
#define DATA_TIMER              3     // Timer 3 is used

//**************************************************************************
// I2C Defines
//**************************************************************************

#define SLAVE1_ADDRESS 1
#define SLAVE2_ADDRESS 2

#define DATA_SIZE 1984
#define CHUNK_SIZE 32
#define ARRAY_SIZE 198

#define HANDSHAKE_REQUEST 0x01
#define HANDSHAKE_ACK 0x06
#define TIMEOUT_MS 3000

#define SCL_PIN 12
#define SDA_PIN 11

#define PCYCLE_PIN 6 //to switch on devices, set to low, to switch off set to high
#define TRIGGER_PIN 8
#define LOW_PIN 9

//read result outcomes
#define READ_COMPLETE 1
#define TIMEOUT 2



#endif