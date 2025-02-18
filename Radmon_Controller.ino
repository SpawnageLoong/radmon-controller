/* 
 * Radmon_Controller.ino
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

#include "Constants.h"
#include "FRAM.h"
#include <CAN.h>
#include "Adafruit_ZeroTimer.h"
#include <RTCZero.h>
#include <Wire.h>

//**************************************************************************
// Compile Flags
//**************************************************************************
#define DEBUG 1
//#define SERIAL_MODE 1
#define CAN_MODE 1
//#define DEBUG_CAN 1
//#define GM_COUNTER 1
//#define I2C_SLAVES 1
//#define TIMER 1
//#define RTC_ON 1


//**************************************************************************
// Global Variables
//**************************************************************************

// IO
char cbuf[100];
uint8_t iInputChar;
uint8_t data[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t err[] = { 0xEE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// FRAM
volatile int rollingAddress = 0x0000;
volatile uint32_t gmTubeCount = 0;

// Timer
#ifdef TIMER
  Adafruit_ZeroTimer dataTimer = Adafruit_ZeroTimer(DATA_TIMER);
#endif

// RTC
#ifdef RTC_ON
  RTCZero rtc;
#endif

// I2C
# ifdef I2C_SLAVES
  int clearBusCounter = 0;
  int handshake1 = 0;
  int handshake2 = 0;
  int errorCount1 = 0;
  int errorCount2 = 0;
  int readResult1 = 0;
  int readResult2 = 0;
  volatile uint8_t slave1_SRAM[ARRAY_SIZE];
  volatile uint8_t slave2_SRAM[ARRAY_SIZE];
#endif

// test vars
volatile bool togglepin = false;


//**************************************************************************
// Timer Config
//**************************************************************************
#ifdef TIMER
  void TC3_Handler() {
    Adafruit_ZeroTimer::timerHandler(3);
  }
  #define DATA_TIMER_DIVIDER     1024
  #define DATA_TIMER_PRESCALER   TC_CLOCK_PRESCALER_DIV1024
  #define DATA_TIMER_COMPARE     (48000000/1024)
#endif


//**************************************************************************
// Function Declarations
//**************************************************************************

void canCallback(int packetSize);
void gmTubeISR();
void dataTimerCallback();
void clearBus();
void powerCycle();
bool performHandshake(int slaveAddress);
void prettyPrint(uint8_t c, int bytesReceived);
int byteErrorCounter(uint8_t c);
void printerReadSram(long int time_taken, int total_error_count);
int readSram(int slaveAddress);

//**************************************************************************
// Setup
//**************************************************************************

void setup() {
  // Serial
  #ifdef DEBUG
    Serial.begin(9600);
    while (! Serial){}
    Serial.println("\n\n Serial init OK");
  #endif

  // CAN
  #ifdef CAN_MODE
    CAN.setPins(CAN_PIN_CS, CAN_PIN_INT);
    //CAN.setClockFrequency(8E6);
    CAN.filter(CAN_REC_ID);
    while (!CAN.begin(500E3)){
      #ifdef DEBUG
        Serial.println("Starting CAN failed!");
      #endif
      delay(1000);
    }
    CAN.onReceive(canCallback);
    #ifdef DEBUG
      Serial.println("CAN init OK");
    #endif
  #endif

  // FRAM
  //pinMode(FRAM_CS,  OUTPUT);
  //digitalWrite(FRAM_CS, HIGH);

  // Geiger Counter
  #ifdef GM_COUNTER
    pinMode(GM_PIN_INT, INPUT_PULLUP);
    pinMode(GM_PIN_OUT, OUTPUT);
  #endif

  // Timer
  #ifdef TIMER
    dataTimer.enable(false);
    dataTimer.configure(DATA_TIMER_PRESCALER,  // prescaler
          TC_COUNTER_SIZE_16BIT,                // bit width of timer/counter
          TC_WAVE_GENERATION_MATCH_PWM          // frequency or PWM mode
    );
    dataTimer.setCompare(0, DATA_TIMER_COMPARE);
    dataTimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, dataTimerCallback);
    dataTimer.enable(true);
  #endif

  // RTC
  #ifdef RTC_ON
    rtc.begin();
  #endif

  // I2C
  #ifdef I2C_SLAVES
    Wire.begin(); // Start I2C as master
    pinMode(SCL_PIN, INPUT_PULLUP); //for open drain
    pinMode(SDA_PIN, INPUT_PULLUP); //for open drain
    pinMode(PCYCLE_PIN, OUTPUT);
    digitalWrite(PCYCLE_PIN, LOW); //set low to switch on the atmegas
    pinMode(LOW_PIN, OUTPUT);
    digitalWrite(LOW_PIN, LOW); //set pin forever low 
    pinMode(TRIGGER_PIN, INPUT_PULLUP); //set pin high unless external drives it low.
  #endif

  // Init success
  #ifdef DEBUG
    Serial.println("Init end");
  #endif

  // Attach interrupts
  #ifdef GM_COUNTER
    attachInterrupt(digitalPinToInterrupt(GM_PIN_INT), gmTubeISR, FALLING);
  #endif

  // test setup
  //pinMode(LED_BUILTIN, OUTPUT);
}

//**************************************************************************
// Main loop
//**************************************************************************

void loop() {
  #ifdef I2C_SLAVES
    handshake1 = 0;
    handshake2 = 0;
    errorCount1 = 0;
    errorCount2 = 0;
    readResult1 = 0;
    readResult2 = 0;
    
    if (digitalRead(TRIGGER_PIN)){//no trigger do nothing
      #ifdef DEBUG
        Serial.println("Read Not Triggered");
      #endif
      clearBusCounter = 0;
      delay(100); //poll ever 100ms
    }

    else{ //triggered, time to get SRAM data
      unsigned long start_loop_time = millis();
      #ifdef DEBUG
        Serial.println("Reading the 2 SRAMs");
      #endif  

      #ifdef DEBUG
        Serial.println("Requesting Handshake from Slave 1");
      #endif
  
      handshake1 = performHandshake(SLAVE1_ADDRESS); //handshake with slave 1

      if (handshake1){ //if handshake 1 is successful
        #ifdef DEBUG
          Serial.println("Handshake with Slave1 Successful");
          Serial.println("Reading SRAM of Slave 1");
        #endif
        readResult1 = readSram(SLAVE1_ADDRESS); //read sram data
      }

      if ((!handshake1) || (readResult1 != READ_COMPLETE)){ //if no handshake or no read
        #ifdef DEBUG
          Serial.println("Need to clear the bus due to slave 1");
        #endif

        if (clearBusCounter >= 1) { //1 clear bus failed
          clearBusCounter = 0;
          powerCycle();
        }
        else{
          clearBus();
          clearBusCounter++;
        }
        //then just let the loop() just run again
      }

      else { //if read1 was done
        #ifdef DEBUG
          Serial.println("Requesting Handshake from Slave 2");
        #endif
  
        handshake2 = performHandshake(SLAVE2_ADDRESS);//try handshake2  
        if (handshake2){ //read SRAM2 
          #ifdef DEBUG
            Serial.println("Handshake with Slave 2 Successful");
            Serial.println("Reading SRAM of Slave 2");
          #endif
          readResult2 = readSram(SLAVE2_ADDRESS); //read sram data
        }

        if ((!handshake2) || (readResult2 != READ_COMPLETE)){ //if either handshake or read wasn't successful.
          #ifdef DEBUG
            Serial.print("Need to clear the bus due to slave 2");
          #endif

          if (clearBusCounter >= 1) { //1 clear bus failed
            clearBusCounter = 0;
            powerCycle();
          }
          else{
            clearBus();
            clearBusCounter++;
          }
        }

        else { //if read2 was successful
          #ifdef DEBUG
            Serial.print("Error count for SRAM1: ");
            Serial.println(errorCount1);
            Serial.print("Error count for SRAM2: ");
            Serial.println(errorCount2);
            Serial.println("Reading of 2 SRAMs successfully completed");
          #endif
        }
      }
      #ifdef DEBUG
        Serial.print("Time since read trigger is ");
        Serial.println(millis()-start_loop_time);
      #endif
      delay(5000); // Wait before attempting SRAM read again
    }//trigger_else
  #endif
} //loop end

//**************************************************************************
// Functions
//**************************************************************************

#ifdef CAN_MODE
  /**
  * \brief           Callback function to handle received CAN packets
  * \details         
  */
  void canCallback(int packetSize) {
    #ifdef DEBUG_CAN
      Serial.print("Received ");
      while (CAN.available()) {
        Serial.println((char)CAN.read(), HEX);
      }
    #else
      uint8_t packetDlc = CAN.packetDlc();
      uint8_t in_arr[packetDlc];
      CAN.readBytes(in_arr, packetDlc);
      #ifdef DEBUG
        sprintf(cbuf, "Frame data: %X", in_arr[0]);
        for (int i=1; i<packetDlc; i++) {
          sprintf(cbuf + strlen(cbuf), ", %X", in_arr[i]);
        }
        sprintf(cbuf + strlen(cbuf), "\n");
        Serial.print(cbuf);
      #endif
      iInputChar = in_arr[0];
      uint16_t paramA = 0;
      uint16_t paramB = 0;
      uint32_t ts;
      switch(iInputChar) {
        case 0x01: // Dump full FRAM data; 32 KBYTE; from 0x0000_0000 to 0x0000_7FFF
          #ifdef DEBUG
            Serial.print("dumping 32kB");
          #endif
          CAN_Dump_FRAM(0x0000, 0x8000);
          break;
        
        case 0x02: // Dump selected section of FRAM
          paramA = ((uint16_t)in_arr[1] << 8) | (uint16_t)in_arr[2];
          paramB = ((uint16_t)in_arr[3] << 8) | (uint16_t)in_arr[4];
          #ifdef DEBUG
            sprintf(cbuf, "Dumping %i bytes to CAN", paramB);
            Serial.print(cbuf);
          #endif
          CAN_Dump_FRAM(paramA, paramB);
          break;

        case 0xAA: // Update RTC
          #ifdef RTC_ON
            ts = ((uint32_t)in_arr[1] << 24 | (uint32_t)in_arr[2] << 16 | (uint32_t)in_arr[3] << 8 | (uint32_t)in_arr[4]);
            rtc.setEpoch(ts);
            #ifdef DEBUG
              sprintf(cbuf, "Updated RTC to %i", ts);
              Serial.print(cbuf);
            #endif
          #endif
          break;

        default: // Unknown command
          #ifdef DEBUG
            Serial.println("Unknown command");
          #endif
          err[1] = 0x01;
          CAN.beginPacket(CAN_SEND_ID);
          CAN.write(err, 8);
          CAN.endPacket();
          break;
      }
      iInputChar = NULL;
    #endif
  }
#endif


#ifdef GM_COUNTER
  /**
  * \brief           Callback function to handle signals from the Geiger-Muller Tube
  * \details         Increments gmTubeCount when triggered.
  */
  void gmTubeISR() {
    gmTubeCount++;  // Increment count each time a falling edge is detected
  }
#endif

#ifdef GM_COUNTER
  /**
  * \brief           Callback function to handle dataTimer compare interrupts
  * \details         Updates FRAM and resets current GM Tube count
  */
  void dataTimerCallback(void) {
    #ifdef RTC_ON
      uint32_t now = rtc.getEpoch();
    #else
      uint32_t now = 0;
    #endif
    uint16_t errCountA = 0;
    uint16_t errCountB = 0;
    uint32_t errCountMerged = ((uint32_t)errCountA << 16 | (uint32_t)errCountB);
    int fixedFrame[] = { now, gmTubeCount, errCountMerged };
    //uint32_t testDataA[64];
    //uint32_t testDataB[64];
    Store_Data_To_FRAM(rollingAddress, fixedFrame, sizeof(fixedFrame));
    rollingAddress += 32; // temp value, 8 bytes
    digitalWrite(LED_BUILTIN, togglepin);
    togglepin = !togglepin;
    gmTubeCount = 0;
  }
#endif


#ifdef I2C_SLAVES
  /**
  * \brief           Clears the I2C bus after a lockup
  * \details         
  */
  void clearBus() { 
    pinMode(SCL_PIN, INPUT_PULLUP);
    pinMode(SDA_PIN, INPUT_PULLUP);

    if (digitalRead(SDA_PIN) == LOW) { // Check if SDA is held low
      #ifdef DEBUG
        Serial.println("Bus lockup detected. Clearing...");
      #endif

      for (int i = 0; i < 9; i++) { // Toggle SCL to clear lockup
        pinMode(SCL_PIN, OUTPUT);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(5);
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(5);
      }

      // Release SCL and SDA lines after toggling
      pinMode(SCL_PIN, INPUT_PULLUP);
      pinMode(SDA_PIN, INPUT_PULLUP);

      // Delay before resuming to allow the bus to stabilize
      delay(100);
    }
    #ifdef DEBUG
      Serial.println("Bus clearing was attempted");
    #endif
  }


  /**
  * \brief           Triggers a power cycle for the ATMegas via PCYCLE_PIN
  * \details         
  */
  void powerCycle(){
    #ifdef DEBUG
      Serial.println("Power Cycling Now");
    #endif
    digitalWrite(PCYCLE_PIN, HIGH);
    delay(3000);
    digitalWrite(PCYCLE_PIN, LOW);
    delay(3000);
    #ifdef DEBUG
      Serial.println("Power Cycling Complete");
    #endif
  }


  /**
  * \brief           Performs a handshake with a slave ATMega
  * \details         
  */
  bool performHandshake(int slaveAddress) {
    unsigned long startTime = millis();

      Wire.beginTransmission(slaveAddress);
      Wire.write(HANDSHAKE_REQUEST); // Send handshake request
      Wire.endTransmission();
      Wire.requestFrom(slaveAddress, 1); // Request 1 byte for handshake acknowledgment
      delay(100);

    // Wait for acknowledgment within timeout
    while (millis() - startTime < TIMEOUT_MS) {
      if(Wire.available()){
      
        int ack = Wire.read();
    
        if (ack == HANDSHAKE_ACK) {
          return true; // Handshake successful
        }
      }
      delay(10); // Small delay before retrying
    }
    #ifdef DEBUG
      Serial.println("Handshake failed");
    #endif
    return false; // Handshake failed
  }


  /**
  * \brief
  * \details         
  */
  void prettyPrint(uint8_t c, int bytesReceived){
    if (bytesReceived%64==0){Serial.println();}
    if (c<16){Serial.print(0);}
    Serial.print(c, HEX); // Print each received byte to Serial Monitor
    Serial.print(" ");
  }


  /**
  * \brief
  * \details         
  */
  int byteErrorCounter(uint8_t c){
    int err_count = 0;
    uint8_t reference_byte = 0x55;
    uint8_t mask = 0x01;
    //do a bitwise xor of reference with c. -> the matching bit locations will be zero, the unmatched locations = 1, count the number of 1's in the byte
    uint8_t xored_byte = reference_byte ^ c; //8'b 0100 1000
    for (int i = 0; i<8; i++){
      if ((xored_byte & mask) == 1) {err_count++;} 
      xored_byte = xored_byte >> 1;
    }
    return err_count;
  }


  /**
  * \brief
  * \details         
  */
  void printerReadSram(long int time_taken, int total_error_count){
    Serial.println();
    Serial.print("Time taken for 1 SRAM dump, to compare and store ");
    Serial.print(time_taken);
    Serial.println(" ms");
    Serial.println("\nData reception complete.");
    Serial.print("Error Count: ");
    Serial.println(total_error_count);
    Serial.print("Now printing stores 10% array");
    for (int i=0; i<ARRAY_SIZE; i++){
      if (i%64 == 0){Serial.println();}
      Serial.print(slave1_SRAM[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }


  /**
  * \brief
  * \details         
  */
  int readSram(int slaveAddress){
    //read from slave and append every 10th byte to array_slave + print on screen
    int bytesReceived = 0;
    int slave_array_index = 0;
    int total_error_count = 0;
    int byte_error_count= 0;

    long int start_time = millis();

    while ((bytesReceived < DATA_SIZE) && (millis()-start_time < TIMEOUT_MS)){
      Wire.requestFrom(slaveAddress, CHUNK_SIZE); // Request CHUNK_SIZE bytes//keep requesting in chunks of 32 from slave.

      while (Wire.available() && (millis()-start_time < TIMEOUT_MS)) {
        uint8_t c = Wire.read(); // Read a byte
        #ifdef DEBUG
          prettyPrint(c, bytesReceived); //print it
        #endif

        if (bytesReceived >= 219 && bytesReceived <= 2219){
          byte_error_count = byteErrorCounter(c); //count number of errors
          total_error_count = total_error_count + byte_error_count; //increment error count

          if ((bytesReceived%10 == 0) && (slave_array_index < ARRAY_SIZE)){ //append to array
            if (slaveAddress == SLAVE1_ADDRESS){slave1_SRAM[slave_array_index] = c;}
            if (slaveAddress == SLAVE2_ADDRESS){slave2_SRAM[slave_array_index] = c;} 
            slave_array_index++;
          }
        }
        bytesReceived++;

        if (bytesReceived >= DATA_SIZE) {
          if (slaveAddress == SLAVE1_ADDRESS){errorCount1 = total_error_count;} //set total error count
          if (slaveAddress == SLAVE2_ADDRESS){errorCount2 = total_error_count;}
          #ifdef DEBUG
            printerReadSram(millis()-start_time, total_error_count);
          #endif
          return READ_COMPLETE; //return
        }
      }
    }

    //else if timeout
    #ifdef DEBUG
      Serial.println();
      Serial.println("Timeout: Possible bus lockup detected.");
    #endif
    return TIMEOUT;
  }
#endif