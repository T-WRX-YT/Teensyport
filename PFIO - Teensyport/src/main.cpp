#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "SPI.h"
#include "ILI9341_t3n.h"

// yo dawg i heard you like global variables

/* CAN BUS SETUP */
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

unsigned char responseData[61]; // holds the entire response, 61 bytes is the max i am working with
uint8_t packetCount = 0;  // how many packets parsed in this block
uint8_t byteCount = 0;  // how many bytes written, this will include any empty bytes filling the last packet, counts as the index of the responseData array
uint8_t responseBytes;  // the value in the 0x10 packet that the ECU says is how many bytes its sending
bool flowCont = 1;  // this starts at 1, but if the 0x30 packet isn't all zeroes it will go to 0 then ssm active will turn off

//unsigned char fineKnockData[4]; // 6 gauge fine kock is 2 lines
float feedbackKnockFinal;  // holds the calculated value for current feedback knock
float feedbackMax = 0;  // holds the max negative value seen for feedback knock
float fineKnockFinal;  // holds the calculated value for the current fine knock correction
float fineMax = 0;  // holds the max negative value seen for fine knock
uint16_t fineRpmMin = 9999;  // this is the smallest RPM value seen when there is fine knock correction, starts at 9999
uint16_t fineRpmMax = 0;  // this is the higest RPM value seen when there is fine knock, combine both to see what range you've got fine knock (its good enough)
float boostFinal;  // holds the calculated boost value
int16_t coolantFinal;  // holds the calculated coolant temperature
float damFinal; // holds the calculated DAM value, if this isn't 1... youve got problems
int16_t intakeTempFinal;  // holds the calculated value of the intake air temp
uint16_t rpmFinal;  // holds the calculated value of engine speed aka RPM

// this is the standard 6 gauge non logging setup request.  this is only valid for my ECU ID yours might be different
const unsigned char req0[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char req1[8] = {0x10, 0x35, 0xA8, 0x00, 0xFF, 0x7D, 0xA0, 0xFF};
const unsigned char req2[8] = {0x21, 0x7D, 0xA1, 0xFF, 0x7D, 0xA2, 0xFF, 0x7D};
const unsigned char req3[8] = {0x22, 0xA3, 0xFF, 0x7E, 0x3C, 0xFF, 0x7E, 0x3D};
const unsigned char req4[8] = {0x23, 0xFF, 0x7E, 0x3E, 0xFF, 0x7E, 0x3F, 0xFF};
const unsigned char req5[8] = {0x24, 0x62, 0x00, 0xFF, 0x62, 0x01, 0xFF, 0x62};
const unsigned char req6[8] = {0x25, 0x02, 0xFF, 0x62, 0x03, 0x00, 0x00, 0x0E};
const unsigned char req7[8] = {0x26, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x08, 0xFF};
const unsigned char req8[8] = {0x27, 0x68, 0x5E, 0x00, 0x00, 0x12, 0x00, 0x00};

// this is the big request used when im data logging, based on my ECU ID yours might be different
unsigned char longReq1[8] = {0x10, 0xB9, 0xA8, 0x00, 0xFF, 0x63, 0xE4, 0xFF};
unsigned char longReq2[8] = {0x21, 0x63, 0xE5, 0xFF, 0x63, 0xE6, 0xFF, 0x63};
unsigned char longReq3[8] = {0x22, 0xE7, 0xFF, 0x62, 0x00, 0xFF, 0x62, 0x01};
unsigned char longReq4[8] = {0x23, 0xFF, 0x62, 0x02, 0xFF, 0x62, 0x03, 0xFF};
unsigned char longReq5[8] = {0x24, 0x8D, 0x30, 0xFF, 0x8D, 0x31, 0xFF, 0x8D};
unsigned char longReq6[8] = {0x25, 0x32, 0xFF, 0x8D, 0x33, 0xFF, 0x1F, 0x00};
unsigned char longReq7[8] = {0x26, 0xFF, 0x1F, 0x01, 0xFF, 0x1F, 0x02, 0xFF};
unsigned char longReq8[8] = {0x27, 0x1F, 0x03, 0xFF, 0x79, 0x90, 0xFF, 0x79};
unsigned char longReq9[8] = {0x28, 0x91, 0xFF, 0x79, 0x92, 0xFF, 0x79, 0x93};
unsigned char longReq10[8] = {0x29, 0xFF, 0x81, 0x04, 0xFF, 0x81, 0x05, 0xFF};
unsigned char longReq11[8] = {0x2A, 0x81, 0x06, 0xFF, 0x81, 0x07, 0xFF, 0x7E};
unsigned char longReq12[8] = {0x2B, 0x3C, 0xFF, 0x7E, 0x3D, 0xFF, 0x7E, 0x3E};
unsigned char longReq13[8] = {0x2C, 0xFF, 0x7E, 0x3F, 0xFF, 0x7D, 0xA0, 0xFF};
unsigned char longReq14[8] = {0x2D, 0x7D, 0xA1, 0xFF, 0x7D, 0xA2, 0xFF, 0x7D};
unsigned char longReq15[8] = {0x2E, 0xA3, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0F};
unsigned char longReq16[8] = {0x2F, 0xFF, 0x1E, 0xE4, 0xFF, 0x1E, 0xE5, 0xFF};
unsigned char longReq17[8] = {0x20, 0x1E, 0xFA, 0xFF, 0x1E, 0xFB, 0xFF, 0x68};
unsigned char longReq18[8] = {0x21, 0x32, 0xFF, 0x68, 0x33, 0xFF, 0x68, 0x4C};
unsigned char longReq19[8] = {0x22, 0xFF, 0x68, 0x4D, 0x00, 0x00, 0x08, 0x00};
unsigned char longReq20[8] = {0x23, 0x00, 0x12, 0x00, 0x00, 0x3D, 0x00, 0x00};
unsigned char longReq21[8] = {0x24, 0x11, 0xFF, 0x67, 0xF4, 0x00, 0x00, 0x09};
unsigned char longReq22[8] = {0x25, 0x00, 0x00, 0xD8, 0x00, 0x00, 0x15, 0x00};
unsigned char longReq23[8] = {0x26, 0x00, 0x20, 0x00, 0x00, 0x10, 0x00, 0x00};
unsigned char longReq24[8] = {0x27, 0x0A, 0xFF, 0x68, 0x5E, 0x00, 0x00, 0x46};
unsigned char longReq25[8] = {0x28, 0x00, 0x00, 0xCE, 0x00, 0x00, 0xCF, 0x00};
unsigned char longReq26[8] = {0x29, 0x00, 0x1D, 0x00, 0x00, 0x30, 0x00, 0x00};
unsigned char longReq27[8] = {0x2A, 0x3C, 0x00, 0x00, 0xD9, 0x00, 0x00, 0x00};
/* CAN BUS SETUP */

/* RPM BAR VARIABLES */
uint16_t yellowMin, yellowMinPx, yellowMax, yellowMaxPx, yellowFill, redMin, redMinPx, revLimitPx, redMax, redMaxPx, redFill;  // defines all the variables for making the RPM bar
/* RPM BAR VARIABLES */


/* SERIAL CONNECTION TO ARDUINO */
#define HWSERIAL Serial1 //RX pin 0
char buf[10];
int16_t oilTemperature, oilPressure;
/* SERIAL CONNECTION TO ARDUINO */


/* SCREEN SETUP */
/*
#define TFT_DC  10
#define TFT_CS 9
#define TFT_RST 255
#define TFT_SCK 13
#define TFT_MISO 12
#define TFT_MOSI 11
*/

#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10

const uint8_t row1 = 100;
const uint8_t row2 = 130;
const uint8_t row3 = 160;
const uint8_t row4 = 190;
const uint8_t row5 = 240;
const uint8_t oilTempRow = row1;
const uint8_t coolantRow = row2;
const uint8_t oilPressRow = row3;
const uint8_t boostRow = row4;
const uint8_t intakeDamRow = row5;
const uint16_t knockRow = 270;
const uint16_t statusRow = 310;

const uint8_t row1Lrg = row1 - 40;
const uint8_t row2Lrg = row2 - 30;
const uint8_t row3Lrg = row3 - 10;

ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST);
//#define DEBUG_PIN 0
/* SCREEN SETUP */



union floatUnion {
  byte byteArray[4];
  float floatValue;
  int intValue;
};



/* GLOBAL and SETUP VARS */
const bool verbose = 0; // prints the raw packet data for each canbus received message.  this generates a LOT of text!
const bool printStats = 0;  // prints the current gauge data values after each 0x30 packet.  most deprecated with printloopstats in place
const bool printLoopStats = 1;  // prints the current gauge data values when pushing to the display
const bool testData = 1;  // generate fake data and loop it to the display
bool ssmActive = 1; // set to 1 for active sending, 0 for passive listening.  will always turn off passive if it sees other traffic
const unsigned int updateInt = 20; // how fast to do an update in the loop, 50 should be ~10 times a second
unsigned int updateHz;// = 1000 / updateInt; // the hz update speed
// 0 - unknown, 1 - normal, 2 - data logging, 3 - normal with bars.  it will auto detect 2 or 3 when active, if using test data set it manually
unsigned int displayMode = 3; // set this manually for test mode, otherwise it will use the below two values
const unsigned int displayModeNormal = 3; // set which display mode to use when the device is not logging.
const unsigned int displayModeLogging = 2; // set which display mode to use when the accessport is logging
/* GLOBAL and SETUP VARS */


/* FUNCTION DECLARATION FOR PLATFORMIO */
void sendMessage(const unsigned char data[8]);
void sendFlow();
void updateAllBuffer();
float calcAfr(unsigned char data);
void canSniffIso(const CAN_message_t &msg);
float calcFloatFull(unsigned char data[4], float multiplier);
int calcIntFull(unsigned char data[2], float multiplier);
int calcTemp(unsigned char data);
float calcByteToFloat(unsigned char data, float multiplier);
void processOil (char *t);
void sendSmallRequest();
/* FUNCTION DECLARATION FOR PLATFORMIO */


void setup(void) {
  Serial.begin(115200);
  delay(400);
  if (!(testData)) { HWSERIAL.begin(9600); }
  delay(400);
  tft.begin();
  delay(400);
  tft.setRotation(2);

  tft.fillScreen(ILI9341_BLACK);
  //if (!(testData)) { delay(5000); }
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  Serial.println(CORE_PIN10_CONFIG, HEX);
  tft.println("INIT");
  tft.println("SERIAL STARTED");

  tft.println();
  tft.println();
  tft.println();
  tft.setTextSize(2);

  tft.println("INIT CAN");
  Serial.println("Starting can");
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniffIso);

  Can0.setFIFOFilter(REJECT_ALL);
  Can0.setFIFOFilter(0, 0x7E8, STD);
  Can0.setFIFOFilter(1, 0x7E0, STD);

  Can0.mailboxStatus();
  tft.println("WAITING FOR CAN MSG");
  //if (!(testData)) { delay(10000); }
  if (testData) { 
    ssmActive = 0;
    flowCont = 0; 
  }  // turn off SSM active is test data is on, no need for this
}



// new hotness, closs enough to isotp to count
void canSniffIso(const CAN_message_t &msg) {
    
    if (verbose) {
    Serial.print("[VERBOSE] MB "); Serial.print(msg.mb);
    Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" EXT: "); Serial.print(msg.flags.extended);
    Serial.print(" TS: "); Serial.print(msg.timestamp);
    Serial.print(" ID: "); Serial.print(msg.id, HEX);
    Serial.print(" IDD: "); Serial.print(msg.id);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.print(msg.buf[i], HEX); Serial.print(" ");
    } Serial.println();
  }

  // detection for something else requesting data, this will turn off sending to prevent collisions with the ecu
  if ((msg.id == 0x7E0) && (ssmActive == 1)) {
    ssmActive = 0;
    Serial.println("Switching ssm active to 0");
  }

  // only parse 7E8, this will keep the 7E0 monitor from interferring with the data
  if (msg.id == 0x7E8) {
    if (msg.buf[0] == 0x10) {
        //zero out the data for 0x10 new response
        for (uint8_t i = 0; i < 61; i++) {
          responseData[i] = 0x00;
        }

        responseBytes = msg.buf[1] - 1; // read the 2nd byte of the response - how much data to expect.  subtract 1 to not count the response code
        if (responseBytes == 0x11) {
          displayMode = displayModeNormal;  // switch to 6 guage mode
        }
        else if (responseBytes == 0x3D) {
          displayMode = displayModeLogging;  // switch to logging mode
        }
        else {
          displayMode = 0;
          Serial.println(responseBytes, HEX);
        }

        // write the last 5 bytes of the response to the beginning of the array
        for (uint8_t i = 0; i < 5; i++) {
          responseData[byteCount] = msg.buf[i+3];  // byte count starts at 0 here for first run, tracking the array index
          byteCount++;
        }
        packetCount++;
    } // finished with 0x10

    // 0x30 received a continue from the first request byte, that means the last message received was the end of the data 
    else if (msg.buf[0] == 0x30) {
        
        if (printStats) {
            // responseBytes are the returned value (array might have more), this will be the entire response pure data
            for (int z = 0; z < responseBytes; z++) {
                Serial.print(responseData[z], HEX);
                Serial.print(" ");
            }
            Serial.println();
                
            Serial.print("30 | FB: "); Serial.print(feedbackKnockFinal);
            Serial.print(" FN: "); Serial.print(fineKnockFinal);
            Serial.print(" BST: "); Serial.print(boostFinal);
            Serial.print(" COOL: "); Serial.print(coolantFinal);
            Serial.print(" DAM: "); Serial.print(damFinal);
            Serial.print(" INTAKE: "); Serial.print(intakeTempFinal);
            Serial.print(" OIL T: "); Serial.print(oilTemperature);
            Serial.print(" OIL P: "); Serial.println(oilPressure);
        }

        flowCont = 1;
        for (int i = 1; i < 8; i++) {
            if (msg.buf[i] != 0x00) { flowCont = 0; }   // the response wasnt all zeroes, set flow continue to 0 to stop trying to send more messages
        }
        if (verbose) {
            if (flowCont) { Serial.println("[VERBOSE] **************** FLOW CONTINUE RECEIVED"); }
            else { Serial.println("[VERBOSE] !!!!!!!!!!!!!!!!!!!!!!!!!!! FLOW ERROR RECEIVED"); }
        }




        Serial.print("Parsed ");
        Serial.print(packetCount);  // each 0x## message parsed
        Serial.print(" packets in this response and ");
        Serial.print(byteCount);    // each data byte added to the responseData array, should all be in order pure data
        Serial.print(" bytes.  ECU says I should have gotten ");
        Serial.print(responseBytes);    // the number of bytes the 0x10 response said to have, -1 for the response code
        Serial.println(" bytes back.");

        for (int i = 0; i < responseBytes; i++) {
          Serial.print(responseData[i], HEX);
        }
        Serial.println();

        // do work on the final data here.  responseData now has the entire response, array indexes depend on the order of your request
        // 1 or 3 means it detected as a 6 gauge mode
        if ((displayMode == 1) or (displayMode == 3)) {
          unsigned char feedbackKnockData[4] = {responseData[3], responseData[2], responseData[1], responseData[0]};
          feedbackKnockFinal = calcFloatFull(feedbackKnockData, 1);

          unsigned char fineKnockData[4] = {responseData[7], responseData[6], responseData[5], responseData[4]};
          fineKnockFinal = calcFloatFull(fineKnockData, 1);

          unsigned char boostData[4] = {responseData[11], responseData[10], responseData[9], responseData[8]};
          boostFinal = calcFloatFull(boostData, 0.01933677);

          unsigned char rpmData[2] = {responseData[13], responseData[12]};
          rpmFinal = calcIntFull(rpmData, .25);

          coolantFinal = calcTemp(responseData[14]);
          damFinal = calcByteToFloat(responseData[15], 0.0625);
          intakeTempFinal = calcTemp(responseData[16]);
        }
        // 2 means the response was long so its in logging mode
        if (displayMode == 2) {
          unsigned char feedbackKnockData[4] = {responseData[31], responseData[30], responseData[29], responseData[28]};
          feedbackKnockFinal = calcFloatFull(feedbackKnockData, 1);

          unsigned char fineKnockData[4] = {responseData[27], responseData[26], responseData[25], responseData[24]};
          fineKnockFinal = calcFloatFull(fineKnockData, 1);

          unsigned char boostData[4] = {responseData[7], responseData[6], responseData[5], responseData[4]};
          boostFinal = calcFloatFull(boostData, 0.01933677);

          unsigned char rpmData[2] = {responseData[33], responseData[32]};
          rpmFinal = calcIntFull(rpmData, .25);

          coolantFinal = calcTemp(responseData[42]);
          damFinal = calcByteToFloat(responseData[53], 0.0625);
          intakeTempFinal = calcTemp(responseData[43]);
        }


        packetCount = 0;    // next message is going to be the 0x10 starting the next response
        byteCount = 0;
    } // finished with 0x30

    // else it is a continuous byte IE: not 10 or 30, assumed to be in order
    else {   
        for (uint8_t i = 1; i < 8; i++) {
            responseData[byteCount] = msg.buf[i];  // write the values to the array, bytecount global tracks the position 
            byteCount++;
        }
      packetCount++;
    } // finished with continuous message
  } // finished with the 7E8 ID message
}



void loop() {
  
  // some crazy stuff i found on the internet.  how i receive and parse two integers at once via serial from an arduino
  if (!(testData)) {
    if (HWSERIAL.available ()) {
        Serial.println("HWSERIAL Received");
        char buf [80];
        int n = HWSERIAL.readBytesUntil ('\n', buf, sizeof(buf));
        Serial.println(n);
        // check for a real value.  weird things happen if the logic converter is connected but nothing is sending
        if ((n > 1) && (n < 15)) {
          buf [n] = '\0';     // terminate with null
          if (verbose) { Serial.println("[VERBOSE] Serial Received"); }

          char *t = strtok (buf, ",");
          Serial.println(t);
          processOil (t);
          while ((t = strtok (NULL, ",")))
              processOil (t);
        }
    }
  }
  
  // if active is still set, send the entire small request.  since flexcan runs on interrupts, this can run in the loop and still hit the cansniffiso parsing
  // if this is not set, cansniffiso will still work in case an AP is plugged in
  if (ssmActive) {
    if (flowCont) { sendSmallRequest(); }
  }

  // bunch of stuff to just make up data to test how the screen looks
  if (testData) {
    for (int i = -40; i < 280; i++) {
      unsigned long start = micros();
      coolantFinal = i;
      oilTemperature = i;
      intakeTempFinal = i;
      feedbackKnockFinal = (random(0,5) * 1.4) * -1;
      fineKnockFinal = (random(0,2) * 1.4) * -1;
      damFinal = 1;
      boostFinal = (random(-20,20) / 1.1 );
      oilPressure = (random(0,99));
      rpmFinal = random(0,6800);
      if (feedbackKnockFinal < feedbackMax) { feedbackMax = feedbackKnockFinal; }
      if (fineKnockFinal < fineMax) { fineMax = fineKnockFinal; }
      if (fineKnockFinal < 0) {
        if (rpmFinal > fineRpmMax) { fineRpmMax = rpmFinal; }
        if (rpmFinal < fineRpmMin) { fineRpmMin = rpmFinal; } 
      }

      if (printLoopStats) {
        Serial.print("[PRINTLOOPSTATS] FB: "); Serial.print(feedbackKnockFinal);
        Serial.print(" FN: "); Serial.print(fineKnockFinal);
        Serial.print(" BST: "); Serial.print(boostFinal);
        Serial.print(" COOL: "); Serial.print(coolantFinal);
        Serial.print(" DAM: "); Serial.print(damFinal);
        Serial.print(" INTAKE: "); Serial.print(intakeTempFinal);
        Serial.print(" OIL T: "); Serial.print(oilTemperature);
        Serial.print(" OIL P: "); Serial.print(oilPressure);
        Serial.print(" RPM: "); Serial.print(rpmFinal);
      }



      updateAllBuffer();
      delay(updateInt);
      updateHz = 1.0 / ((micros() - start) / 1000000.0);
    }
  }
  // else - you are getting real data from flexcan
  else {
    unsigned long start = micros();
    if (feedbackKnockFinal < feedbackMax) { feedbackMax = feedbackKnockFinal; }
    if (fineKnockFinal < fineMax) { fineMax = fineKnockFinal; }
      if (fineKnockFinal < 0) {
        if (rpmFinal > fineRpmMax) { fineRpmMax = rpmFinal; }
        if (rpmFinal < fineRpmMin) { fineRpmMin = rpmFinal; } 
      }


    if (printLoopStats) {
      Serial.print("[PRINTLOOPSTATS] FB: "); Serial.print(feedbackKnockFinal);
      Serial.print(" FN: "); Serial.print(fineKnockFinal);
      Serial.print(" BST: "); Serial.print(boostFinal);
      Serial.print(" COOL: "); Serial.print(coolantFinal);
      Serial.print(" DAM: "); Serial.print(damFinal);
      Serial.print(" INTAKE: "); Serial.print(intakeTempFinal);
      Serial.print(" OIL T: "); Serial.print(oilTemperature);
      Serial.print(" OIL P: "); Serial.print(oilPressure);
      Serial.print(" RPM: "); Serial.print(rpmFinal);
    }


    updateAllBuffer();
    delay(updateInt);
    updateHz = 1.0 / ((micros() - start) / 1000000.0);
  }
}







// found on stack, wanky way of converting a string containing two integers received via serial into two different ints.  this sucks
void processOil (char *t) {
  char c;
  int  val;
  sscanf (t, "%c%d", &c, &val);

  switch (c) {
  case 'a':
      //if (verbose) {
        Serial.print ("[VERBOSE] cmd a ");
        Serial.println (val);
      //}
      oilTemperature = (val);
      break;

  case 'b':
      //if (verbose) {
        Serial.print ("[VERBOSE] cmd b ");
        Serial.println (val);
      //}
      // write the second value as oil pressure
      if (val < 0) { val = 0; }
      oilPressure = (val);
      break;

  default:
      //if (verbose) {
        Serial.print ("[VERBOSE] unknown cmd ");
        Serial.println (val);
      //}
      break;
  }
}

float calcFloatFull(unsigned char data[4], float multiplier) {
  	if (verbose) {
      Serial.print("[VERBOSE] Input data: ");
      for(int z = 0; z < 4; z++) {
        Serial.print(data[z], HEX);
      }
      Serial.println(); 
    }
  
  
	floatUnion converter; 
  
    for (int i = 0; i < 4; i++) {
      converter.byteArray[i] = data[i];
    }
  
  	float calc = converter.floatValue * multiplier;
  	return calc;  	
}

int calcTemp(unsigned char data) {
  	if (verbose) {
      Serial.print("[VERBOSE] Input data: ");
      Serial.println(data, HEX);
      Serial.println();
    }

	int calc = 32+((9*((data)-40))/5);
	return calc;		
}

float calcByteToFloat(unsigned char data, float multiplier) {
  	if (verbose) {
      Serial.print("[VERBOSE] Input data: ");
      Serial.println(data, HEX);
      Serial.println();
    }

	float calc = data * multiplier;
	return calc;
}

int calcIntFull(unsigned char data[2], float multiplier) { 	
  	if (verbose) {
      Serial.print("[VERBOSE] Input data: ");
      for(int z = 0; z < 2; z++) {
        Serial.print(data[z], HEX);
      }
      Serial.println();
    }

	floatUnion converter; 
  
    for (int i = 0; i < 2; i++) {
      converter.byteArray[i] = data[i];
    }
  
  	int calc = converter.intValue * multiplier;
  	return calc;
}


float calcTargetBoost(unsigned char data[2]) {
  	Serial.print("Input data: ");
  	for(int z = 0; z < 2; z++) {
  		Serial.print(data[z], HEX);
    }
  	Serial.println();
  
	floatUnion converter; 

	float calc = converter.intValue;
	calc = (calc-760)*0.01933677;
	return calc;	
}


int calcAvcs(unsigned char data) {
  	Serial.print("Input data: ");
  	Serial.println(data, HEX);
  	Serial.println();

	int calc = data - 50;
	return calc;
}

float calcTiming(unsigned char data) {
  	Serial.print("Input data: ");
  	Serial.println(data, HEX);
  	Serial.println();
	
	float calc = (data - 128) / 2;
	return calc;
}

int calcByteToInt(unsigned char data) {
  	Serial.print("Input data: ");
  	Serial.println(data, HEX);
  	Serial.println();

	int calc = data;
	return calc;
}

float calcAfCorrection(unsigned char (data)) {
  	Serial.print("Input data: ");
  	Serial.println(data, HEX);
  	Serial.println();

	float calc = ((data - 128) * 100) / 128;
	return calc;
}

float calcThrottle(unsigned char data) {
  	Serial.print("Input data: ");
  	Serial.println(data, HEX);
  	Serial.println();
	
	float calc = (data * 100) / 255;
	return calc;
}	

float calcInjDutyCycle(unsigned char data) {
  	Serial.print("Input data: ");
  	Serial.println(data, HEX);
  	Serial.println();

	float calc = (data * 256) / 1000;
	return calc;
}

float calcAfr(unsigned char data) {
  	Serial.print("Input data: ");
  	Serial.println(data, HEX);
  	Serial.println();

	float calc = (data / 128) * 14.7;
	return calc;
}


void updateAllBuffer() {
  //unsigned long start = micros();
  tft.useFrameBuffer(1);
  tft.fillScreen(ILI9341_BLACK);
  

  /* Top RPM bar */
  int barMap;
  int rpmPx = map(rpmFinal, 0, 8000, 0, 240);

  // defines the yellow and redline blocks based on oil temperature
  if (oilTemperature <= 120) {
    yellowMin = 2500;
    yellowMax = 2999;
    redMin = 3000;
    redMax = 8000;
  }
  else if ((oilTemperature > 120) && (oilTemperature < 160)) {
    yellowMin = 2500;
    yellowMax = 3499;
    redMin = 3500;
    redMax = 8000;
  }
  else {
    yellowMin = 5000;
    yellowMax = 5999;
    redMin = 6000;
    redMax = 8000;
  }

  yellowMinPx = map(yellowMin, 0, 8000, 0, 240);
  yellowMaxPx = map(yellowMax, 0, 8000, 0, 240);
  yellowFill = yellowMaxPx - yellowMinPx;
  redMinPx = map(redMin, 0, 8000, 0, 240);
  revLimitPx = 204;
  redMaxPx = map(redMax, 0, 8000, 0, 240);
  redFill = redMaxPx - redMinPx;

  // draws the actual line for rpm based on 8000 max and 240px width
  if (rpmFinal < yellowMin) {
    tft.fillRect(0, 0, rpmPx, 40, ILI9341_WHITE);
  }
  else if ((rpmFinal >= yellowMin) && (rpmFinal <= yellowMax)) {
    tft.fillRect(0, 0, rpmPx, 40, ILI9341_YELLOW);
  }
  else if ((rpmFinal >= redMin) && (rpmFinal <= redMax)) {
    tft.fillRect(0, 0, rpmPx, 40, ILI9341_RED);
  }

  // this draws the yellow and red top block sections
  tft.fillRect(yellowMinPx, 0, yellowFill, 10, ILI9341_YELLOW);
  tft.fillRect(redMinPx, 0, redFill, 10, ILI9341_RED);

  // this draws the vertical lines that seperate the colors
  tft.drawLine(yellowMinPx, 0, yellowMinPx, 40, ILI9341_WHITE);
  tft.drawLine(redMinPx, 0, redMinPx, 40, ILI9341_WHITE);
  tft.drawLine(revLimitPx, 0, revLimitPx, 40, ILI9341_WHITE);
  /* Top RPM bar */

    
  


  // displaymode 1 - normal mode
  if (displayMode == 1) {
    // row 1 left
    tft.setCursor(10, row1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL TEMP");

    //oil temp
    tft.setCursor(10, row1 + 10);
    tft.setTextSize(3);

    // row 1 right
      // 40 - 129: blue
    if ((oilTemperature >= 40) && (oilTemperature < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    // 130-159 and 225-240: yellow
    else if (((oilTemperature >= 130) && (oilTemperature < 160)) || ((oilTemperature >= 225) && (oilTemperature < 241))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((oilTemperature >= 241) || (oilTemperature < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    } 
    // 160 to 225: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
    }  
    tft.print(oilTemperature);


    tft.setCursor(150, row1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("COOLANT"); 

    //coolant
    // 40 - 129: blue
    if ((coolantFinal >= 40) && (coolantFinal < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    // 130-159 and 207-209: yellow
    else if (((coolantFinal >= 130) && (coolantFinal < 160)) || ((coolantFinal >= 207) && (coolantFinal < 210))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((coolantFinal >= 210) || (coolantFinal < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    } 
    // 160 to 206: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
    }  
    
    tft.setCursor(130, row1 + 10);
    tft.setTextSize(3);
    tft.print(coolantFinal);





    // row 2 left
    tft.setCursor(10, row2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL PRESS");

    //oil pressure
    tft.setCursor(10, row2 + 10);
    tft.setTextSize(3);
    tft.print(oilPressure);


    // row 2 right
    tft.setCursor(150, row2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("INTAKE");

    // intake temp
    tft.setCursor(130, row2 + 10);
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_WHITE);
    tft.print(intakeTempFinal);


    // row 3 left
    tft.setCursor(10, row3);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("DAM");

    //dam
    tft.setCursor(10, row3 + 10);
    tft.setTextSize(3);
    if (damFinal != 1.00) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(damFinal);


    // row 3 right
    tft.setCursor(150, row3);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("BOOST");

    //boost
    tft.setCursor(130, row3 + 10);
    tft.setTextSize(3);
    tft.print(boostFinal);


    // row 4 left
    tft.setCursor(10, row4);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FEEDBACK KNOCK");

    // feedback knock
    tft.setCursor(10, row4 + 10);
    tft.setTextSize(3);
    if (feedbackKnockFinal < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(feedbackMax);


    // row 4 right
    tft.setCursor(150, row4);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FINE KNOCK");

    //fine knock
    tft.setCursor(130, row4 + 10);
    tft.setTextSize(3);
    if (fineKnockFinal < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(fineMax);
    tft.setTextColor(ILI9341_WHITE);


    tft.setTextSize(2);
    tft.setCursor(130, 225);
    if (fineRpmMin == 9999) { tft.print(0); }
    else { tft.print(fineRpmMin); }
    tft.setCursor(190, 225);
    tft.print(fineRpmMax);
  }

  // displaymode 2 aka race mode
  else if (displayMode == 2) {
  
    // row 1 left
    tft.setCursor(10, row1Lrg);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL TEMP");

    //oil temp
    tft.setCursor(10, row1Lrg + 10);
    tft.setTextSize(6);

    // row 1 right
      // 40 - 129: blue
    if ((oilTemperature >= 40) && (oilTemperature < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    // 130-159 and 225-240: yellow
    else if (((oilTemperature >= 130) && (oilTemperature < 160)) || ((oilTemperature >= 225) && (oilTemperature < 241))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((oilTemperature >= 241) || (oilTemperature < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    } 
    // 160 to 225: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
    }  
    tft.print(oilTemperature);


    tft.setCursor(150, row1Lrg);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("COOLANT"); 

    //coolant
    // 40 - 129: blue
    if ((coolantFinal >= 40) && (coolantFinal < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    // 130-159 and 207-209: yellow
    else if (((coolantFinal >= 130) && (coolantFinal < 160)) || ((coolantFinal >= 207) && (coolantFinal < 210))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((coolantFinal >= 210) || (coolantFinal < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    } 
    // 160 to 206: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
    }  
    
    tft.setCursor(130, row1Lrg + 10);
    tft.setTextSize(6);
    tft.print(coolantFinal);





    // row 2 left
    tft.setCursor(10, row2Lrg + 30);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL PRESS");

    //oil pressure
    tft.setCursor(10, row2Lrg + 40);
    tft.setTextSize(4);
    tft.print(oilPressure);


    // row 2 right
    tft.setCursor(150, row2Lrg + 30);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("INTAKE");

    // intake temp
    tft.setCursor(130, row2Lrg + 40);
    tft.setTextSize(4);
    tft.setTextColor(ILI9341_WHITE);
    tft.print(intakeTempFinal);


    // row 3 left
    tft.setCursor(10, row3Lrg + 40);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("DAM");

    //dam
    tft.setCursor(10, row3Lrg + 50);
    tft.setTextSize(3);
    if (damFinal != 1.00) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(damFinal);


    // row 3 right
    tft.setCursor(150, row3Lrg + 40);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("BOOST");

    //boost
    tft.setCursor(130, row3Lrg + 50);
    tft.setTextSize(3);
    tft.print(boostFinal);


    // row 4 left
    tft.setCursor(10, row4 + 50);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FEEDBACK KNOCK");

    // feedback knock
    tft.setCursor(10, row4 + 60);
    tft.setTextSize(3);
    if (feedbackKnockFinal < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(feedbackMax);


    // row 4 right
    tft.setCursor(150, row4 + 50);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FINE KNOCK");

    //fine knock
    tft.setCursor(130, row4 + 60);
    tft.setTextSize(3);
    if (fineKnockFinal < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(fineMax);
    tft.setTextColor(ILI9341_WHITE);


    // prints the RPM range of the fine knock events its seen
    tft.setTextSize(2);
    tft.setCursor(130, 280);
    if (fineRpmMin == 9999) { tft.print(0); }
    else { tft.print(fineRpmMin); }
    tft.setCursor(190, 280);
    tft.print(fineRpmMax);    
  }

  // displaymodd 3 - new normal mode with bars
  else if (displayMode == 3) {
    
    /* OIL TEMP */
    // print oil temp label
    tft.setCursor(0, oilTempRow);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println(" OIL\n TEMP");

    // draws the first row empty bar
    tft.drawRect(40, oilTempRow, 130, 15, ILI9341_WHITE);
    tft.fillRect(117, oilTempRow + 1, 25, 4, ILI9341_GREEN); // green range
    tft.fillRect(142, oilTempRow + 1, 5, 4, ILI9341_YELLOW); // yellow range
    tft.fillRect(147, oilTempRow + 1, 22, 4, ILI9341_RED); // red range

    // maps the oil temp value to pixels for the bar printing
    //barMap = map(oilTemperature, 147, 277, 0, 130);
    barMap = map(oilTemperature, -40, 300, 0, 130);
    //Serial.println(barMap);

    // prints oil temp bar fill and number
    tft.setCursor(180, oilTempRow - 4);
    tft.setTextSize(3);
      // 40 - 129: blue
    if ((oilTemperature >= 40) && (oilTemperature < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      tft.fillRect(41, oilTempRow + 1, barMap, 13, ILI9341_BLUE);
    }
    // 130-159 and 225-240: yellow
    else if (((oilTemperature >= 130) && (oilTemperature < 160)) || ((oilTemperature >= 225) && (oilTemperature < 241))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
      tft.fillRect(41, oilTempRow + 1, barMap, 13, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((oilTemperature >= 241) || (oilTemperature < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
      if (barMap > 128) { barMap = 128; }
      tft.fillRect(41, oilTempRow + 1, barMap, 13, ILI9341_RED);
    } 
    // 160 to 225: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
      tft.fillRect(41, oilTempRow + 1, barMap, 13, ILI9341_WHITE);
    }  
    tft.print(oilTemperature);
    /* OIL TEMP */



    /* COOLANT TEMP */
    tft.setCursor(0, coolantRow);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println(" COOL\n TEMP");

    // draws the first row empty bar
    //tft.drawRect(40, 90, 130, 24, ILI9341_WHITE);
    tft.drawRect(40, coolantRow, 130, 15, ILI9341_WHITE);
    tft.fillRect(125, coolantRow + 1, 19, 4, ILI9341_GREEN); // green range
    tft.fillRect(144, coolantRow + 1, 2, 4, ILI9341_YELLOW); // yellow range
    tft.fillRect(146, coolantRow + 1, 23, 4, ILI9341_RED); // red range

    // maps the coolant temp value to pixels for the bar printing
    //barMap = map(coolantFinal, 140, 270, 0, 130);
    barMap = map(coolantFinal, -40, 270, 0, 130);
    Serial.println(barMap);

    //coolant
    // 40 - 129: blue
    if ((coolantFinal >= 40) && (coolantFinal < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
      tft.fillRect(41, coolantRow + 1, barMap, 13, ILI9341_BLUE);
    }
    // 130-159 and 207-209: yellow
    else if (((coolantFinal >= 130) && (coolantFinal < 160)) || ((coolantFinal >= 207) && (coolantFinal < 210))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
      tft.fillRect(41, coolantRow + 1, barMap, 13, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((coolantFinal >= 210) || (coolantFinal < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
      if (barMap > 128) { barMap = 128; }
      tft.fillRect(41, coolantRow + 1, barMap, 13, ILI9341_RED);
    } 
    // 160 to 206: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
      tft.fillRect(41, coolantRow + 1, barMap, 13, ILI9341_WHITE);
    }  
    
    tft.setCursor(180, coolantRow - 4);
    tft.setTextSize(3);
    tft.print(coolantFinal);
    /* COOLANT TEMP */



    /* OIL PRESSURE */
    tft.setCursor(10, oilPressRow);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL\nPRESS");

    //oil pressure
    tft.setCursor(180, oilPressRow - 4);
    tft.setTextSize(3);
    if (oilPressure <= 14 | oilPressure >= 90) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(oilPressure);

    // draws the first row empty bar
    tft.drawRect(40, oilPressRow, 130, 15, ILI9341_WHITE);
    tft.fillRect(41, oilPressRow + 1, 20, 4, ILI9341_RED); // green range
    tft.fillRect(156, oilPressRow + 1, 13, 4, ILI9341_RED); // yellow range

    // maps the coolant temp value to pixels for the bar printing
    barMap = map(oilPressure, 0, 100, 0, 130);
    tft.fillRect(41, oilPressRow + 1, barMap, 13, ILI9341_WHITE);
    /* OIL PRESSURE */



    /* BOOST */
    tft.setCursor(0, boostRow);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("BOOST");

    //boost
    tft.setCursor(160, boostRow - 4);
    tft.setTextSize(2);
    tft.print(boostFinal);

    // draws the first row empty bar
    tft.drawRect(40, boostRow - 4, 110, 15, ILI9341_WHITE);
    tft.fillRect(144, boostRow - 3, 5, 4, ILI9341_RED); // yellow range

    // maps the boost value to pixels for the bar printing
    barMap = map(boostFinal, 0, 20, 0, 110);
    tft.fillRect(41, boostRow -4, barMap, 14, ILI9341_WHITE);
    /* BOOST */
    
    


    /* INTAKE AND DAM */
    tft.setCursor(0, intakeDamRow);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("INTAKE");

    // intake temp
    tft.setCursor(50, intakeDamRow - 4);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.print(intakeTempFinal);


    // row 5 right
    tft.setCursor(130, intakeDamRow);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("DAM");

    //dam
    tft.setCursor(170, intakeDamRow - 4);
    tft.setTextSize(2);
    if (damFinal != 1.00) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(damFinal);
    /* INTAKE AND DAM */


    // row 6 left
    tft.setCursor(0, knockRow);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FEED");

    // feedback knock
    tft.setCursor(40, knockRow);
    tft.setTextSize(2);
    if (feedbackKnockFinal < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(feedbackMax);


    // row 6 right
    tft.setCursor(130, knockRow);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FINE");

    //fine knock
    tft.setCursor(160, knockRow);
    tft.setTextSize(2);
    if (fineKnockFinal < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(fineMax);
    tft.setTextColor(ILI9341_WHITE);


    tft.setTextSize(1);
    tft.setCursor(130, knockRow + 20);
    if (fineRpmMin == 9999) { tft.print(0); }
    else { tft.print(fineRpmMin); }
    tft.setCursor(190, knockRow + 20);
    tft.print(fineRpmMax);








  }

  // anything else, mostly to handle a mode 0 if the canbus data is not parsed right
  else {


    tft.setCursor(10, row1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL TEMP");

    //oil temp
    tft.setCursor(10, row1 + 10);
    tft.setTextSize(4);

    // row 1 right
      // 40 - 129: blue
    if ((oilTemperature >= 40) && (oilTemperature < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    // 130-159 and 225-240: yellow
    else if (((oilTemperature >= 130) && (oilTemperature < 160)) || ((oilTemperature >= 225) && (oilTemperature < 241))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((oilTemperature >= 241) || (oilTemperature < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    } 
    // 160 to 225: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
    }  
    tft.print(oilTemperature);












    tft.setCursor(180, row1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL PRESS");





    //oil pressure
    tft.setCursor(180, row1 + 10);
    tft.setTextSize(4);
    tft.print(oilPressure);


    tft.setCursor(0, 150);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(3);
    tft.println("UNKN CAN DATA");
  }





  ////////////////////////////////////////////////////////////////////// bottom
  // sets the first block: active/passive/testing mode
  if (ssmActive) {
    tft.setCursor(5,statusRow);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
    tft.print("ACTIVE");
  }
  else if ((!(ssmActive)) && (testData)) {
    tft.setCursor(5,statusRow);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_BLACK, ILI9341_RED);
    tft.print("TESTING");    
  }
  else {
    tft.setCursor(5,statusRow);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    tft.print("PASSIVE");
  }

  // sets the second block: refresh rate, now actually shows the real value based on the global var
  tft.setCursor(50,statusRow);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
  tft.print(updateHz);
  tft.print("Hz");
  
  // sets the third block: display mode normal/race/unknown
  tft.setCursor(85,statusRow);
  tft.setTextSize(1);
  if (displayMode == 1) {
    tft.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
    tft.print("MODE: NORMAL");   
  }
  else if (displayMode == 2) {
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLUE);
    tft.print("MODE: RACE");   
  }
  else if (displayMode == 3) {
    tft.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
    tft.print("MODE: NORMAL v2");   
  }
  else {
    tft.setTextColor(ILI9341_BLACK, ILI9341_RED);
    tft.print("MODE: UNKNOWN");   
  }

  tft.setCursor(180,statusRow);
  if (flowCont) {
    tft.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
  }
  else {
    tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
  }
  tft.print("FLOW"); 
  
  ////////////////////////////////////////////////////////////////////// bottom end

  
  tft.updateScreen(); 
  //fps = 1.0 / ((micros() - start) / 1000000.0);
  //Serial.println(fps);
}




// sends the large request that mimics my AP datalogging mode, not really used for anything currently
void sendLargeRequest() {
  
  sendMessage(longReq1);
  delay(10);
  sendMessage(longReq2);
  sendMessage(longReq3);
  sendMessage(longReq4);
  sendMessage(longReq5);
  sendMessage(longReq6);
  sendMessage(longReq7);
  sendMessage(longReq8);
  sendMessage(longReq9);
  sendMessage(longReq10);
  sendMessage(longReq11);
  sendMessage(longReq12);
  sendMessage(longReq13);
  sendMessage(longReq14);
  sendMessage(longReq15);
  sendMessage(longReq16);
  sendMessage(longReq17);
  sendMessage(longReq18);
  sendMessage(longReq19);
  sendMessage(longReq20);
  sendMessage(longReq21);
  sendMessage(longReq22);
  sendMessage(longReq23);
  sendMessage(longReq24);
  sendMessage(longReq25);
  sendMessage(longReq26);
  sendMessage(longReq27);
  delay(10);
  sendFlow();
}

// sends the regular 6 gauge display request for normal operation
void sendSmallRequest() {
  sendMessage(req1);
  delay(10);
  sendMessage(req2);
  sendMessage(req3);
  sendMessage(req4);
  sendMessage(req5);
  sendMessage(req6);
  sendMessage(req7);
  sendMessage(req8);
  delay(10);
  sendFlow();
}

void sendFlow() {
    CAN_message_t msg;
    msg.id = 0x7E0;

    for (int i = 0; i < 8; i++) {
      msg.buf[i] = req0[i];
    }

    if (verbose) { Serial.println("[VERBOSE] Sending flow message"); }
    Can0.write(msg);
}

void sendMessage(const unsigned char data[8]) {
    CAN_message_t msg;
    msg.id = 0x7E0;


      //Serial.print("Sending message: ");
      for (int i = 0; i < 8; i++) {
        msg.buf[i] = data[i];
        //Serial.print(msg.buf[i], HEX);
        //Serial.print(" ");
      }
      //Serial.println();
      Can0.write(msg);
}
