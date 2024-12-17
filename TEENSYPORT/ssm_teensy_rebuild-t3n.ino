#include <FlexCAN_T4.h>
#include "SPI.h"
#include "ILI9341_t3n.h"

/* CAN BUS SETUP */
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

bool set1 = 0; // only used for the old cansniff function, not the isotp style
bool set2 = 0; // only used for the old cansniff function, not the isotp style

unsigned char responseData[61]; // holds the entire response
uint8_t packetCount = 0;  // how many packets parsed in this block
uint8_t byteCount = 0;  // how many bytes written, this will include any empty bytes filling the last packet
uint8_t responseBytes;  // the value in 0x10 that the ECU says is how many bytes its sending

unsigned char fineKnockData[4]; // 6 gauge fine kock is 2 lines
float feedbackKnockFinal;
float feedbackMax = 0;
float fineKnockFinal;
float fineMax = 0;
int fineRpmMin = 9999;
int fineRpmMax = 0;
float boostFinal;
int coolantFinal;
float damFinal;
int intakeTempFinal;
int rpmFinal;
int shift;  // calculated 1-13 number of #'s to print for the shift lights.  0-1000 for 1, then 500 increments

const unsigned char req0[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char req1[8] = {0x10, 0x35, 0xA8, 0x00, 0xFF, 0x7D, 0xA0, 0xFF};
const unsigned char req2[8] = {0x21, 0x7D, 0xA1, 0xFF, 0x7D, 0xA2, 0xFF, 0x7D};
const unsigned char req3[8] = {0x22, 0xA3, 0xFF, 0x7E, 0x3C, 0xFF, 0x7E, 0x3D};
const unsigned char req4[8] = {0x23, 0xFF, 0x7E, 0x3E, 0xFF, 0x7E, 0x3F, 0xFF};
const unsigned char req5[8] = {0x24, 0x62, 0x00, 0xFF, 0x62, 0x01, 0xFF, 0x62};
const unsigned char req6[8] = {0x25, 0x02, 0xFF, 0x62, 0x03, 0x00, 0x00, 0x0E};
const unsigned char req7[8] = {0x26, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x08, 0xFF};
const unsigned char req8[8] = {0x27, 0x68, 0x5E, 0x00, 0x00, 0x12, 0x00, 0x00};

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

bool flowCont = 1;
/* CAN BUS SETUP */



/* SERIAL CONNECTION TO ARDUINO */
#define HWSERIAL Serial1
char buf[10];
int oilTemperature, oilPressure;
int prevOilTemperature = 0;
/* SERIAL CONNECTION TO ARDUINO */


/* SCREEN SETUP */
#define TFT_DC  10
#define TFT_CS 9
#define TFT_RST 255
#define TFT_SCK 13
#define TFT_MISO 12
#define TFT_MOSI 11

int row1 = 40;
int row2 = 90;
int row3 = 140;
int row4 = 190;
int row5 = 240;

ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCK, TFT_MISO);
//#define DEBUG_PIN 0
/* SCREEN SETUP */



union floatUnion {
  byte byteArray[4];
  float floatValue;
  int intValue;
};




bool verbose = 1; // prints the raw packet data for each canbus received message
bool printStats = 0;  // prints the current gauge data values after each 0x30 packet
bool printLoopStats = 1;  // prints the current gauge data values when pushing to the display
bool testData = 1;  // generate fake data and loop it to the display
bool ssmActive = 1; // set to 1 for active sending, 0 for passive listening.  will always turn off passive if it sees other traffic
int updateInt = 500; // how fast to do an update in the loop, 50 should be 20 times a second
int displayMode = 2; // 0 - unknown, 1 - normal, 2 - data logging.  it will auto detect from boot set to 0, if using test data set it manually

void setup(void) {
  Serial.begin(115200); delay(400);
  HWSERIAL.begin(9600); delay(400);
  tft.begin(); delay(400);

  tft.fillScreen(ILI9341_BLACK);
  delay(5000);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  Serial.println(CORE_PIN10_CONFIG, HEX);
  tft.println("INIT");
  tft.println("SERIAL STARTED");

  // read diagnostics (optional but can help debug problems)
  tft.setTextSize(1);
  //digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  tft.print("Display Power Mode: 0x"); tft.println(x, HEX);
  //digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  tft.print("MADCTL Mode: 0x"); tft.println(x, HEX);
  //digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  tft.print("Pixel Format: 0x"); tft.println(x, HEX);
  //digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  tft.print("Image Format: 0x"); tft.println(x, HEX);
  //digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);
  tft.print("Self Diagnostic: 0x"); tft.println(x, HEX);
  tft.println();
  tft.println();
  tft.println();
  tft.setTextSize(2);



  tft.println("INIT CAN");
  Serial.println("Starting can");
  //pinMode(6, OUTPUT); digitalWrite(6, LOW); // optional tranceiver enable pin
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
  delay(10000);
}




// old and busted.  it worked but its ugly
void canSniff(const CAN_message_t &msg) {
    
  if (verbose) {
    Serial.print("MB "); Serial.print(msg.mb);
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


  if (msg.buf[0] == 0x10) {
    
   
    // original packet by packet parsing
    unsigned char data[4] = {msg.buf[6], msg.buf[5], msg.buf[4], msg.buf[3]};
    feedbackKnockFinal = calcFloatFull(data, 1);

    fineKnockData[3] = msg.buf[7];
    set1 = 1;
    if (set1 & set2) {
      fineKnockFinal = calcFloatFull(fineKnockData, 1);
      set1 = 0;
      set2 = 0;
    }
    //
  } // finished with 0x10

  if (msg.buf[0] == 0x21) {
    fineKnockData[2] = msg.buf[1];
    fineKnockData[1] = msg.buf[2];
    fineKnockData[0] = msg.buf[3];
    set2 = 1;
    if (set1 & set2) {
      fineKnockFinal = calcFloatFull(fineKnockData, 1);
      set1 = 0;
      set2 = 0;
    }

    unsigned char data[4] = {msg.buf[7], msg.buf[6], msg.buf[5], msg.buf[4]};
    boostFinal = calcFloatFull(data, 0.01933677);
  }

  if (msg.buf[0] == 0x22) {
    unsigned char data[2] = {msg.buf[2], msg.buf[1]};
    rpmFinal = calcIntFull(data, .25);
    shift = calcShift(rpmFinal);
    coolantFinal = calcTemp(msg.buf[3]);
    damFinal = calcByteToFloat(msg.buf[4], 0.0625);
    intakeTempFinal = calcTemp(msg.buf[5]);
  }
  
  if (msg.buf[0] == 0x30) {

    if (printStats) {
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
      if (msg.buf[i] != 0x00) { flowCont = 0; }
    }
    if (verbose) {
      if (flowCont) { Serial.println("**************** FLOW CONTINUE RECEIVED"); }
      else { Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!! FLOW ERROR RECEIVED"); }
    }
    //cnt++;
  }
}

// new hotness, closs enough to isotp to count
void canSniffIso(const CAN_message_t &msg) {
    
    if (verbose) {
    Serial.print("MB "); Serial.print(msg.mb);
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
          displayMode = 1;  // switch to 6 guage mode
        }
        else if (responseBytes == 0x3D) {
          displayMode = 2;  // switch to logging mode
        }
        else {
          displayMode = 0;
          Serial.println(responseBytes, HEX);
        }

        // write the last 5 bytes of the response to the beginning of the array
        for (uint8_t i = 0; i < 5; i++) {
          responseData[byteCount] = msg.buf[i+3];  // byte count starts at 0 here, tracking the array
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
            if (flowCont) { Serial.println("**************** FLOW CONTINUE RECEIVED"); }
            else { Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!! FLOW ERROR RECEIVED"); }
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

        // do work on the final data here

        if (displayMode == 1) {
          unsigned char feedbackKnockData[4] = {responseData[3], responseData[2], responseData[1], responseData[0]};
          feedbackKnockFinal = calcFloatFull(feedbackKnockData, 1);

          unsigned char fineKnockData[4] = {responseData[7], responseData[6], responseData[5], responseData[4]};
          fineKnockFinal = calcFloatFull(fineKnockData, 1);

          unsigned char boostData[4] = {responseData[11], responseData[10], responseData[9], responseData[8]};
          boostFinal = calcFloatFull(boostData, 0.01933677);

          unsigned char rpmData[2] = {responseData[13], responseData[12]};
          rpmFinal = calcIntFull(rpmData, .25);
          shift = calcShift(rpmFinal);

          coolantFinal = calcTemp(responseData[14]);
          damFinal = calcByteToFloat(responseData[15], 0.0625);
          intakeTempFinal = calcTemp(responseData[16]);
        }

        if (displayMode == 2) {
          unsigned char feedbackKnockData[4] = {responseData[31], responseData[30], responseData[29], responseData[28]};
          feedbackKnockFinal = calcFloatFull(feedbackKnockData, 1);

          unsigned char fineKnockData[4] = {responseData[27], responseData[26], responseData[25], responseData[24]};
          fineKnockFinal = calcFloatFull(fineKnockData, 1);

          unsigned char boostData[4] = {responseData[7], responseData[6], responseData[5], responseData[4]};
          boostFinal = calcFloatFull(boostData, 0.01933677);

          unsigned char rpmData[2] = {responseData[33], responseData[32]};
          rpmFinal = calcIntFull(rpmData, .25);
          shift = calcShift(rpmFinal);

          coolantFinal = calcTemp(responseData[42]);
          damFinal = calcByteToFloat(responseData[53], 0.0625);
          intakeTempFinal = calcTemp(responseData[43]);
        }


        packetCount = 0;    // next message is going to be the 0x10 starting the next response
        byteCount = 0;
    } // finished with 0x30

    // else it is a continuous byte, assumed to be in order
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
  
  
  if (HWSERIAL.available ()) {
      char buf [80];
      int n = HWSERIAL.readBytesUntil ('\n', buf, sizeof(buf));
      buf [n] = '\0';     // terminate with null
       if (verbose) { Serial.println("Serial Received"); }

      char *t = strtok (buf, ",");
      processOil (t);
      while ((t = strtok (NULL, ",")))
          processOil (t);
  }
  
  
  if (ssmActive) {
    if (flowCont) { sendSmallRequest(); }
  }

  if (testData) {
    for (int i = -40; i < 280; i++) {
      coolantFinal = i;
      oilTemperature = i;
      intakeTempFinal = i;
      feedbackKnockFinal = (random(0,5) * 1.4) * -1;
      fineKnockFinal = (random(0,2) * 1.4) * -1;
      damFinal = 1;
      boostFinal = (random(-20,20) / 1.1 );
      oilPressure = (random(0,99));
      rpmFinal = random(0,6800);
      shift = random(1,13);
      if (feedbackKnockFinal < feedbackMax) { feedbackMax = feedbackKnockFinal; }
      if (fineKnockFinal < fineMax) { fineMax = fineKnockFinal; }
      if (fineKnockFinal < 0) {
        if (rpmFinal > fineRpmMax) { fineRpmMax = rpmFinal; }
        if (rpmFinal < fineRpmMin) { fineRpmMin = rpmFinal; } 
      }

      updateAllBuffer(row1, feedbackKnockFinal, fineKnockFinal, row2, coolantFinal, intakeTempFinal, row3, damFinal, boostFinal, shift, row4, oilTemperature, oilPressure, row5, feedbackMax, fineMax, fineRpmMin, fineRpmMax);
      delay(updateInt);
    }
  }
  else {
    if (feedbackKnockFinal < feedbackMax) { feedbackMax = feedbackKnockFinal; }
    if (fineKnockFinal < fineMax) { fineMax = fineKnockFinal; }
      if (fineKnockFinal < 0) {
        if (rpmFinal > fineRpmMax) { fineRpmMax = rpmFinal; }
        if (rpmFinal < fineRpmMin) { fineRpmMin = rpmFinal; } 
      }
    updateAllBuffer(row1, feedbackKnockFinal, fineKnockFinal, row2, coolantFinal, intakeTempFinal, row3, damFinal, boostFinal, shift, row4, oilTemperature, oilPressure, row5, feedbackMax, fineMax, fineRpmMin, fineRpmMax);
  }
  
  

  if (printLoopStats) {
    Serial.print("FB: "); Serial.print(feedbackKnockFinal);
    Serial.print(" FN: "); Serial.print(fineKnockFinal);
    Serial.print(" BST: "); Serial.print(boostFinal);
    Serial.print(" COOL: "); Serial.print(coolantFinal);
    Serial.print(" DAM: "); Serial.print(damFinal);
    Serial.print(" INTAKE: "); Serial.print(intakeTempFinal);
    Serial.print(" OIL T: "); Serial.print(oilTemperature);
    Serial.print(" OIL P: "); Serial.print(oilPressure);
    Serial.print(" RPM: "); Serial.print(rpmFinal);
    Serial.print(" SHIFT: "); Serial.println(shift);
  }

  delay(updateInt);  
}






int calcShift(int data) {
  if (verbose) {
    Serial.println(data);
  }

  int calc;

  switch (data) {
    case 0 ... 1000:
      calc = 1;
      break;
    case 1001 ... 1500:
      calc = 2;
      break;
    case 1501 ... 2000:
      calc = 3;
      break;
    case 2001 ... 2500:
      calc = 4;
      break;
    case 2501 ... 3000:
      calc = 5;
      break;
    case 3001 ... 3500:
      calc = 6;
      break;
    case 3501 ... 4000:
      calc = 7;
      break;
    case 4001 ... 4500:
      calc = 8;
      break;
    case 4501 ... 5000:
      calc = 9;
      break;
    case 5001 ... 5500:
      calc = 10;
      break;
    case 5501 ... 6000:
      calc = 11;
      break;
    case 6001 ... 6500:
      calc = 12;
      break;
    case 6501 ... 10000:
      calc = 13;
      break;
    default:
      calc = 13;
      break;
  }

  return calc;
}



void processOil (char *t) {
  char c;
  int  val;
  sscanf (t, "%c%d", &c, &val);

  switch (c) {
  case 'a':
      if (verbose) {
        Serial.print (" cmd a ");
        Serial.println (val);
      }
      oilTemperature = (val);
      if (verbose) {
        Serial.print("Previous Oil: ");
        Serial.print(prevOilTemperature);
        Serial.print(" Current Oil: ");
        Serial.print(val);
      }
      oilTemperature = (oilTemperature + prevOilTemperature) / 2;
      if (verbose) {
        Serial.print(" Average Oil: ");
        Serial.println(oilTemperature);
      }
      prevOilTemperature = oilTemperature;

      break;

  case 'b':
      if (verbose) {
        Serial.print (" cmd b ");
        Serial.println (val);
      }
      oilPressure = (val);
      break;

  default:
      if (verbose) {
        Serial.print (" unknown cmd ");
        Serial.println (val);
      }
      break;
  }
}

float calcFloatFull(unsigned char data[4], float multiplier) {
  	if (verbose) {
      Serial.print("Input data: ");
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
      Serial.print("Input data: ");
      Serial.println(data, HEX);
      Serial.println();
    }

	int calc = 32+((9*((data)-40))/5);
	return calc;		
}

float calcByteToFloat(unsigned char data, float multiplier) {
  	if (verbose) {
      Serial.print("Input data: ");
      Serial.println(data, HEX);
      Serial.println();
    }

	float calc = data * multiplier;
	return calc;
}

int calcIntFull(unsigned char data[2], float multiplier) { 	
  	if (verbose) {
      Serial.print("Input data: ");
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

// updateAllBuffer(row1, feedbackKnockFinal, fineKnockFinal, row2, coolantFinal, intakeTempFinal, row3, damFinal, boostFinal, shift, row4, oilTemperature, oilPressure, row5, feedbackMax, fineMax, fineRpmMin, fineRpmMax);
// v1 = feedback
// v2 = fine knock
// v3 = coolant
// v4 = intake temp
// v5 = dam
// v6 = boost
// v8 = oil temp
// v9 = oil pressure
// v10 = feedback max
// v11 = fine max
// v12 = fine rpm min
// v13 = fine rpm max

void updateAllBuffer(int row1, float v1, float v2, int row2, int v3, int v4, int row3, float v5, float v6, int v7, int row4, int v8, int v9, int row5, float v10, float v11, int v12, int v13) {
  tft.useFrameBuffer(1);
  tft.fillScreen(ILI9341_BLACK);

  /* RPM SECTION */
  tft.setTextSize(3);
  tft.setCursor(0,15);
  // handle 0-6000
  if (v7 <= 11) {
    for (int z = 0; z < v7; z++) {
      if (z < 5) {
        tft.setTextColor(ILI9341_WHITE);
      }
      else if ((z >= 5) && (z < 9)) {
        tft.setTextColor(ILI9341_GREEN);
      }
      else if ((z >= 9)) {
        tft.setTextColor(ILI9341_RED);
      }
      tft.print("#");
    }
  }
  //handle 6001+
  if (v7 > 11) {
    tft.setTextColor(ILI9341_BLUE);
    tft.print("#############");
  }
  /* RPM SECTION END */




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
    if ((v8 >= 40) && (v8 < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    // 130-159 and 225-240: yellow
    else if (((v8 >= 130) && (v8 < 160)) || ((v8 >= 225) && (v8 < 241))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((v8 >= 241) || (v8 < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    } 
    // 160 to 225: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
    }  
    tft.print(v8);


    tft.setCursor(150, row1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("COOLANT"); 

    //coolant
    // 40 - 129: blue
    if ((v3 >= 40) && (v3 < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    // 130-159 and 207-209: yellow
    else if (((v3 >= 130) && (v3 < 160)) || ((v3 >= 207) && (v3 < 210))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((v3 >= 210) || (v3 < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    } 
    // 160 to 206: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
    }  
    
    tft.setCursor(130, row1 + 10);
    tft.setTextSize(3);
    tft.print(v3);





    // row 2 left
    tft.setCursor(10, row2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL PRESS");

    //oil pressure
    tft.setCursor(10, row2 + 10);
    tft.setTextSize(3);
    tft.print(v9);


    // row 2 right
    tft.setCursor(150, row2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("INTAKE");

    // intake temp
    tft.setCursor(130, row2 + 10);
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_WHITE);
    tft.print(v4);


    // row 3 left
    tft.setCursor(10, row3);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("DAM");

    //dam
    tft.setCursor(10, row3 + 10);
    tft.setTextSize(3);
    if (v5 != 1.00) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(v5);


    // row 3 right
    tft.setCursor(150, row3);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("BOOST");

    //boost
    tft.setCursor(130, row3 + 10);
    tft.setTextSize(3);
    tft.print(v6);


    // row 4 left
    tft.setCursor(10, row4);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FEEDBACK KNOCK");

    // feedback knock
    tft.setCursor(10, row4 + 10);
    tft.setTextSize(3);
    if (v1 < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(v10);


    // row 4 right
    tft.setCursor(150, row4);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FINE KNOCK");

    //fine knock
    tft.setCursor(130, row4 + 10);
    tft.setTextSize(3);
    if (v2 < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(v11);
    tft.setTextColor(ILI9341_WHITE);


    //feedback max
    //tft.setCursor(10, row5 + 10);
    //tft.print(v1);

    //fine max
    //tft.setCursor(130, row5 + 10);
    //tft.print(v2);

    tft.setTextSize(2);
    tft.setCursor(130, 225);
    if (v12 == 9999) { tft.print(0); }
    else { tft.print(v12); }
    tft.setCursor(190, 225);
    tft.print(v13);
  }

  else if (displayMode == 2) {
  
    // row 1 left
    tft.setCursor(10, row1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL TEMP");

    //oil temp
    tft.setCursor(10, row1 + 10);
    tft.setTextSize(6);

    // row 1 right
      // 40 - 129: blue
    if ((v8 >= 40) && (v8 < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    // 130-159 and 225-240: yellow
    else if (((v8 >= 130) && (v8 < 160)) || ((v8 >= 225) && (v8 < 241))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((v8 >= 241) || (v8 < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    } 
    // 160 to 225: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
    }  
    tft.print(v8);


    tft.setCursor(150, row1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("COOLANT"); 

    //coolant
    // 40 - 129: blue
    if ((v3 >= 40) && (v3 < 130)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    }
    // 130-159 and 207-209: yellow
    else if (((v3 >= 130) && (v3 < 160)) || ((v3 >= 207) && (v3 < 210))) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    // 210+ or below 40: red
    else if ((v3 >= 210) || (v3 < 40)) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    } 
    // 160 to 206: normal
    else {
      tft.setTextColor(ILI9341_WHITE);
    }  
    
    tft.setCursor(130, row1 + 10);
    tft.setTextSize(6);
    tft.print(v3);





    // row 2 left
    tft.setCursor(10, row2 + 30);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("OIL PRESS");

    //oil pressure
    tft.setCursor(10, row2 + 40);
    tft.setTextSize(4);
    tft.print(v9);


    // row 2 right
    tft.setCursor(150, row2 + 30);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("INTAKE");

    // intake temp
    tft.setCursor(130, row2 + 40);
    tft.setTextSize(4);
    tft.setTextColor(ILI9341_WHITE);
    tft.print(v4);


    // row 3 left
    tft.setCursor(10, row3 + 40);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("DAM");

    //dam
    tft.setCursor(10, row3 + 50);
    tft.setTextSize(3);
    if (v5 != 1.00) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(v5);


    // row 3 right
    tft.setCursor(150, row3 + 40);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("BOOST");

    //boost
    tft.setCursor(130, row3 + 50);
    tft.setTextSize(3);
    tft.print(v6);


    // row 4 left
    tft.setCursor(10, row4 + 50);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FEEDBACK KNOCK");

    // feedback knock
    tft.setCursor(10, row4 + 60);
    tft.setTextSize(3);
    if (v1 < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(v10);


    // row 4 right
    tft.setCursor(150, row4 + 50);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("FINE KNOCK");

    //fine knock
    tft.setCursor(130, row4 + 60);
    tft.setTextSize(3);
    if (v2 < 0) {
      tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
    }
    tft.print(v11);
    tft.setTextColor(ILI9341_WHITE);


    //feedback max
    //tft.setCursor(10, row5 + 10);
    //tft.print(v1);

    //fine max
    //tft.setCursor(130, row5 + 10);
    //tft.print(v2);

    tft.setTextSize(2);
    tft.setCursor(130, 280);
    if (v12 == 9999) { tft.print(0); }
    else { tft.print(v12); }
    tft.setCursor(190, 280);
    tft.print(v13);    
  }





  // bottom
  if (ssmActive) {
    tft.setCursor(5,310);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
    tft.print("ACTIVE");
  }
  else {
    tft.setCursor(5,310);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
    tft.print("PASSIVE");
  }

  tft.setCursor(50,310);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
  tft.print("20Hz");
  
  tft.setCursor(85,310);
  tft.setTextSize(1);
  if (displayMode == 0) {
    tft.setTextColor(ILI9341_BLACK, ILI9341_RED);
    tft.print("MODE: UNKNOWN");   
  }
  if (displayMode == 1) {
    tft.setTextColor(ILI9341_BLACK, ILI9341_GREEN);
    tft.print("MODE: NORMAL");   
  }
  if (displayMode == 2) {
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLUE);
    tft.print("MODE: RACE");   
  }


  
  tft.updateScreen();
}


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

    if (verbose) { Serial.println("Sending flow message"); }
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
