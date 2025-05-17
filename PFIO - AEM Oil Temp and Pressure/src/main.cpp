#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Wire.h>


#define SERIESRESISTOR 2200 // the value of the 'other' resistor
#define THERMISTORPIN A0 // What pin to connect the sensor to

SoftwareSerial mySerial(A2, A3); // RX, TX


const int sendSerial = 1;  // set whether to actually the results, for testing stuff
 
void setup(void) {
  Serial.begin(115200);
  delay(400);
  if (sendSerial) { 
    mySerial.begin(9600); 
    delay(400); 
  }
}



void loop(void) {

  // read A1, then convert it to voltage based on 5v source
  // using the AEM linear function for oil pressure, calculate the pressure value
  int sensorValue = analogRead(A1);
  float voltage = sensorValue * (5.0 / 1023.0);
  int psi = (37.5*(voltage))-18.75;

  // read A0 pin, then calculate the resistance based on 2.2k resistor
  float reading;
  reading = analogRead(THERMISTORPIN);
  reading = (1023 / reading)  - 1;     // (1023/ADC - 1) 
  reading = SERIESRESISTOR / reading;  // resistor / (1023/ADC - 1)

  // arm is non-linear function, the best I could do is the middle using a function, the rest use a bunch of MAP()'s.  sue me
  // -40 - 140
  if (reading >= 2701) {
    //Serial.print("Manual mapping ");

    if (reading <= 402000 && reading >= 289001) {
      //Serial.println("-40 to -31");
      reading = map(reading, 289001, 402000, -31, -40);
    }
    if (reading <= 289000 && reading >= 210001) {
      //Serial.println("-31 to -22");
      reading = map(reading, 210001, 289000, -22, -31);
    }
    if (reading <= 210000 && reading >= 154001) {
      //Serial.println("-22 to -13");
      reading = map(reading, 154001, 210000, -13, -22);
    }
    if (reading <= 154000 && reading >= 114001) {
      //Serial.println("-13 to -4");
      reading = map(reading, 114001, 154000, -4, -13);
    }
    if (reading <= 114000 && reading >= 85001) {
      //Serial.println("-4 to 5");
      reading = map(reading, 85001, 114000, 5, -4);
    }
    if (reading <= 85000 && reading >= 64301) {
      //Serial.println("5 to 14");
      reading = map(reading, 64301, 85000, 14, 5);
    }
    if (reading <= 64300 && reading >= 48901) {
      //Serial.println("14 to 23");
      reading = map(reading, 48901, 64300, 23, 14);
    }
    if (reading <= 48900 && reading >= 37501) {
      //Serial.println("23 to 32");
      reading = map(reading, 37501, 48900, 32, 23);
    }
    if (reading <= 37500 && reading >= 29001) {
      //Serial.println("32 to 41");
      reading = map(reading, 29001, 37500, 41, 32);
    }
    if (reading <= 29000 && reading >= 22501) {
      //Serial.println("41 to 50");
      reading = map(reading, 22501, 29000, 50, 41);
    }
    if (reading <= 22500 && reading >= 17701) {
      //Serial.println("50 to 59");
      reading = map(reading, 17701, 22500, 59, 50);
    }
    if (reading <= 17700 && reading >= 14001) {
      //Serial.println("59 to 68");
      reading = map(reading, 14001, 17700, 68, 59);
    }
    if (reading <= 14000 && reading >= 11101) {
      //Serial.println("68 to 77");
      reading = map(reading, 11101, 14000, 77, 68);
    }
    if (reading <= 11100 && reading >= 8901) {
      //Serial.println("77 to 86");
      reading = map(reading, 8901, 11100, 86, 77);
    }
    if (reading <= 8900 && reading >= 7201) {
      //Serial.println("86 to 95");
      reading = map(reading, 7201, 8900, 95, 86);
    }
    if (reading <= 7200 && reading >= 5801) {
      //Serial.println("95 to 104");
      reading = map(reading, 5801, 7200, 104, 95);
    }
    if (reading <= 5800 && reading >= 4701) {
      //Serial.println("104 to 113");
      reading = map(reading, 4701, 5800, 113, 104);
    }
    if (reading <= 4700 && reading >= 3901) {
      //Serial.println("113 to 122");
      reading = map(reading, 3901, 4700, 122, 113);
    }
    if (reading <= 3900 && reading >= 3201) {
      //Serial.println("122 to 131");
      reading = map(reading, 3201, 3900, 131, 122);
    }
    if (reading <= 3200 && reading >= 2701) {
      //Serial.println("131 to 140");
      reading = map(reading, 2701, 3200, 140, 131);
    }
  }

  // 140 - 230
  else if (reading <= 2700 && reading >=531 ) {
    //Serial.println("Quadratic calulation");
    reading = (-0.0000000096*reading*reading*reading) + (0.0000635181*reading*reading) - (0.1610960986*reading) + 298.35;
  }

  // 230 - 302
  else {
    //Serial.print("Manual mapping ");

    if (reading <= 531 && reading >= 463) {
      //Serial.println("230 to 239");
      reading = map(reading, 463, 531, 239, 230);
    }
    else if (reading <= 462 && reading >= 404) {
      //Serial.println("239 to 248");
      reading = map(reading, 404, 462, 248, 239);
    }
    else if (reading <= 403 && reading >= 353) {
      //Serial.println("248 to 257");
      reading = map(reading, 353, 403, 257, 248);
    }
    else if (reading <= 352 && reading >= 310) {
      //Serial.println("257 to 266");
      reading = map(reading, 310, 352, 266, 257);
    }
    else if (reading <= 309 && reading >= 273) {
      //Serial.println("267 to 275");
      reading = map(reading, 273, 309, 275, 266);
    }
    else if (reading <= 272 && reading >= 242) {
      //Serial.println("276 to 284");
      reading = map(reading, 242, 272, 284, 275);
    }
    else if (reading <= 241 && reading >= 214) {
      //Serial.println("284 to 293");
      reading = map(reading, 214, 241, 293, 284);
    }
    else if (reading <= 213 && reading >= 189) {
      //Serial.println("293 to 302");
      reading = map(reading, 189, 213, 302, 293);
    }
  }

  // round it off and store it as the oil temperature
  reading = round(reading);
  int temperature = reading;


  // since I don't have a good way of sending 2 int's at once i sent it as a weird string a123,b123
  Serial.print("a");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print("b");
  Serial.println(psi);
  
  if (sendSerial) {
    Serial.println("Sending...");
    mySerial.print("a");
    mySerial.print(temperature);
    mySerial.print(",");
    mySerial.print("b");
    mySerial.println(psi);
  }

  // in my case, this needs to be slower than the time it takes to run a loop on the teensy at the far end
  delay(200);
}

