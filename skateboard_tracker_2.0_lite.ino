#include <SoftwareSerial.h>
//#include "LowPower.h"
#include <TinyGPS.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire display;

// constants won't change. Used here to set a pin number:
const int ledPin =  LED_BUILTIN;// the number of the LED pin

// Variables will change:
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)

const byte address0 = 0;
const byte address1 = 1;
double AcX;
double  AcY;
double  AcZ;
double  Tmp;
const int MPU = 0x68;
double maxAcc = 0;
byte dots  = 0;
const float sens = 0.05;

//#define OLED_RESET 4
//Adafruit_SSD1306 display(OLED_RESET);

TinyGPS gps;
SoftwareSerial ss(9, 10);

float previousLong, previousLat;
float totalDist;
float mSpeed;
float currDist = 0;
byte first  = 1;
float maxSp = 0;

void readAcc();
byte hour_convert(int hour);
void updateDist();
void updateSpeed();
void setup()
{
  pinMode(ledPin, OUTPUT);
  // Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);

#if RST_PIN >= 0
  display.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  display.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0
display.setFont(System5x7);
  display.clear();
  
  ss.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x6C);  // PWR_MGMT_1 register
  Wire.write(0x07);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  

  display.println("  YIXING");
  display.println("    QIE");
  display.println(" FW: 2.00");
  

  //  EEPROM.put(address0, 0.00f);
  // EEPROM.put(address1, 0.00f);
  EEPROM.get(address0, totalDist);
  EEPROM.get(address1, mSpeed);
  // totalDist =
  delay(5000);
  display.clear();

}

void loop()
{
  display.setFont(System5x7);
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
  float vi = ((analogRead(A6) / 1023.0) * 5.0) - 0.1;
  float vb = (((analogRead(A0) / 1023.0) * 5.0) * (41.8 / 3.81)) - 0.1;
 // display.setTextColor(WHITE);
 // display.setTextSize(1); //8.5
 // display.setCursor(0, 0);
  bool newData = false;

  readAcc();
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
   {
      char c = ss.read();
      if (gps.encode(c)){
        newData = true;
      }
    }
  }
   
  if (newData) {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    int year;
    byte month, day, hour, minute, second;
    byte temp;
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &temp , &age);
    double sp = gps.f_speed_kmph();
    double distance  = gps.distance_between(flat, flon, previousLat, previousLong) / 1000.0;


    if ((AcX * 9.81) > maxAcc && (AcZ > 0.8)) {
        maxAcc  = AcX * 9.81;
      }

       if ( abs(AcX) > sens) {
        
        if (sp > maxSp) {
          maxSp = sp;
        }
        if (sp > mSpeed) {
          mSpeed = sp;
        }
      } 

      if (abs(AcX) > sens) {
        totalDist = (totalDist + distance);
        currDist = (currDist + distance);
      } else {
        distance = 0;
      }

     display.clear();
    if (first) {
      previousLat = flat;
      previousLong  = flon;
      first = 0;
    } else {
      display.print(month);
      display.print("-");
      display.print(day);
      display.print("-");
      display.print(year);

      display.print("    ");
      display.print(hour_convert(hour));
      display.print(":");
      display.print(minute);
      display.print(":");
      display.println(second);


      //      display.println();
      display.print("[");
      // display.print("LT: ");
      display.print(flat);
      display.print(", ");
      //display.print("  LN: ");
      display.print(flon);
      display.print("]");
      // display.println();
      display.print("  #: ");
      display.println(gps.satellites());

      //  display.print("  ");

      display.print("T: ");
      display.print((int)(Tmp));
      display.print("C");

      display.print("     MAcc: ");
      
      display.println(maxAcc, 1);
      // display.println(" M/S^2");
      // display.print("SPD: ");
     
      //    display.println(" Km/Hr  ");
      //   display.println();
      display.print("Vi: ");
      display.print(vi, 1);
      display.print("V");
      display.print("    Vb: ");
      display.print(vb, 1);
      display.println("V");
      // display.print("ALT: ");
      // display.print((gps.f_altitude()));
      //display.print("M");
      display.print("CurrSpd: ");
      display.print(maxSp, 1);
      display.println(" Km/Hr ");

      display.print("Curr: ");
      display.print(currDist);
      display.println(" Km");

      display.print("TotalDist: ");
      display.print(totalDist);
      display.println(" Km");
      previousLat = flat;
      previousLong  = flon;
      
      display.print("TotalSpd: ");
      display.print(mSpeed, 1);
      display.println(" Km/Hr");
      updateSpeed();
      updateDist();


    }
  } else {
    float y = AcY * 9.8;
    float x = AcX * 9.8;
    float z = AcZ * 9.8;
    display.clear();
    display.print("NO GPS SIGNAL");
    display.print("  T: ");
    display.print((int)Tmp);
    display.println("C");
    display.print("TotalDist: ");
    display.print(totalDist);
    display.println(" Km");
    display.print("TotalSpd: ");
    display.print(mSpeed, 1);
    display.println(" Km/Hr");
    display.print("AccY: ");
    display.print(y, 1);
    display.println(" m/s^2");
    display.print("AccX: ");
    display.print(x, 1);
    display.println(" m/s^2");
    display.print("AccZ: ");
    display.print(z, 1);
    display.println(" m/s^2");
    display.print("Vi: ");
    display.print(vi, 1);
    display.print("V");
    display.print("    Vb: ");
    display.print(vb, 1);
    display.println("V");
    display.print("Searching");
    if (dots == 0) {
      display.print(".");
      dots++;
    } else if (dots == 1) {
      display.print("..");
      dots++;
    } else {
      display.print("...");
      dots = 0;
    }
  }


 // display.display();
 // display.clearDisplay();
 // display.clearDisplay();

  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); //put to sleep
  // delay(100);
}

void readAcc() {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU, 8, true);
  AcX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AcY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AcZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
  Tmp = ((Wire.read() << 8 | Wire.read()) + 12412.0) / 340.0;
}

byte hour_convert(int hour) {
  if (hour >= 5  && hour <= 16) {
    return hour - 4;
  } else if (hour >= 17 && hour <= 24) {
    return hour - 16;
  } else if (hour >= 0 && hour <= 4) {
    return hour + 8;
  }

}
void updateDist() {
  if (AcZ < -0.8) {
    float temp;
    EEPROM.get(address0, temp);
    if (temp != totalDist) {
      EEPROM.put(address0, totalDist);
    }
  }
}

void updateSpeed() {
  if (AcZ < -0.8) {
    float temp;
    EEPROM.get(address1, temp);
    if (temp != mSpeed) {
      EEPROM.put(address1, mSpeed);
    }
  }
}
