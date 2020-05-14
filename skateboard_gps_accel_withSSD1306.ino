#include <SoftwareSerial.h>
//#include "LowPower.h"
#include <TinyGPS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
double AcX;
double  AcY;
double  AcZ;
double  Tmp;
const int MPU = 0x68;

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

TinyGPS gps;
SoftwareSerial ss(9, 10);

double previousLong, previousLat;
double totalDist = 0;
byte first  = 1;
double maxSp = 0;

void readAcc();
void setup()
{
  // Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
  display.clearDisplay();
  display.clearDisplay();
  ss.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop()
{
  readAcc();
  float flat, flon;
  unsigned long age;
  display.setTextSize(1); //8.5
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  bool newData = false;

  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      if (gps.encode(c))
        newData = true;
    }
  }

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
    int year;
    byte month, day, hour, minute, second;
    byte temp;
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &temp , &age);
    if (first) {
      previousLat = flat;
      previousLong  = flon;
      first = 0;
    } else {
      display.print((String)month);
      display.print("-");
      display.print((String)day);
      display.print("-");
      display.print((String)year);


      display.print("    ");
      display.print(abs(hour - 12));
      display.print(":");
      display.print(minute);
      display.print(":");
      display.println(second);


      //      display.println();
      display.print("LT: ");
      display.print((double)flat);
      display.print("  LN: ");
      display.println((double)flon);
     // display.println();
      display.print("SAT: ");
      display.print(gps.satellites());
      
      display.print("  ");

      display.print("    T: ");
      display.print(Tmp - 6);
      display.println("C");

      display.print("Acc: ");
      display.print(AcY * 9.81);
      display.println(" M/S^2");
      display.print("SPD: ");
      if ( abs(AcY) > 0.1) {
        double sp = gps.f_speed_kmph();
        display.print(sp);
        if (sp > maxSp) {
          maxSp = sp;
        }
      } else {
        display.print("0.00");
      }
      display.println("Km/Hr  ");
      //   display.println();
      
      display.print("ALT: ");
      display.print((int)(gps.f_altitude()));
      display.println("M  ");
      display.print("MAX: ");
      display.print(maxSp);
      double distance  = gps.distance_between(flat, flon, previousLat, previousLong) / 1000.0;
      if (abs(AcY) > 0.1) {
        totalDist = (totalDist + distance);
        // display.print(distance);
      } else {
        distance = 0;
        //   display.print("0.00");
      }

      display.println("Km/Hr ");
      display.print("ODO: ");
      display.print(totalDist);
      display.print("KM ");
      previousLat = flat;
      previousLong  = flon;
      //
      //      display.print(" ");
      //     // display.print(gps.cardinal(gps.f_course()));
      //      display.println();
    }
  } else {
    display.println("NO GPS DATA");
    display.print("AccY: ");
    display.println(AcY);
    display.print("AccX: ");
    display.println(AcX);
    display.print("AccZ: ");
    display.println(AcZ);
    display.print("Tmp: ");
    display.print(Tmp);
  }


  display.display();
  display.clearDisplay();
  display.clearDisplay();
  //   LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_ON); //put to sleep
  // delay(1000);
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
