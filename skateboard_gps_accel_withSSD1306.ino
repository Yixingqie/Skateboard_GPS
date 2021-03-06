#include <SoftwareSerial.h>
#include "LowPower.h"
#include <TinyGPS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
double AcX;
double  AcY;
double  AcZ;
double  Tmp;
const int MPU = 0x68;
double maxAcc = 0;
int dots  = 0;

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
  display.setTextSize(2); //8.5
  display.setTextColor(WHITE);
  display.setCursor(1, 10);
  display.println("  YIXING");
  display.println("    QIE");
  display.println(" FW: 1.00");
  display.display();
  display.clearDisplay();
  display.clearDisplay();
  delay(5000);

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
      display.print(month);
      display.print("-");
      display.print(day);
      display.print("-");
      display.print(year);

      int x;
      //Serial.println(hour);
      display.print("    ");
      switch (hour) {
        case 1:
          x = 9;
          break;
        case 2:
          x = 10;
          break;
        case 3:
          x = 11;
          break;
        case 4:
          x = 12;
          break;
        case 5:
          x = 1;
          break;
        case 6:
          x = 2;
          break;
        case 7:
          x = 3;
          break;
        case 8:
          x = 4;
          break;
        case 9:
          x = 5;
          break;
        case 10:
          x = 6;
          break;
        case 11:
          x = 7;
          break;
        case 12:
          x = 8;
          break;
        case 13:
          x = 9;
          break;
        case 14:
          x = 10;
          break;
        case 15:
          x = 11;
          break;
        case 16:
          x = 12;
          break;
        case 17:
          x = 1;
          break;
        case 18:
          x = 2;
          break;
        case 19:
          x = 3;
          break;
        case 20:
          x = 4;
          break;
        case 21:
          x = 5;
          break;
        case 22:
          x = 6;
          break;
        case 24:
          x = 7;
          break;
        case 0:
          x = 8;
          break;
      }
      display.print(x);
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
      if ((AcX * 9.81) > maxAcc) {
        maxAcc  = AcX * 9.81;
      }
      display.print(maxAcc);
      display.println(" M/S^2");
      // display.print("SPD: ");
      if ( abs(AcX) > 0.1) {
        double sp = gps.f_speed_kmph();
        //   display.print(sp);
        if (sp > maxSp) {
          maxSp = sp;
        }
      } else {
        //     display.print("0.00");
      }
      //    display.println(" Km/Hr  ");
      //   display.println();
      display.print("Vi: ");
      display.print((analogRead(A6) / 1023.0) * 5.0);
      display.print("V");
      display.print(" Vb: ");
      display.print(((analogRead(A0) / 1023.0) * 5.0) * (42.0 / 3.79));
      display.println("V");
      display.print("ALT: ");
      display.print((gps.f_altitude()));
      display.println(" M");
      display.print("MAX: ");
      display.print(maxSp);
      double distance  = gps.distance_between(flat, flon, previousLat, previousLong) / 1000.0;
      if (abs(AcX) > 0.1) {
        totalDist = (totalDist + distance);
        // display.print(distance);
      } else {
        distance = 0;
        //   display.print("0.00");
      }

      display.println(" Km/Hr ");
      display.print("ODO: ");
      display.print(totalDist);
      display.print(" KM ");
      previousLat = flat;
      previousLong  = flon;
      //
      //      display.print(" ");
      //     // display.print(gps.cardinal(gps.f_course()));
      //      display.println();
    }
  } else {
    display.println("NO GPS SIGNAL");
    display.println();
    display.print("AccY: ");
    display.println(AcY);
    display.print("AccX: ");
    display.println(AcX);
    display.print("AccZ: ");
    display.println(AcZ);
    display.print("Tmp: ");
    display.print(Tmp);
    display.println("C");
    display.print("Vi: ");
    display.print((analogRead(A6) / 1023.0) * 5.0);
    display.print("V");
    display.print(" Vb: ");
    display.print(((analogRead(A0) / 1023.0) * 5.0) * (42.0 / 3.79));
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


  display.display();
  display.clearDisplay();
  display.clearDisplay();
 // LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF); //put to sleep
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
