//By YIXING QIE
#include <SoftwareSerial.h>
#include <SlowSoftI2CMaster.h>
#include "LowPower.h"
#include <TinyGPS.h>
//#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
TinyGPS gps;
SoftwareSerial ss(9, 10);

const int sda = 4;
const int scl =5;
SlowSoftI2CMaster si = SlowSoftI2CMaster(sda, scl, true);
uint8_t Buf[8];
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
uint8_t read_i2c(uint8_t address, uint8_t reg);
void initialize_i2c();

void setup()
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  ss.begin(9600);

  si.i2c_init();
  si.i2c_start((MPU << 1) | I2C_WRITE);
  si.i2c_write(0x6B);
  si.i2c_write(0);
  si.i2c_stop();

  si.i2c_init();
  si.i2c_start((MPU << 1) | I2C_WRITE);
  si.i2c_write(0x6C);
  si.i2c_write(0x07);
  si.i2c_stop();

  display.setTextSize(2); //8.5
  display.setTextColor(WHITE);
  display.setCursor(1, 10);
  display.println("  YIXING");
  display.println("    QIE");
  display.println(" FW: 3.00");
  display.display();
  display.clearDisplay();
  //  EEPROM.put(address0, 0.00f);
  // EEPROM.put(address1, 0.00f);
  EEPROM.get(address0, totalDist);
  EEPROM.get(address1, mSpeed);
  // totalDist =
  delay(5000);

}

void loop()
{
  initialize_i2c();
  float vi = ((analogRead(A6) / 1023.0) * 5.0) - 0.1;
  float vb = (((analogRead(A0) / 1023.0) * 5.0) * (41.8 / 3.81)) - 0.1;
  display.setTextColor(WHITE);
  display.setTextSize(1); //8.5
  display.setCursor(0, 0);
  bool newData = false;

  readAcc();

  for (unsigned long start = millis(); millis() - start < 1000;)
  {

    while (ss.available())
    {
      char c = ss.read();
      if (gps.encode(c)) {
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
    if ((AcX * 9.81) > maxAcc && (AcZ > 0.8)) {
      maxAcc  = AcX * 9.81;
    }

    if ( abs(AcX) > sens) {
      double sp = gps.f_speed_kmph();
      //   display.print(sp);
      if (sp > maxSp) {
        maxSp = sp;
      }
      if (sp > mSpeed) {
        mSpeed = sp;
      }
    }


    double distance  = gps.distance_between(flat, flon, previousLat, previousLong) / 1000.0;
    if (abs(AcX) > sens) {
      totalDist = (totalDist + distance);
      currDist = (currDist + distance);
      // display.print(distance);
    } //else {
    // distance = 0;
    //   display.print("0.00");
    //  }


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


      display.print("[");
      // display.print("LT: ");
      display.print(flat);
      display.print(", ");
      //display.print("  LN: ");
      display.print(flon);
      display.print("]");

      display.print("  #:");
      display.println(gps.satellites());


      display.print("T: ");
      display.print((int)(Tmp));
      display.print("C");

      display.print("     MAcc: ");

      display.println(maxAcc, 1);

      display.print("Vi: ");
      display.print(vi, 1);
      display.print("V");
      display.print("    Vb: ");
      display.print(vb, 1);
      display.println("V");

      display.print("CurrSpd: ");
      display.print(maxSp, 1);
      display.println(" Km/Hr ");


      display.print("Curr: ");
      display.print(currDist);
      display.println(" Km");

      display.print("TotalDist: ");
      display.print(totalDist);
      display.println(" Km");

      display.print("TotalSpd: ");
      display.print(mSpeed, 1);
      display.println(" Km/Hr");
      updateSpeed();
      updateDist();
      previousLat = flat;
      previousLong  = flon;
    }
  } else {
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
    display.print(AcY * 9.8, 1);
    display.println(" m/s^2");
    display.print("AccX: ");
    display.print(AcX * 9.8, 1);
    display.println(" m/s^2");
    display.print("AccZ: ");
    display.print(AcZ * 9.8, 1);
    display.println(" m/s^2");
    display.print("Vi: ");
    display.print(vi, 1);
    display.print("V");
    display.print("    Vb: ");
    display.print(vb, 1);
    display.println("V");
    display.print("Searching");
    if (dots == 0) {
      display.print("");
      dots++;
    } else if (dots == 1) {
      display.print("*");
      dots++;
    } else if (dots == 2) {
      display.print("**");
      dots++;
    } else {
      display.print("***");
      dots = 0;
    }
  }


  display.display();
  display.clearDisplay();
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); //put to sleep
}
uint8_t read_i2c(uint8_t address, uint8_t reg) {

  si.i2c_start((address << 1) | I2C_WRITE);
  si.i2c_write(reg);
  si.i2c_rep_start((address << 1) | I2C_READ);
  uint8_t val = si.i2c_read(true);
  si.i2c_stop();
  return val;
}

void readAcc() {
  Buf[0] = read_i2c(MPU, 0x3B);
  Buf[1] = read_i2c(MPU, 0x3C);
  Buf[2] = read_i2c(MPU, 0x3D);
  Buf[3] = read_i2c(MPU, 0x3E);
  Buf[4] = read_i2c(MPU, 0x3F);
  Buf[5] = read_i2c(MPU, 0x40);
  Buf[6] = read_i2c(MPU, 0x41);
  Buf[7] = read_i2c(MPU, 0x42);
  AcX = (Buf[0] << 8 | Buf[1]) / 16384.0;
  AcY = (Buf[2] << 8 | Buf[3]) / 16384.0;
  AcZ = (Buf[4] << 8 | Buf[5]) / 16384.0;
  Tmp = ((Buf[6] << 8 | Buf[7]) + 12412.0) / 340.0;

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

void initialize_i2c() {
  pinMode(scl, OUTPUT);
  pinMode(sda, OUTPUT);
  digitalWrite(scl, HIGH);
  digitalWrite(sda, HIGH);

}
