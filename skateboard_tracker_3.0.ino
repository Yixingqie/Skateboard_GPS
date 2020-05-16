#include <SoftwareSerial.h>
#include <SlowSoftI2CMaster.h>
//#include "LowPower.h"
#include <TinyGPS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
//By YIXING QIE
int sda = 4;
int scl = 5;
bool sdaOn = false;
bool sclOn = false;

void initialize_i2c() {
  pinMode(scl, OUTPUT);
  pinMode(sda, OUTPUT);
  digitalWrite(sda, HIGH);
   digitalWrite(scl, HIGH);
  //PIND = _BV(PB3); //sda
  //PIND = _BV(PB5); //scl
  sdaOn = true;
  sclOn = true;

}

uint8_t Buf[8];

SlowSoftI2CMaster si = SlowSoftI2CMaster(4, 5, true);

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

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

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
//
//_Bool write_Reg(unsigned char address, unsigned char reg, unsigned char val){
// _Bool check = 0;
//             start_i2c();//Start connecting to Slave
//             address_Masterwrite(address); //Asks to write to slave
//             i2c_wait_ack(); //waits for slave acknowledgment
//             write_i2c(reg); 
//             i2c_wait_ack(); 
//             write_i2c(val); 
//             check = i2c_wait_ack();//waits for slave acknowledgment
//             stop_i2c();
//             return check;
//}
//unsigned char read_Reg(unsigned char address, unsigned char reg){
//unsigned char val;
//
//               start_i2c();//Start connecting to slave device
//               address_Masterwrite(address); //asks to write the the slave
//               i2c_wait_ack();//waits for slave acknowledgment
//               write_i2c(reg);
//               i2c_wait_ack();//waits for slave acknowledgment
//             repeated_start(); //allows for switch into read mode
//             address_MasterRead(address); //Ask to read from the slave
//             i2c_wait_ack(); //waits for slave acknowledgment
//             val = read_i2c(1);
//             stop_i2c();  
//             return val;
//}
void setup()
{

  display.clearDisplay();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  display.clearDisplay();
  ss.begin(9600);

   si.i2c_init();
  si.i2c_start((MPU<<1)|I2C_WRITE);
  si.i2c_write(0x6B);
  si.i2c_write(0);
  si.i2c_stop();
  
  si.i2c_init();
  si.i2c_start((MPU<<1)|I2C_WRITE);
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
  display.clearDisplay();
  //  EEPROM.put(address0, 0.00f);
  // EEPROM.put(address1, 0.00f);
  EEPROM.get(address0, totalDist);
  EEPROM.get(address1, mSpeed);
  // totalDist =
  delay(3000);

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
    } else {
      distance = 0;
      //   display.print("0.00");
    }


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
  // Wire.endTransmission(true);

  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); //put to sleep
  // delay(100);
}

void readAcc() {
 si.i2c_start((MPU<<1)|I2C_WRITE);
  si.i2c_write(0x3B);
  si.i2c_rep_start((MPU<<1)|I2C_READ);
  Buf[0]=si.i2c_read(true);
  si.i2c_stop();
  
  si.i2c_start((MPU<<1)|I2C_WRITE);
  si.i2c_write(0x3C);
  si.i2c_rep_start((MPU<<1)|I2C_READ);
  Buf[1]=si.i2c_read(true);
  si.i2c_stop();

   si.i2c_start((MPU<<1)|I2C_WRITE);
  si.i2c_write(0x3D);
  si.i2c_rep_start((MPU<<1)|I2C_READ);
  Buf[2]=si.i2c_read(true);
  si.i2c_stop();

   si.i2c_start((MPU<<1)|I2C_WRITE);
  si.i2c_write(0x3E);
  si.i2c_rep_start((MPU<<1)|I2C_READ);
  Buf[3]=si.i2c_read(true);
  si.i2c_stop();

   si.i2c_start((MPU<<1)|I2C_WRITE);
  si.i2c_write(0x3F);
  si.i2c_rep_start((MPU<<1)|I2C_READ);
  Buf[4]=si.i2c_read(true);
  si.i2c_stop();

   si.i2c_start((MPU<<1)|I2C_WRITE);
  si.i2c_write(0x40);
  si.i2c_rep_start((MPU<<1)|I2C_READ);
  Buf[5]=si.i2c_read(true);
  si.i2c_stop();

   si.i2c_start((MPU<<1)|I2C_WRITE);
  si.i2c_write(0x41);
  si.i2c_rep_start((MPU<<1)|I2C_READ);
  Buf[6]=si.i2c_read(true);
 si.i2c_stop();

   si.i2c_start((MPU<<1)|I2C_WRITE);
  si.i2c_write(0x42);
  si.i2c_rep_start((MPU<<1)|I2C_READ);
Buf[7]=si.i2c_read(true);
  si.i2c_stop();
 
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
