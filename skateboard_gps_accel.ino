#include <SoftwareSerial.h>

#include <TinyGPS.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

long timer = 0;
TinyGPS gps;
SoftwareSerial ss(9, 10);
bool first  = true;
float previousLong, previousLat;
double totalDist = 0;
void setup()
{
  Serial.begin(115200);
  ss.begin(9600);
  Wire.begin();
  mpu6050.begin();
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

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
    float flat, flon;
    unsigned long age;
     int year;
  byte month, day, hour, minute, second, hundredths;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    Serial.print("Month: ");
    Serial.print(month);
    Serial.print(" Day: ");
    Serial.print(day);
    Serial.print(" Year: ");
    Serial.print(year);
    Serial.print(" Time: ");
    Serial.print(hour);
    Serial.print(":");
    Serial.print(minute);
    Serial.print(":");
    Serial.print(second);
    Serial.println();
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT: ");
    Serial.print(flat);
    Serial.print(" LON: ");
    Serial.print(flon);
    Serial.print(" SAT: ");
    Serial.print(gps.satellites());
  //  Serial.print(" PREC: ");
  //  Serial.print(gps.hdop());
  Serial.println();
    Serial.print("SPEED: ");
    if (gps.f_speed_kmph() >= 1) {
      Serial.print(gps.f_speed_kmph());
    } else {
      Serial.print("0.00");
    }
    Serial.print("Km/Hr  ");

    Serial.print("ALTITUDE: ");
    Serial.print(gps.f_altitude());
    Serial.print("M  ");
    Serial.print("DISTANCE: ");
    if (first) {
      previousLat = flat;
      previousLong  = flon;
      first = false;
    } else {
      float distance  = gps.distance_between(flat, flon, previousLat, previousLong);
      if (distance > 1 && gps.f_speed_kmph()> 1) {
        totalDist += distance;
        Serial.print(distance);
      } else {
        distance = 0;
        Serial.print(0);
      }
      
      Serial.print("M ");
      Serial.print(" ODOMETER: ");
      Serial.print(totalDist);
      Serial.print("M ");
      previousLat = flat;
      previousLong  = flon;
    }
    Serial.print(" ");
    Serial.print(gps.cardinal(gps.f_course()));
    Serial.println();
  }else{
    Serial.println("NO GPS DATA");
  }

  mpu6050.update();


  Serial.print("temp : "); Serial.println(mpu6050.getTemp());
  Serial.print("accX : "); Serial.print(mpu6050.getAccX());
  Serial.print("  accY : "); Serial.print(mpu6050.getAccY());
  Serial.print("  accZ : "); Serial.println(mpu6050.getAccZ());
  Serial.print("accAngleX : "); Serial.print(mpu6050.getAccAngleX());
  Serial.print("  accAngleY : "); Serial.println(mpu6050.getAccAngleY());

  Serial.println();
  
  delay(2000);
}
