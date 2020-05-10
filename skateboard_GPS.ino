#include <SoftwareSerial.h>

#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial ss(9, 10);
bool first  = true;
float previousLong, previousLat;
double totalDist = 0;
void setup()
{
  Serial.begin(115200);
  ss.begin(9600);
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
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat);
    Serial.print(" LON=");
    Serial.print(flon);
    Serial.print(" SAT=");
    Serial.print(gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop());
    Serial.print("  ");
    if(gps.f_speed_kmph()>=1){
    Serial.print(gps.f_speed_kmph()); 
    }else{
     Serial.print("0.00"); 
      }
    Serial.print("KM/Hr  "); 
    Serial.print(gps.f_altitude()); 
    Serial.print("M  "); 
    if(first){
      previousLat= flat;
      previousLong  = flon;
      first = false;
      }else{
        double distance  = gps.distance_between(flat, flon, previousLat, previousLong);
        if(distance > 1){
        totalDist += distance;
        }else{
          distance = 0;
          }
        Serial.print(distance);
        Serial.print("M ");
        Serial.print(totalDist);
        previousLat = flat;
        previousLong  = flon;
        }
     Serial.print(" ");
     Serial.print(gps.cardinal(gps.f_course()));
  }
  Serial.println();
  gps.stats(&chars, &sentences, &failed);
  if (chars == 0)
    Serial.println("NO DATA");
}
