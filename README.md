# Skateboard_GPS
  -100uF cap required on 5V to maintain stability\
  -Cannot use Hardware I2C for both MPU6050 and SSD1306 as it will cause random crashes due to interference\
  -MPU6050 is put on Software I2C, which SSD1306 is on Hardware I2C
