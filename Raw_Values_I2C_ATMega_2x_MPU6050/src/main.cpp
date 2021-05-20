

/***********************************************************************************************************
*  Readout of the MPU 6050                                                 *                               *
*                                                                          *  Comments & Notes             *
*  1.0  2019-12-09  Initial                                                *  Raw values ok.               *                                                                           
*                                                                          *  Zeroing missing              *
*  1.0  2019-12-10  Editorial                                              *                               *                                                                        
***********************************************************************************************************/
#include  <Wire.h>                                                         // Wire library
#include <Arduino.h>
int       I2CaddMPU    =   0x69;                                           // MPU 6050 AD0 (JP3 set)
float     accelX;                                                          // Measurement Acceleration X
float     accelY;                                                          // Measurement Acceleration Y
float     accelZ;                                                          // Measurement Acceleration Z
float     gyroX;                                                           // Measurement Gyro Rate X
float     gyroY;                                                           // Measurement Gyro Rate Y
float     gyroZ;                                                           // Measurement Gyro Rate Z  
float     roll;                                                            // Rollangle in °
float     pitch;                                                           // Pitchangle in °
float     yaw;                                                             // Yawangle in °
float     sumsquare;                                                       // sum of X^2+Y^2+Z^2
float     temp;                                                            // measurement of temperature
float     tempC;                                                           // temp -> °C

void setup() 
/***********************************************************************************************************
 *  Initialisierung und Kalibrierung der MPU 6050                                                          *
 *  Die Kalibrierung wird nur einmal für die MPU 6050 durchgefahren                                        *
 **********************************************************************************************************/
{
/**************************************************************************
 *  Wakeup & Test-Monitor                                                 *
 *  Reg. 0x6B -> 0                                                        *
 *************************************************************************/
  Serial.begin(9600);                                                      // Outputrate Monitor
  Wire.begin();                                                            // Start I2C bus
  Wire.beginTransmission(I2CaddMPU);                                       // Start Transmission to MPU6050
  Wire.write(0x6B);                                                        // PWR_MGMT_1 register
  Wire.write(0);                                                           // setting 0 wakes up MPU 6050
  Wire.endTransmission(true);                                              // end of Transmission
/**************************************************************************
 *  Calibration                                                           *
 *************************************************************************/

/**************************************************************************
 *  Set Resolution Acc and Gyro                                           *
 *  Later the Resolution should auto adjust                               *
 *  Reg. 1B Gyro, 1C Acc                                                  *
 *  Bit 7 6 5 4 3 2 1 0       Gyro FS      Acc FS                         *
 *      0 0 0 0 0 0 0 0       +/-250°/s    +/-2g                          *
 *      0 0 0 0 1 0 0 0       +/-500°/s    +/-4g                          *
 *      0 0 0 1 0 0 0 0       +/-1000°/s   +/-8g                          *
 *      0 0 0 1 1 0 0 0       +/-2000°/s   +/-16g                         *
 *  Bit 7 -5 are the selft test activation bit and should always be 0     *
 *************************************************************************/
}

void loop() 
/************************************************************************************************************
 *  Auslesen der Acceleration, Temp und Gyro Daten                                                          *
 *  Reg. 0x3B-0x40 Acc, 0x41-0x42 Temp, 0x43-0x48 Gyro                                                      *
 ***********************************************************************************************************/
{
  Wire.beginTransmission(I2CaddMPU);                                       // -> I2C MPU
  Wire.write(0x3B);                                                        // Start 
  Wire.endTransmission(false);                                             // transmission not finished
  Wire.requestFrom(I2CaddMPU,14,true);                                     // Pick the 14 registers
  accelX    = Wire.read()<<8|Wire.read();                                  // combine AccX High & Low
  accelY    = Wire.read()<<8|Wire.read();                                  // combine AccY High & Low
  accelZ    = Wire.read()<<8|Wire.read();                                  // combine AccZ High & Low
  temp      = Wire.read()<<8|Wire.read();                                  // combine raw temp High & Low
  gyroX     = Wire.read()<<8|Wire.read();                                  // combine GyroX High  Low
  gyroY     = Wire.read()<<8|Wire.read();                                  // combine GyroY High  Low
  gyroZ     = Wire.read()<<8|Wire.read();                                  // combine GyroZ High  Low  
  tempC     = temp/340.0 + 36.53;                                          // temp °C
  accelX    = accelX/pow(2,14);                                            // scaling 2^14
  accelY    = accelY/pow(2,14);                                            // scaling 2^14       
  accelZ    = accelZ/pow(2,14);                                            // scaling 2^14
  sumsquare = sqrt(accelX*accelX+accelY*accelY +accelZ*accelZ);            // Quadratsum
  accelX    = accelX/sumsquare;                                            // adjust
  accelY    = accelY/sumsquare;
  accelZ    = accelZ/sumsquare;
  roll      = atan2(accelY, accelZ)*180/PI;                                // Roll Angle
  pitch     = -asin(accelX)*180/PI;                                        // Pitch Angle
  Serial.print(" accelX:  ");
  Serial.println((accelX*1000),6);
  Serial.print(" Roll  :  ");
  Serial.print(roll,4);
  Serial.print(" Pitch :  ");
  Serial.print(pitch,4);
  Serial.print(" Temp  :  ");
  Serial.print(tempC,4);

  delay(100);
}