/***********************************************************************************************************
*  Readout of the MPU 6050                                                 *                               *
*                                                                          *  Comments & Notes             *
*  1.0  2019-12-09  Initial                                                *  Raw values ok.               *                                                                           
*                                                                          *  Zeroing missing              *
*  1.0  2019-12-10  Editorial                                              *                               *                                                                        
***********************************************************************************************************/

/* Titel: ATMega 2560 mit 2 MPU6050 
*
* Version: v1.0
* Autor: Jeremy und Heiko
* Datum: 20.05.2021
*
* Kurze Beschreibung:
* Auslesen der Raw Werte beider Sensoren per I2C
*
* Ausgabe:
* Raw Werte
*
* Anschluss weiterer Elektronik:
* 2x MPU6050 per I2C anschließen, VCC, GND, SCL an PIN 16, SDA an PIN 15 ((ATMega 2560)
* zweiter MPU erhält Spannungsversorgung über AD0 statt über VCC (andere Adresse im I2C Bus)
*
* Changelog:
* 
*  
*/

//################################################################//
//########################## Includes ############################//
//################################################################//

#include  <Wire.h>                                                         // Wire library
#include <Arduino.h>

//################################################################//
//###################### globale Variablen #######################//
//################################################################//

int       I2CaddMPU_Sensor_1    =   0x68;                                           // MPU 6050 VCC (JP3 set)
int       I2CaddMPU_Sensor_2    =   0x69;                                           // MPU 6050 AD0 (JP3 set)
float       accelX;                                                          // Measurement Acceleration X
float       accelY;                                                          // Measurement Acceleration Y
float       accelZ;                                                          // Measurement Acceleration Z
float       gyroX;                                                           // Measurement Gyro Rate X
float       gyroY;                                                           // Measurement Gyro Rate Y
float       gyroZ;                                                           // Measurement Gyro Rate Z  
float     roll;                                                            // Rollangle in °
float     pitch;                                                           // Pitchangle in °
float     yaw;                                                             // Yawangle in °
float     sumsquare;                                                       // sum of X^2+Y^2+Z^2
float     temp;                                                            // measurement of temperature
float     tempC;                                                           // temp -> °C
float     angleX;
float     angleY;
float     angleZ;

float lasttime= 0;

//################################################################//
//######################### Functions ############################//
//################################################################//

// void getAngle(float* pAngle, float* pGyro)
// {
//   float currentTime = float(millis())*1000;
//   pAngle = pGyro * (currentTime - lasttime);
// }

void SetOffset_Sensor_1(char pRegAdress,int pData)
{
  unsigned char dataArray[2]= {0,0};
  dataArray[0] = (pData >> 8) & 0xff;
  dataArray[1] = (pData) & 0xff;
  Wire.beginTransmission(I2CaddMPU_Sensor_1);
  Wire.write(0x06);                                                                                   // Register for Offset
  Wire.write(dataArray[0]);                                                                             // Shifte nach rechts um MSB zu schreiben
  Wire.write(dataArray[1]);                                                                                  // LSB schreiben
  Wire.endTransmission(true);
}


void SetOffset_Sensor_2(char pRegAdress,int pData)
{
  Serial.print("pData: ");
  Serial.println(pData);
  Wire.beginTransmission(I2CaddMPU_Sensor_2);
  Wire.write(pRegAdress);                                                                                   // Register for Offset 
  Wire.write(pData >> 8) & 0xFF;                                                                             // Shifte nach rechts um MSB zu schreiben
  Wire.write(pData) & 0xFF;                                                                                  // LSB schreiben
  Wire.endTransmission(true);
}


void ReadOffset_Sensor_1(char pRegAdress)
{
  float temp;
  Wire.beginTransmission(I2CaddMPU_Sensor_1);
  Wire.requestFrom(pRegAdress,2,true);
  temp = Wire.read()<<8|Wire.read(); 
  Serial.print("x-Gyro Offset zu Beginn: \t");
  Serial.println(temp);
  Wire.endTransmission(true); 
}

void ResetRegisters(char pSensor)
{
  if (pSensor==1)
  {
    int temp;
    Wire.beginTransmission(I2CaddMPU_Sensor_1);
    Wire.requestFrom(0x6B,2,true);
    temp = Wire.read()<<8|Wire.read(); 
    Serial.print("Register 0x6B vor verunden: \t");
    Serial.println(temp);
    Wire.write(0x6B);
    Wire.write(0);                                                                             // Shifte nach rechts um MSB zu schreiben
    Wire.endTransmission(true);                                             // transmission not finished
    Serial.print("Register 0x6B NACH verunden: \t");
    Serial.println(temp);

  }
  else
  {
    int temp;
    Wire.beginTransmission(I2CaddMPU_Sensor_2);
    Wire.requestFrom(0x6B,2,true);
    temp = Wire.read()<<8|Wire.read(); 
    Serial.print("Register 0x6B vor verunden: \t");
    Serial.println(temp);
    Wire.write(0x6B);
    Wire.write(0);                                                                             // Shifte nach rechts um MSB zu schreiben
    Wire.endTransmission(true);                                             // transmission not finished
    Serial.print("Register 0x6B NACH verunden: \t");
    Serial.println(temp);
  }
}



//################################################################//
//########################### SETUP ##############################//
//################################################################//

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
Serial.begin(115200);                                                      // Outputrate Monitor
Wire.begin();                                                            // Start I2C bus
Wire.beginTransmission(I2CaddMPU_Sensor_1);                                       // Start Transmission to MPU6050
Wire.write(0x6B);                                                        // PWR_MGMT_1 register
Wire.write(0);                                                           // setting 0 wakes up MPU 6050
Wire.endTransmission(true);                                              // end of Transmission


/////////////////////////////////////////////
////////// Set Sensitivities ////////////////
/////////////////////////////////////////////

/////////////////////
/////  Sensor 1 /////
/////////////////////

///// Gyro /////
Wire.beginTransmission(I2CaddMPU_Sensor_1);
Wire.write((uint8_t) 0x1B);                                                     // Gyro Sensitivity Register
Wire.write((uint8_t) 0x00);                                                     // 0000 0000 für FS_SEL 0 (+- 250°/s)
Wire.endTransmission();

///// Acc /////
Wire.beginTransmission(I2CaddMPU_Sensor_1);
Wire.write((uint8_t) 0x1C);                                                     // Acc Sensitivity Register
Wire.write((uint8_t) 0x18);                                                     // 0001 1000 für AFS_SEL 3 (+- 16g)
Wire.endTransmission();


/////////////////////
/////  Sensor 2 /////
/////////////////////

///// Gyro /////
// Wire.beginTransmission(I2CaddMPU_Sensor_2);
// Wire.write((uint8_t) 0x1B);                                                     // Gyro Sensitivity Register
// Wire.write((uint8_t) 0x00);                                                     // 0000 0000 für FS_SEL 0 (+- 250°/s)
// Wire.endTransmission();

// ///// Acc /////
// Wire.beginTransmission(I2CaddMPU_Sensor_2);
// Wire.write((uint8_t) 0x1C);                                                     // Acc Sensitivity Register
// Wire.write((uint8_t) 0x00);                                                     // 0000 0000 für AFS_SEL 0 (+- 2g)
// Wire.endTransmission();


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





// lasttime = millis();

// Wire.beginTransmission(I2CaddMPU_Sensor_1);
// Wire.write(0x6B);
// Wire.write(0b00000000);
// Wire.endTransmission();

// ReadOffset_Sensor_1(0x13);
// ResetRegisters(1);

/////////////////////////////////////////////
////////////// Set Offsets //////////////////
/////////////////////////////////////////////

/////////////////////
/////  Sensor 1 /////
/////////////////////

///// Gyro /////
// SetOffset_Sensor_1(0x13,162);
// SetOffset_Sensor_1(0x15,2);
// SetOffset_Sensor_1(0x017,-100);

///// Acc /////

// SetOffset_Sensor_1(0x06,xx);
// SetOffset_Sensor_1(0x08,xx);
// SetOffset_Sensor_1(0x0A,xx);


/////////////////////
/////  Sensor 2 /////
/////////////////////

/// Gyro /////
// SetOffset_Sensor_2(0x13,300);
// SetOffset_Sensor_2(0x15,131);
// SetOffset_Sensor_2(0x017,524);

/// Acc /////

// SetOffset_Sensor_1(0x06,xx);
// SetOffset_Sensor_1(0x08,xx);
// SetOffset_Sensor_1(0x0A,xx);


/// AUslesen OFFSET ACC ///

  // Wire.beginTransmission(I2CaddMPU_Sensor_1);
  // Wire.read(0x77);                                                                                   // Register for Offset
  // Wire.write((char)pData >> 8);                                                                             // Shifte nach rechts um MSB zu schreiben
  // Wire.write((char)pData);                                                                                  // LSB schreiben
  // Wire.endTransmission();


}


//################################################################//
//############################ Loop  #############################//
//################################################################//

void loop() 
/************************************************************************************************************
 *  Auslesen der Acceleration, Temp und Gyro Daten                                                          *
 *  Reg. 0x3B-0x40 Acc, 0x41-0x42 Temp, 0x43-0x48 Gyro                                                      *
 ***********************************************************************************************************/
{
  Wire.beginTransmission(I2CaddMPU_Sensor_1);                                       // -> I2C MPU
  Wire.write(0x3B);                                                        // Start 
  Wire.endTransmission(false);                                             // transmission not finished
  Wire.requestFrom(I2CaddMPU_Sensor_1,14,true);                                     // Pick the 14 registers
  accelX    = Wire.read()<<8|Wire.read();                                  // combine AccX High & Low
  accelY    = Wire.read()<<8|Wire.read();                                  // combine AccY High & Low
  accelZ    = Wire.read()<<8|Wire.read();                                  // combine AccZ High & Low
  temp      = Wire.read()<<8|Wire.read();                                  // combine raw temp High & Low
  gyroX     = Wire.read()<<8|Wire.read();                                  // combine GyroX High  Low
  gyroY     = Wire.read()<<8|Wire.read();                                  // combine GyroY High  Low
  gyroZ     = Wire.read()<<8|Wire.read();                                  // combine GyroZ High  Low  
  tempC     = temp/340.0 + 36.53;                                          // temp °C

  // Serial.print(accelX);
  // Serial.print("\t");  
  // Serial.print(accelY);
  // Serial.print("\t");  
  // Serial.print(accelZ);
  // Serial.print("\t");  

  // Serial.print(gyroX);
  // Serial.print("\t");  
  // Serial.print(gyroY);
  // Serial.print("\t");  
  // Serial.println(gyroZ);

  accelX    = ((float)accelX/2048.0);                                         
  accelY    = ((float)accelY/2048.0);                                               
  accelZ    = ((float)accelZ/2048.0);                                         

  gyroX     = ((float)gyroX)/131;
  gyroY     = ((float)gyroY)/131;
  gyroZ     = ((float)gyroZ)/131;

  // getAngle(&angleX,&gyroX);
  // getAngle(&angleY,&gyroY);
  // getAngle(&angleZ,&gyroZ);


  // sumsquare = sqrt(accelX*accelX+accelY*accelY +accelZ*accelZ);            // Quadratsum
  // accelX    = accelX/sumsquare;                                            // adjust
  // accelY    = accelY/sumsquare;
  // accelZ    = accelZ/sumsquare;
  // roll      = atan2(accelY, accelZ)*180/PI;                                // Roll Angle
  // pitch     = -asin(accelX)*180/PI;                                        // Pitch Angle
  // Serial.print(" accelX:  ");
  // Serial.println((accelX*1000),6);
  // Serial.print(" Roll  :  ");
  // Serial.print(roll,4);
  // Serial.print(" Pitch :  ");
  // Serial.print(pitch,4);
  // Serial.print(" Temp  :  ");
  // Serial.print(tempC,4);

  Serial.print(accelX);
  Serial.print("\t");  
  Serial.print(accelY);
  Serial.print("\t");  
  Serial.print(accelZ);
  Serial.print("\t");  

  Serial.print(gyroX);
  Serial.print("\t");  
  Serial.print(gyroY);
  Serial.print("\t");  
  Serial.println(gyroZ);
  

  // Serial.print(angleX);
  // Serial.print("\t");  
  // Serial.print(angleY);
  // Serial.print("\t");  
  // Serial.println(angleZ);


   delay(500);
  // lasttime = millis();
}