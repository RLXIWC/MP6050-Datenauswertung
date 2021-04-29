/* Titel: ATMega 2560 mit 2 MPU6050 
*
* Version: v1.0
* Autor: Jeremy und Heiko
* Datum: 29.04.2021
*
* Kurze Beschreibung:
* Auslesen beider Sensoren und abspeichern als Quaternionen
* Erste Versuche der Fusionierung
*
* Ausgabe:
* Quaternionen und Produkt
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

#include <Arduino.h>
#include "I2Cdev.h"                                                                                                     // Library für I2C Sensor MPU6050 und anderen Sensoren
#include "MPU6050_6Axis_MotionApps_V6_12.h"                                                                             // MPU6050 Library von I2CDev
#include "Wire.h"                                                                                                       // Nötig für I2C Bus

//// Sensorobjekte ////

MPU6050 Sensor_1(MPU6050_ADDRESS_AD0_LOW);                                                                              // Standard Adresse 0x68  -  ADO0 Low ist ein define in der MPU6050.h
MPU6050 Sensor_2(MPU6050_ADDRESS_AD0_HIGH);                                                                             // Parameter ist neue Adresse 0x69   -   ebenso ein define

//################################################################//
//########################## Defines #############################//
//################################################################//


///// I2C Pins zuweisen /////  
#define SDA_Pin 20;                                                                                                     // Seriell Data Pin
#define SCL_Pin 21;                                                                                                     // Seriell Clock Pin (Entwicklungsboard aber anderst verdrahtet)



//################################################################//
//###################### globale Variablen #######################//
//################################################################//

///// DMP /////
bool DMP_Status_Sensor_1 = false;                                                                                       // Status des Digital Motion Processor des Sensor 1 (zu Beginn auf false - nicht init)
bool DMP_Status_Sensor_2 = false;                                                                                       // Status des Digital Motion Processor des Sensor 2    

///// Data Array aus FIFO /////
char Data_Array_Sensor_1[64];                                                                                           // hier werden die Bytes aus dem FIFO von Sensor 1 eingelesen (kommen als HEx Werte an)                                                                                  
char Data_Array_Sensor_2[64];                                                                                           // hier werden die Bytes aus dem FIFO von Sensor 2 eingelesen 

///// Beschleunigungswerte /////
VectorInt16 Acc_Values_Sensor_1;                                                                                        // [x, y, z] Beschleunigungs Werte Sensor 1 - Cotr ohne Parameter setzt alle Werte auf 0
VectorInt16 Acc_Values_Sensor_2;                                                                                        // [x, y, z] Beschleunigungs Werte Sensor 2 - Cotr ohne Parameter setzt alle Werte auf 0

///// Drehraten - Drehgeschwindigkeit /////
float Drehraten_Sensor_1[3];                                                                                            // speichert die Drehraten / Drehgeschwindigkeit um alle 3 Achsen in Grad/sec von Sensor 1
float Drehraten_Sensor_2[3];                                                                                            // speichert die Drehraten / Drehgeschwindigkeit um alle 3 Achsen in Grad/sec von Sensor 2

//################################################################//
//########################### SETUP ##############################//
//################################################################//

void setup()
{
 
}

//################################################################//
//############################ Loop  #############################//
//################################################################//

void loop()
{
  
}