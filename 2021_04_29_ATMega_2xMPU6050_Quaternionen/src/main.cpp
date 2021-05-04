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
uint8_t Data_Array_Sensor_1[64];                                                                                           // hier werden die Bytes aus dem FIFO von Sensor 1 eingelesen (kommen als Hex Werte an)                                                                                  
uint8_t Data_Array_Sensor_2[64];                                                                                           // hier werden die Bytes aus dem FIFO von Sensor 2 eingelesen 


///// Beschleunigungswerte /////
VectorInt16 Acc_Values_Sensor_1;                                                                                        // [x, y, z] Beschleunigungs Werte Sensor 1 - Cotr ohne Parameter setzt alle Werte auf 0
VectorInt16 Acc_Values_Sensor_2;                                                                                        // [x, y, z] Beschleunigungs Werte Sensor 2 - Cotr ohne Parameter setzt alle Werte auf 0

///// Drehraten - Drehgeschwindigkeit /////
VectorInt16 Gyro_Values_Sensor_1;                                                                                          // speichert die Drehraten / Drehgeschwindigkeit um alle 3 Achsen in Grad/sec von Sensor 1
VectorInt16 Gyro_Values_Sensor_2;                                                                                          // speichert die Drehraten / Drehgeschwindigkeit um alle 3 Achsen in Grad/sec von Sensor 2



//################################################################//
//######################### Functions ############################//
//################################################################//

void HextoDezimal(int *Dezimal_output, const char* Hex_input) 
{

}


//################################################################//
//########################### SETUP ##############################//
//################################################################//

void setup()
{
    Wire.begin();                                                                                                       // Starte I2C
    Wire.setClock(400000);                                                                                              // Übertragungsgeschwindigkeit auf 400 kHz setzen (Fastwire)

    Serial.begin(115200);                                                                                               // Konsolen ausgabe ermöglichen
    Serial.println("Initalisieren der Sensoren ueber I2C ....");
    Sensor_1.initialize();                                                                                              // legt fest: clock Quelle für die Gyro -> Winkel Berechnung  // MPU sleep deaktivieren // g Range auf +- 2g // Gyro Range auf +# 250° / sec
    Sensor_2.initialize();

    /// Verbingungstest Sensor 1 ///
    if(Sensor_1.testConnection())
    {
        Serial.println("Verbindung zum ersten Sensor erfogreich");
    }
    else
    {
        Serial.println("Verbindung zum ersten Sensor NICHT erfogreich");
    }


    /// Verbingungstest Sensor 2 ///
    if(Sensor_2.testConnection())
    {
        Serial.println("Verbindung zum zweiten Sensor erfogreich");
    }
    else
    {
        Serial.println("Verbindung zum zweiten Sensor NICHT erfogreich");
    }


    /// Initialisieren der Sensor DMP ///
    Serial.println(F("Initalisieren der DMP..."));
    DMP_Status_Sensor_1 = Sensor_1.dmpInitialize();                                                                      // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 1 // Gyro Range wird auf +- 2000 °/sec gesetzt
    DMP_Status_Sensor_2 = Sensor_2.dmpInitialize();                                                                      // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 2 // Gyro Range wird auf +- 2000 °/sec gesetzt



    /* /// Offset setzen wir erst nach Filter Test ///
    firstMPUSensor.setXGyroOffset(51);
    firstMPUSensor.setYGyroOffset(8);
    firstMPUSensor.setZGyroOffset(21);
    firstMPUSensor.setXAccelOffset(1150);
    firstMPUSensor.setYAccelOffset(-50);
    firstMPUSensor.setZAccelOffset(1060);
    */

    /// Kalibrierung wenn nötig ///

    /*
    if (devStatus == 0 || devStatus2 == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        firstMPUSensor.CalibrateAccel(6);
        firstMPUSensor.CalibrateGyro(6);
        secondMPUSensor.CalibrateAccel(6);
        secondMPUSensor.CalibrateGyro(6);
        Serial.println();
        firstMPUSensor.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        firstMPUSensor.setDMPEnabled(true);
        secondMPUSensor.setDMPEnabled(true);

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = firstMPUSensor.dmpGetFIFOPacketSize();
        packetSize2 = secondMPUSensor.dmpGetFIFOPacketSize();
    }
    else
    {
    // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP 1 Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        Serial.print(F("DMP 2 Initialization failed (code "));
        Serial.print(devStatus2);
        Serial.println(F(")"));
    }
    */

}

//################################################################//
//############################ Loop  #############################//
//################################################################//

void loop()
{
  
    Sensor_1.dmpGetCurrentFIFOPacket(Data_Array_Sensor_1);
    Sensor_2.dmpGetCurrentFIFOPacket(Data_Array_Sensor_2);

    Sensor_1.dmpGetGyro(&Gyro_Values_Sensor_1,Data_Array_Sensor_1);




        Serial.print(Gyro_Values_Sensor_1.x);
        Serial.print(Gyro_Values_Sensor_1.y);
        Serial.print(Gyro_Values_Sensor_1.z);



    delay(1000);



}