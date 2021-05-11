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
#define SDA_Pin 20                                                                                                     // Seriell Data Pin
#define SCL_Pin 21                                                                                                     // Seriell Clock Pin (Entwicklungsboard aber anderst verdrahtet)

///// Interrupt Pin /////
#define interruptPin_Sensor_1 18                                                                                       // Interrupt Pin für ATMega 2560 für Sensor 1
#define interruptPin_Sensor_2 19                                                                                       // Interrupt Pin für ATMega 2560 für Sensor 2

///// AUSGABE DEFINES /////

//#define YAW_PITCH_ROLL
//#define EULER
#define Raw_ACCEL_GYRO


//################################################################//
//###################### globale Variablen #######################//
//################################################################//

///// DMP Status /////
uint8_t dmpStatus;                                                                                                  // Status des DMP als Wert
bool dmpReady = false;                                                                                              // Status des DMP als Boolean


///// aktuelle FIFO Groesse /////

uint16_t fifo_count_Sensor_1;                                                                                                    // Speichervariable für die aktuellen FIFO Größe für Sensor 1
uint16_t fifo_count_Sensor_2;                                                                                                    // Speichervariable für die aktuellen FIFO Größe für Sensor 2

///// Interrupt /////
bool Sensor_1_Interrupt_Bool_Status = false;                                                                                    // Interrupt Status Sensor 1                                                 
bool Sensor_2_Interrupt_Bool_Status = false;                                                                                    // Interrupt Stauts Sensor 2                                                 

///// Interrupt Status /////
uint8_t Sensor_1_Interrupt_Int_Status;                                                                                  // Statuswert des Interrupt von Sensor 1
uint8_t Sensor_2_Interrupt_Int_Status;                                                                                  // Statuswert des Interrupt von Sensor 2


///// DMP Status/////
bool DMP_Status_Bool_Sensor_1 = false;                                                                               // Status des Digital Motion Processor des Sensor 1 (zu Beginn auf false - nicht init)
bool DMP_Status_Bool_Sensor_2 = false;                                                                               // Status des Digital Motion Processor des Sensor 2    
uint8_t DMP_Status_Int_Sensor_1;                                                                                     // Status des DMP als Wert Sensor 1
uint8_t DMP_Status_Int_Sensor_2;                                                                                     // Status des DMP als Wert Sensor 2

///// Paketgröße zum FIFO lesen ////
uint16_t packetSize;                                                                                                // wird im Initialize auf 28 Byte festgelegt (16 Byte für Quaternionen, 6 Acc, 6 Gyro)

///// Data Array aus FIFO /////
uint8_t Data_Array_Sensor_1[64];                                                                                    // hier werden die Bytes aus dem FIFO von Sensor 1 eingelesen (kommen als Hex Werte an)                                                                                  
uint8_t Data_Array_Sensor_2[64];                                                                                     // hier werden die Bytes aus dem FIFO von Sensor 2 eingelesen 


///// Beschleunigungswerte /////
int16_t Acc_x_Sensor_1;                                                                                                   // X-Achsen Beschleunigungs Wert Sensor 1
int16_t Acc_y_Sensor_1;                                                                                                   // Y-Achsen Beschleunigungs Wert Sensor 1
int16_t Acc_z_Sensor_1;                                                                                                   // Z-Achsen Beschleunigungs Wert Sensor 1
int16_t Acc_x_Sensor_2;                                                                                                   // X-Achsen Beschleunigungs Wert Sensor 2
int16_t Acc_y_Sensor_2;                                                                                                   // Y-Achsen Beschleunigungs Wert Sensor 2
int16_t Acc_z_Sensor_2;                                                                                                   // Z-Achsen Beschleunigungs Wert Sensor 2

///// Drehraten - Drehgeschwindigkeit /////

int16_t Gyro_x_Sensor_1;                                                                                                   // X-Achsen Drehraten Wert Sensor 1
int16_t Gyro_y_Sensor_1;                                                                                                   // Y-Achsen Drehraten Wert Sensor 1
int16_t Gyro_z_Sensor_1;                                                                                                   // Z-Achsen Drehraten Wert Sensor 1
int16_t Gyro_x_Sensor_2;                                                                                                   // X-Achsen Drehraten Wert Sensor 2
int16_t Gyro_y_Sensor_2;                                                                                                   // Y-Achsen Drehraten Wert Sensor 2
int16_t Gyro_z_Sensor_2;                                                                                                   // Z-Achsen Drehraten Wert Sensor 2

///// Quaternionen Objekte /////
Quaternion quaternion_Sensor_1;                                                                                         // speichert Quaternionen Werte von Sensor 1 - Winkel und Drehachse in x, y, z Richtung                            
Quaternion quaternion_Sensor_2;                                                                                         // speichert Quaternionen Werte von Sensor 2 - Winkel und Drehachse in x, y, z Richtung                            

///// Gravity Vektor /////
VectorFloat gravity_Sensor_1;                                                                                           // Speichert Vektor der Richtung der Erdbeschleunigung
VectorFloat gravity_Sensor_2;                                                                                           // Speichert Vektor der Richtung der Erdbeschleunigung

///// Gravity Vektor /////
float yaw_pitch_roll_Sensor_1[3];                                                                                          // Speichert den jeweiligen Winkel zur z, y, x Achse - Sensor 1
float yaw_pitch_roll_Sensor_2[3];                                                                                          // Speichert den jeweiligen Winkel zur z, y, x Achse - Sensor 2

float euler_Sensor_1[3];                                                                                                // Speichert den jeweiligen Euler WInkel zur z,y,x Achse -Sensor 1


//################################################################//
//######################### Functions ############################//
//################################################################//

void HextoDezimal(int *Dezimal_output, const char* Hex_input) 
{

}

void Data_Available_ISR_Sensor_1()
{
    Sensor_1_Interrupt_Bool_Status = true;                                                                                              // Interrupt Flag setzten für Sensor 1
    //Serial.print("Interrupt ausgeloest :");
    //Serial.print(Sensor_1.getIntStatus());
    //Serial.println("");
}

void Data_Available_ISR_Sensor_2()
{
    Sensor_2_Interrupt_Bool_Status = true;                                                                                              // Interrupt Flag setzten für Sensor 2
}


//################################################################//
//########################### SETUP ##############################//
//################################################################//

void setup()
{
    /// I2C einrichten ///
    Wire.begin();                                                                                                       // Starte I2C
    Wire.setClock(400000);                                                                                              // Übertragungsgeschwindigkeit auf 400 kHz setzen (Fastwire)

    /// Serielle Schnittstelle ///
    Serial.begin(115200);                                                                                               // Konsolen ausgabe ermöglichen
    //while (!Serial);


    /// Sensoren aktivieren ///
    Serial.println("Initalisieren der Sensoren ueber I2C ....");
    Sensor_1.initialize();                                                                                              // legt fest: clock Quelle für die Gyro -> Winkel Berechnung  // MPU sleep deaktivieren // g Range auf +- 2g // Gyro Range auf +# 250° / sec
    //Sensor_2.initialize();


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

    /*
    /// Serial Buffer leeren ///
    Serial.println("Send any character to begin DMP programming and demo: ");
    while (Serial.available() && Serial.read()) // empty buffer
    {
        Serial.println("emtpy buffer");
    }
    while (!Serial.available())                // wait for data
    {
        Serial.println("wait for data");
    }
    while (Serial.available() && Serial.read()) // empty buffer again
    {
        Serial.println("emtpy again");
    }
*/

    /// Initialisieren der Sensor DMP ///
    Serial.println(F("Initalisieren der DMP..."));
    DMP_Status_Int_Sensor_1 = Sensor_1.dmpInitialize();                                                                      // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 1 // Gyro Range wird auf +- 2000 °/sec gesetzt // FIFO Data available interrupt wird aktiviert
    //DMP_Status_Int_Sensor_2 = Sensor_2.dmpInitialize();                                                                      // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 2 // Gyro Range wird auf +- 2000 °/sec gesetzt // FIFO Data available interrupt wird aktiviert

    if(DMP_Status_Int_Sensor_1 == 0)
    {
        /// Interrupts ///
        Sensor_1.setDMPEnabled(true);                                                                                         // Interrupts enablen Sensor 1
        //Sensor_2.setDMPEnabled(true);                                                                                         // Interrupts enablen Sensor 2

        //attachInterrupt(2, Data_Available_ISR_Sensor_1,RISING);                                                               // Interrupt Routine zuwesien Sensor 1 - Pin 18
        //attachInterrupt(4, Data_Available_ISR_Sensor_2,RISING);                                                               // Interrupt Routine zuwesien Sensor 2 - Pin 19

        dmpReady = true;

        /// Abfragen der Paketgröße //
        packetSize = Sensor_1.dmpGetFIFOPacketSize();                                                                         // PacketSize einspeichern, die wir in der Initialize festgelegt haben 
    
        Serial.println("DMP Sensor 1 Initializing Succeeded");
    }
    else
    {
        Serial.println("Initializing failed -> press RESET");

    }

    /*
    /// Offset setzen wir erst nach Filter Test ///
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

    ///// Testausgabe Überschrift /////
    //Serial.println("z-Achse, y-Achse, x-Achse");


}

//################################################################//
//############################ Loop  #############################//
//################################################################//

void loop()
{
    if(!dmpReady)                                                                                              // Initialize nicht funktioniert -> kein Start 
    {
        Serial.print("DMP Stauts failed -> return im Loop");
        return;                                 
    }
    


    /// Weitere Aufgaben auf dem Controller ///
   /*
    while(!Sensor_1_Interrupt_Bool_Status && fifo_count_Sensor_1 < packetSize)                                                          // noch kein Interrupt und der FIFO noch kleiner als ein abzuholendes Paket
    {
        Serial.println("Other Stuff");
        // Do Other stuff
    }
  */

    //Sensor_1_Interrupt_Bool_Status = false;                                                                                                 // Interrupt flag Sensor 1 driekt wieder zurücksetzten nach ISR
    //Sensor_2_Interrupt_Bool_Status = false;                                                                                                 // Interrupt flag Sensor 2 zurücksetzten

    /// Interrupt FIFO auslesen ///
    //Sensor_1_Interrupt_Int_Status = Sensor_1.getIntStatus();                                                                        // Abfragen des aktuellen Interrupt Status Sensor 1
    //Sensor_2_Interrupt_Int_Status = Sensor_2.getIntStatus();                                                                        // Abfragen des aktuellen Interrupt Status Sensor 2

    //Serial.println("Sensor_1_Interrupt_Int_Status :");
    //Serial.println(Sensor_1_Interrupt_Int_Status);

    /// FIFO Count lesen ///
    //fifo_count_Sensor_1 = Sensor_1.getFIFOCount();


    /// Overflow Interrupt prüfen ///
    /*
    if(Sensor_1_Interrupt_Int_Status & 0x10) || fifo_count_Sensor_1 == 1024)
    {
        Sensor_1.resetFIFO();
        //Serial.println("Warnung! FIFO overflow!");
    }
    */
    
    /// Data Available Interrupt prüfen ///
    //if(Sensor_1_Interrupt_Int_Status & 0x02)                                                                                        // veroderung mit 0x02 damit if Bedingung ausgeführt wird
    //{
        // Warten bis genügend Daten im FIFO vorhanden
        /*
        while(fifo_count_Sensor_1 < packetSize)
        {
            fifo_count_Sensor_1 = Sensor_1.getFIFOCount();                                                                      // aktualisieren des aktuellen FIFO Counts Sensor 1
        }
    */

                /// Abholen der Daten in Paketgröße ///
        Sensor_1.dmpGetCurrentFIFOPacket(Data_Array_Sensor_1);
        //Sensor_2.getFIFOBytes(Data_Array_Sensor_2,packetSize);

        //fifo_count_Sensor_1 -= packetSize;                                                                                      // Rücksetzten des FIFO Counts um zu lesende Paketgröße





    #ifdef YAW_PITCH_ROLL

        Sensor_1.dmpGetQuaternion(&quaternion_Sensor_1,Data_Array_Sensor_1);
        //Sensor_2.dmpGetQuaternion(&quaternion_Sensor_2,Data_Array_Sensor_2);

        Sensor_1.dmpGetGravity(&gravity_Sensor_1,&quaternion_Sensor_1);
        //Sensor_2.dmpGetGravity(&gravity_Sensor_2,&quaternion_Sensor_2);
        
        Sensor_1.dmpGetYawPitchRoll(yaw_pitch_roll_Sensor_1,&quaternion_Sensor_1,&gravity_Sensor_1);
        //Sensor_2.dmpGetYawPitchRoll(yaw_pitch_roll_Sensor_2,&quaternion_Sensor_2,&gravity_Sensor_2);

        Serial.print("yaw-pitch-roll\t");
        Serial.print(yaw_pitch_roll_Sensor_1[0] * 180 / M_PI);
        Serial.print(", ");
        Serial.print(yaw_pitch_roll_Sensor_1[1] * 180 / M_PI);
        Serial.print(", ");
        Serial.print(yaw_pitch_roll_Sensor_1[2] * 180 / M_PI);
        Serial.println("");
    #endif

    /*
    #ifdef EULER

        Sensor_1.dmpGetQuaternion(&quaternion_Sensor_1,Data_Array_Sensor_1);
        //Sensor_2.dmpGetQuaternion(&quaternion_Sensor_2,Data_Array_Sensor_2);

        Sensor_1.dmpGetEuler(euler_Sensor_1,&quaternion_Sensor_1);
    
        /// Serielle Ausgabe Euler ///
        Serial.print("euler\t");
        Serial.print(euler_Sensor_1[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(euler_Sensor_1[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(euler_Sensor_1[2] * 180 / M_PI);

    #endif
*/
    #ifdef Raw_ACCEL_GYRO

    Sensor_1.getMotion6(&Acc_x_Sensor_1,&Acc_y_Sensor_1,&Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);

        //Serial.print("Acceleration:\t");
        Serial.print(Acc_x_Sensor_1); Serial.print("\t");
        Serial.print(Acc_y_Sensor_1); Serial.print("\t");
        Serial.print(Acc_z_Sensor_1); Serial.print("\t");
        Serial.println("");

        //Serial.print("Gyro:\t");
        //Serial.print(Gyro_x_Sensor_1); Serial.print("\t");
        //Serial.print(Gyro_y_Sensor_1); Serial.print("\t");
        //Serial.println(Gyro_z_Sensor_1);

    #endif



    //}

        delay(1000);


}