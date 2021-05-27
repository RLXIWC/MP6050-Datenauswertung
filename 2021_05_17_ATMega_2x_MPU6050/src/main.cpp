/* Titel: ATMega 2560 mit 2 MPU6050 
*
* Version: v1.0
* Autor: Jeremy und Heiko
* Datum: 17.05.2021
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
#include "I2Cdev.h"                         // Library für I2C Sensor MPU6050 und anderen Sensoren
#include "MPU6050_6Axis_MotionApps_V6_12.h" // MPU6050 Library von I2CDev
#include "Wire.h"                           // Nötig für I2C Bus

//// Sensorobjekte ////

MPU6050 Sensor_1(MPU6050_ADDRESS_AD0_LOW);  // Standard Adresse 0x68  -  ADO0 Low ist ein define in der MPU6050.h
MPU6050 Sensor_2(MPU6050_ADDRESS_AD0_HIGH); // Parameter ist neue Adresse 0x69   -   ebenso ein define

//################################################################//
//########################## Defines #############################//
//################################################################//

///// I2C Pins zuweisen /////
#define SDA_Pin 20 // Seriell Data Pin
#define SCL_Pin 21 // Seriell Clock Pin (Entwicklungsboard aber anderst verdrahtet)

///// Interrupt Pin /////
#define interruptPin_Sensor_1 18 // Interrupt Pin für ATMega 2560 für Sensor 1
#define interruptPin_Sensor_2 19 // Interrupt Pin für ATMega 2560 für Sensor 2

///// Mathematische AUsdrücke /////
const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159

///// AUSGABE DEFINES /////

// #define QUATERNION_VALUES
#define YAW_PITCH_ROLL
//#define EULER_VALUES
//#define ACCEL_GYRO
//#define RAW_VALUES_COMPLEMENTARY

//################################################################//
//###################### globale Variablen #######################//
//################################################################//

///// aktuelle FIFO Groesse /////

uint16_t fifo_count_Sensor_1; // Speichervariable für die aktuellen FIFO Größe für Sensor 1
uint16_t fifo_count_Sensor_2; // Speichervariable für die aktuellen FIFO Größe für Sensor 2

///// Interrupt /////
bool Sensor_1_Interrupt_Bool_Status = false; // Interrupt Status Sensor 1
bool Sensor_2_Interrupt_Bool_Status = false; // Interrupt Stauts Sensor 2
uint8_t Sensor_1_Interrupt_Int_Status; // Statuswert des Interrupt von Sensor 1
uint8_t Sensor_2_Interrupt_Int_Status; // Statuswert des Interrupt von Sensor 2

///// DMP Status/////
bool DMP_Status_Bool_Sensor_1 = false; // Status des Digital Motion Processor des Sensor 1 (zu Beginn auf false - nicht init)
bool DMP_Status_Bool_Sensor_2 = false; // Status des Digital Motion Processor des Sensor 2
uint8_t DMP_Status_Int_Sensor_1;       // Status des DMP als Wert Sensor 1
uint8_t DMP_Status_Int_Sensor_2;       // Status des DMP als Wert Sensor 2

///// Paketgröße zum FIFO lesen ////
uint16_t packetSize_Sensor_1; // wird im Initialize auf 28 Byte festgelegt (16 Byte für Quaternionen, 6 Acc, 6 Gyro)
uint16_t packetSize_Sensor_2; // wird im Initialize auf 28 Byte festgelegt (16 Byte für Quaternionen, 6 Acc, 6 Gyro)

///// Data Array aus FIFO /////
uint8_t Data_Array_Sensor_1[64]; // hier werden die Bytes aus dem FIFO von Sensor 1 eingelesen (kommen als Hex Werte an)
uint8_t Data_Array_Sensor_2[64]; // hier werden die Bytes aus dem FIFO von Sensor 2 eingelesen

///// Beschleunigungswerte /////
int16_t Acc_x_Sensor_1; // X-Achsen Beschleunigungs Wert Sensor 1
int16_t Acc_y_Sensor_1; // Y-Achsen Beschleunigungs Wert Sensor 1
int16_t Acc_z_Sensor_1; // Z-Achsen Beschleunigungs Wert Sensor 1
int16_t Acc_x_Sensor_2; // X-Achsen Beschleunigungs Wert Sensor 2
int16_t Acc_y_Sensor_2; // Y-Achsen Beschleunigungs Wert Sensor 2
int16_t Acc_z_Sensor_2; // Z-Achsen Beschleunigungs Wert Sensor 2
VectorInt16 Acc_Sensor_1;
VectorInt16 Acc_Sensor_2;

///// Drehraten - Drehgeschwindigkeit /////

int16_t Gyro_x_Sensor_1; // X-Achsen Drehraten Wert Sensor 1
int16_t Gyro_y_Sensor_1; // Y-Achsen Drehraten Wert Sensor 1
int16_t Gyro_z_Sensor_1; // Z-Achsen Drehraten Wert Sensor 1
int16_t Gyro_x_Sensor_2; // X-Achsen Drehraten Wert Sensor 2
int16_t Gyro_y_Sensor_2; // Y-Achsen Drehraten Wert Sensor 2
int16_t Gyro_z_Sensor_2; // Z-Achsen Drehraten Wert Sensor 2
VectorInt16 Gyro_Sensor_1;
VectorInt16 Gyro_Sensor_2;

///// Variablen für Mittelwert /////
float Acc_x_mean_Sensor_1;  // X-Achsen Beschleunigungs Mittelwert
float Acc_y_mean_Sensor_1;  // Y-Achsen Beschleunigungs Mittelwert
float Acc_z_mean_Sensor_1;  // Z-Achsen Beschleunigungs Mittelwert
float Gyro_x_mean_Sensor_1; // X-Achsen Drehraten Mittelwert
float Gyro_y_mean_Sensor_1; // Y-Achsen Drehraten Mittelwert
float Gyro_z_mean_Sensor_1; // Z-Achsen Drehraten Mittelwert

int16_t Acc_x_mean_Sensor_2;  // X-Achsen Beschleunigungs Mittelwert
int16_t Acc_y_mean_Sensor_2;  // Y-Achsen Beschleunigungs Mittelwert
int16_t Acc_z_mean_Sensor_2;  // Z-Achsen Beschleunigungs Mittelwert
int16_t Gyro_x_mean_Sensor_2; // X-Achsen Drehraten Mittelwert
int16_t Gyro_y_mean_Sensor_2; // Y-Achsen Drehraten Mittelwert
int16_t Gyro_z_mean_Sensor_2; // Z-Achsen Drehraten Mittelwert

///// Quaternionen Objekte /////
Quaternion quaternion_Sensor_1; // speichert Quaternionen Werte von Sensor 1 - Winkel und Drehachse in x, y, z Richtung
Quaternion quaternion_Sensor_2; // speichert Quaternionen Werte von Sensor 2 - Winkel und Drehachse in x, y, z Richtung

///// Gravity Vektor /////
VectorFloat gravity_Sensor_1; // Speichert Vektor der Richtung der Erdbeschleunigung
VectorFloat gravity_Sensor_2; // Speichert Vektor der Richtung der Erdbeschleunigung

///// Gravity Vektor /////
float yaw_pitch_roll_Sensor_1[3]; // Speichert den jeweiligen Winkel zur z, y, x Achse - Sensor 1
float yaw_pitch_roll_Sensor_2[3]; // Speichert den jeweiligen Winkel zur z, y, x Achse - Sensor 2

float euler_Sensor_1[3]; // Speichert den jeweiligen Euler WInkel zur z,y,x Achse -Sensor 1
float euler_Sensor_2[3]; // Speichert den jeweiligen Euler WInkel zur z,y,x Achse -Sensor 2


/////////////////////////////////
////// Komplementär Filter //////
/////////////////////////////////

///// Last Values for Angle Integration /////

/// Sensor 1 ///
unsigned long last_read_time_Sensor_1;
float last_x_angle_Sensor_1; // These are the filtered angles
float last_y_angle_Sensor_1;
float last_z_angle_Sensor_1;
float last_gyro_x_angle_Sensor_1; // Store the gyro angles to compare drift
float last_gyro_y_angle_Sensor_1;
float last_gyro_z_angle_Sensor_1;

/// Sensor 2 ///
unsigned long last_read_time_Sensor_2;
float last_x_angle_Sensor_2; // These are the filtered angles
float last_y_angle_Sensor_2;
float last_z_angle_Sensor_2;
float last_gyro_x_angle_Sensor_2; // Store the gyro angles to compare drift
float last_gyro_y_angle_Sensor_2;
float last_gyro_z_angle_Sensor_2;

//################################################################//
//######################### Functions ############################//
//################################################################//

void Data_Available_ISR_Sensor_1()
{
    Sensor_1_Interrupt_Bool_Status = true; // Interrupt Flag setzten für Sensor 1
    //Serial.print("Interrupt ausgeloest :");
    //Serial.print(Sensor_1.getIntStatus());
    //Serial.println("");
}

void Data_Available_ISR_Sensor_2()
{
    Sensor_2_Interrupt_Bool_Status = true; // Interrupt Flag setzten für Sensor 2
}

////////////////////////
////// Mittelwert //////
////////////////////////
void calculate_Mean_Value()
{
    Acc_x_mean_Sensor_1 = (Acc_x_Sensor_1 + Acc_x_Sensor_2) / 2;
    Acc_y_mean_Sensor_1 = (Acc_y_Sensor_1 + Acc_y_Sensor_2) / 2;
    Acc_z_mean_Sensor_1 = (Acc_z_Sensor_1 + Acc_z_Sensor_2) / 2;

    Gyro_x_mean_Sensor_1 = (Gyro_x_Sensor_1 + Gyro_x_Sensor_2) / 2;
    Gyro_y_mean_Sensor_1 = (Gyro_y_Sensor_1 + Gyro_y_Sensor_2) / 2;
    Gyro_z_mean_Sensor_1 = (Gyro_z_Sensor_1 + Gyro_z_Sensor_2) / 2;
}

/////////////////////////////////////
////// Mittelwert Euler Winkel //////
////////////////////////////////////
void calculate_Mean_Value_Euler(float *meanEuler, float *euler1, float *euler2)
{
    meanEuler[0] = (euler1[0] + euler2[0]) / 2;
    meanEuler[1] = (euler1[1] + euler2[1]) / 2;
    meanEuler[2] = (euler1[2] + euler2[2]) / 2;
}



void calibrate_sensors()
{
    int num_readings = 10;

    // Discard the first reading (don't know if this is needed or
    // not, however, it won't hurt.)
    Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);

    // Read and average the raw values
    for (int i = 0; i < num_readings; i++)
    {
    Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);
    Gyro_x_mean_Sensor_1 += Gyro_x_Sensor_1;
    Gyro_y_mean_Sensor_1 += Gyro_y_Sensor_1;
    Gyro_z_mean_Sensor_1 += Gyro_z_Sensor_1;
    Acc_x_mean_Sensor_1 += Acc_x_Sensor_1;
    Acc_y_mean_Sensor_1 += Acc_y_Sensor_1;
    Acc_z_mean_Sensor_1 += Acc_z_Sensor_1;
    }

    Gyro_x_mean_Sensor_1 /= num_readings;
    Gyro_y_mean_Sensor_1 /= num_readings;
    Gyro_z_mean_Sensor_1 /= num_readings;
    Acc_x_mean_Sensor_1 /= num_readings;
    Acc_y_mean_Sensor_1 /= num_readings;
    Acc_z_mean_Sensor_1 /= num_readings;
}


inline unsigned long get_last_time(char pSensor)
 { 
     if(pSensor == 1)
     {
        return last_read_time_Sensor_1; 
     }
     else if (pSensor == 2)
     {
         return last_read_time_Sensor_2; 
     }
 }

inline float get_last_x_angle() { return last_x_angle_Sensor_1; }
inline float get_last_y_angle() { return last_y_angle_Sensor_1; }
inline float get_last_z_angle() { return last_z_angle_Sensor_1; }
inline float get_last_gyro_x_angle() { return last_gyro_x_angle_Sensor_1; }
inline float get_last_gyro_y_angle() { return last_gyro_y_angle_Sensor_1; }
inline float get_last_gyro_z_angle() { return last_gyro_z_angle_Sensor_1; }

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
{
    last_read_time_Sensor_1 = time;
    last_x_angle_Sensor_1 = x;
    last_y_angle_Sensor_1 = y;
    last_z_angle_Sensor_1 = z;
    last_gyro_x_angle_Sensor_1 = x_gyro;
    last_gyro_y_angle_Sensor_1 = y_gyro;
    last_gyro_z_angle_Sensor_1 = z_gyro;
}



//################################################################//
//########################### SETUP ##############################//
//################################################################//

void setup()
{
    /// I2C einrichten ///
    Wire.begin();          // Starte I2C
    Wire.setClock(400000); // Übertragungsgeschwindigkeit auf 400 kHz setzen (Fastwire)

    /// Serielle Schnittstelle ///
    Serial.begin(115200); // Konsolen ausgabe ermöglichen
    //while (!Serial);

    /// Sensoren aktivieren ///
    Serial.println("Initalisieren der Sensoren ueber I2C ....");
    Sensor_1.initialize(); // legt fest: clock Quelle für die Gyro -> Winkel Berechnung  // MPU sleep deaktivieren // g Range auf +- 2g // Gyro Range auf +# 250° / sec
    // Sensor_2.initialize();

    /// Verbingungstest Sensor 1 ///
    if (Sensor_1.testConnection())
    {
        Serial.println("Verbindung zum ersten Sensor erfogreich");
    }
    else
    {
        Serial.println("Verbindung zum ersten Sensor NICHT erfogreich");
    }

    /// Verbingungstest Sensor 2 ///
    // if (Sensor_2.testConnection())
    // {
    //     Serial.println("Verbindung zum zweiten Sensor erfogreich");
    // }
    // else
    // {
    //     Serial.println("Verbindung zum zweiten Sensor NICHT erfogreich");
    // }

    /// Serial Buffer leeren ///
    Serial.println("Send any character to begin DMP programming and demo: ");
    while (Serial.available() && Serial.read()) // empty buffer
    {
        Serial.println("emtpy buffer");
    }
    while (!Serial.available()) // wait for data
    {
        Serial.println("Bitte Zeichen eingeben");
        delay(500);
    }
    while (Serial.available() && Serial.read()) // empty buffer again
    {
        Serial.println("emtpy again");
    }


    /////////////////////////////////////////////
    /////// Initialisieren der Sensor DMP ///////
    /////////////////////////////////////////////

    Serial.println(F("Initalisieren der DMP..."));
    DMP_Status_Int_Sensor_1 = Sensor_1.dmpInitialize(); // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 1 // Gyro Range wird auf +- 2000 °/sec gesetzt // FIFO Data available interrupt wird aktiviert
    // DMP_Status_Int_Sensor_2 = Sensor_2.dmpInitialize(); // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 2 // Gyro Range wird auf +- 2000 °/sec gesetzt // FIFO Data available interrupt wird aktiviert

    /////////////////////////////////////////////
    ////////////// Set Offsets //////////////////
    /////////////////////////////////////////////

    /////////////////////
    /////  Sensor 1 /////
    /////////////////////
    // Sensor_1.setXGyroOffset(107);
    // Sensor_1.setYGyroOffset(-1);
    // Sensor_1.setZGyroOffset(30);
    // Sensor_1.setXAccelOffset(-1700);
    // Sensor_1.setYAccelOffset(-3673);
    // Sensor_1.setZAccelOffset(5078);

    /////////////////////
    /////  Sensor 2 /////
    /////////////////////
    // Sensor_2.setXGyroOffset(301);
    // Sensor_2.setYGyroOffset(-57);
    // Sensor_2.setZGyroOffset(60);
    // Sensor_2.setXAccelOffset(-4868);
    // Sensor_2.setYAccelOffset(-1227);
    // Sensor_2.setZAccelOffset(1144);

    ////////////////////////////////////////////
    ///// Sensor 1 PID Regler Kalibrierung /////
    ////////////////////////////////////////////

    if (DMP_Status_Int_Sensor_1 == 0)
    {

        Sensor_1.CalibrateAccel(15); // Starten des PID Reglers
        Sensor_1.CalibrateGyro(15);
        Serial.println();
        Sensor_1.PrintActiveOffsets(); // Zeige ermittelte Offset Werte

        /// Interrupt ///
        Sensor_1.setDMPEnabled(true); // Interrupts enablen Sensor 1
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //attachInterrupt(2, Data_Available_ISR_Sensor_1,RISING);                                                               // Interrupt Routine zuwesien Sensor 1 - Pin 18

        DMP_Status_Bool_Sensor_1 = true; // alles geklappt -> setze DMP Status auf True

        /// Abfragen der Paketgröße //
        packetSize_Sensor_1 = Sensor_1.dmpGetFIFOPacketSize(); // PacketSize einspeichern, die wir in der Initialize festgelegt haben

        Serial.println("DMP Sensor 1 Initializing Succeeded");
    }
    else
    {
        Serial.println("Initializing DMP Sensor 1 failed -> press RESET");
    }

    ////////////////////////////////////////////
    ///// Sensor 2 PID Regler Kalibrierung /////
    ////////////////////////////////////////////

    // if (DMP_Status_Int_Sensor_2 == 0)
    // {
    //     Sensor_2.CalibrateAccel(15); // Starten des PID Reglers
    //     Sensor_2.CalibrateGyro(15);
    //     Serial.println();
    //     Sensor_2.PrintActiveOffsets(); // Zeige ermittelte Offset Werte

    //     /// Interrupt ///
    //     Sensor_2.setDMPEnabled(true); // Interrupts enablen Sensor 1
    //     //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //     //attachInterrupt(2, Data_Available_ISR_Sensor_1,RISING);                                                               // Interrupt Routine zuwesien Sensor 1 - Pin 18

    //     DMP_Status_Bool_Sensor_2 = true; // alles geklappt -> setze DMP Status auf True

    //     /// Abfragen der Paketgröße //
    //     packetSize_Sensor_2 = Sensor_2.dmpGetFIFOPacketSize(); // PacketSize einspeichern, die wir in der Initialize festgelegt haben

    //     Serial.println("DMP Sensor 2 Initializing Succeeded");
    // }
    // else
    // {
    //     Serial.println("Initializing DMP Sensor 2 failed -> press RESET");
    // }



///////////////////////////////////////
/////////// Startausgaben /////////////
///////////////////////////////////////
#ifdef Raw_ACCEL_GYRO
    ///// Testausgabe Überschrift /////
    Serial.print("Acceleration Values: x-Achse, y-Achse, z-Achse\t");
    Serial.println("Gyro Values: x-Achse, y-Achse, z-Achse");
#endif

#ifdef EULER_VALUES
    ///// Testausgabe Überschrift /////
    Serial.println("Winkel in: z-Achse, y-Achse, x-Achse");
#endif

    //////////////////////////////////
    
    Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);                                        // 0x68 Sensor 1
    Wire.write(0x1B);                                                                       // Gyro Config Register
    Wire.endTransmission(false);                                                            // transmission not finished
    char temp = Wire.requestFrom(MPU6050_ADDRESS_AD0_LOW,1,true);                           // Pick the 14 registers
    Wire.endTransmission(true);                                                            // transmission finished

    Serial.print("Gyro Sensitivity set before loop: \t");
    Serial.println((int)temp);

    Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);                                        // 0x68 Sensor 1
    Wire.write(0x1C);                                                                       // Gyro Config Register
    Wire.endTransmission(false);                                                            // transmission not finished
    temp = Wire.requestFrom(MPU6050_ADDRESS_AD0_LOW,1,true);                           // Pick the 14 registers
    Wire.endTransmission(true);                                                            // transmission finished

    Serial.print("Acceleration Sensitivity set before loop: \t");
    Serial.println((int)temp);

    //////////////////////////////////



    //calibrate_sensors();
    delay(5000); // Start abwarten bis eingependelt nach Kalibrierung
}

//################################################################//
//############################ Loop  #############################//
//################################################################//

void loop()
{
    if (!DMP_Status_Bool_Sensor_1 /*|| !DMP_Status_Bool_Sensor_2*/) // Initialize nicht funktioniert -> kein Start
    {
        Serial.print("DMP Status failed -> return im Loop");
        return;
    }

    /////////////////////////////////////////////
    ///////////////// Get Data //////////////////
    /////////////////////////////////////////////

    Sensor_1.dmpGetCurrentFIFOPacket(Data_Array_Sensor_1); // get Current FIFO Packet holt autpmatisch die richtige packetsize
    // Sensor_2.dmpGetCurrentFIFOPacket(Data_Array_Sensor_2);


/////////////////////////////////////////////
//////////////// Quaternionen ///////////////
/////////////////////////////////////////////
#ifdef QUATERNION_VALUES
    Sensor_1.dmpGetQuaternion(&quaternion_Sensor_1, Data_Array_Sensor_1);
    Sensor_2.dmpGetQuaternion(&quaternion_Sensor_2, Data_Array_Sensor_2);

    Serial.print(quaternion_Sensor_1.w);
    Serial.print("\t");
    Serial.print(quaternion_Sensor_1.x);
    Serial.print("\t");
    Serial.print(quaternion_Sensor_1.y);
    Serial.print("\t");
    Serial.print(quaternion_Sensor_1.z);
    Serial.print("\t");

    Serial.print(quaternion_Sensor_2.w);
    Serial.print("\t");
    Serial.print(quaternion_Sensor_2.x);
    Serial.print("\t");
    Serial.print(quaternion_Sensor_2.y);
    Serial.print("\t");
    Serial.print(quaternion_Sensor_2.z);
    Serial.println("");

#endif

/////////////////////////////////////////////
////////////// Yaw Pitch Roll ///////////////
/////////////////////////////////////////////
#ifdef YAW_PITCH_ROLL

    Sensor_1.dmpGetQuaternion(&quaternion_Sensor_1, Data_Array_Sensor_1);
    //Sensor_2.dmpGetQuaternion(&quaternion_Sensor_2,Data_Array_Sensor_2);

    Sensor_1.dmpGetGravity(&gravity_Sensor_1, &quaternion_Sensor_1);
    //Sensor_2.dmpGetGravity(&gravity_Sensor_2,&quaternion_Sensor_2);

    Sensor_1.dmpGetYawPitchRoll(yaw_pitch_roll_Sensor_1, &quaternion_Sensor_1, &gravity_Sensor_1);
    //Sensor_2.dmpGetYawPitchRoll(yaw_pitch_roll_Sensor_2,&quaternion_Sensor_2,&gravity_Sensor_2);

    // Serial.print("yaw-pitch-roll\t");
    Serial.print(yaw_pitch_roll_Sensor_1[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(yaw_pitch_roll_Sensor_1[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(yaw_pitch_roll_Sensor_1[2] * 180 / M_PI);
    Serial.println("\t");
#endif

/////////////////////////////////////////////
/////////////// Euler Winkel ////////////////
/////////////////////////////////////////////
#ifdef EULER_VALUES

    Sensor_1.dmpGetQuaternion(&quaternion_Sensor_1, Data_Array_Sensor_1);
    Sensor_2.dmpGetQuaternion(&quaternion_Sensor_2, Data_Array_Sensor_2);

    // Quaternion product_quaternion;                                                                                                          // lokales Quaternionen zum Speichern des Produkt Ergebnisses
    //product_quaternion = quaternion_Sensor_1.getProduct(quaternion_Sensor_2);                                                               // Multiplikation von Quaternion aus Sensor 1 und Sensor 2 (Methode wirs aus der Quaternion 1 Klasse aufgerufen)

    // float euler_ergebnis[3];
    // Sensor_1.dmpGetEuler(euler_ergebnis,&product_quaternion);                                                                               // Speichern der Eulerwinkel in eine lokale Variable (ob Sensor_1 oder Sensor_2 genutzt wird spielt keine Rolle)
    Sensor_1.dmpGetEuler(euler_Sensor_1, &quaternion_Sensor_1);
    Sensor_2.dmpGetEuler(euler_Sensor_2, &quaternion_Sensor_2);

    //float euler_ergebnis[3];

    //calculate_Mean_Value_Euler(euler_ergebnis, euler_Sensor_1, euler_Sensor_2);

    Serial.print(euler_Sensor_1[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler_Sensor_1[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler_Sensor_1[2] * 180 / M_PI);
    Serial.print("\t");

    Serial.print(euler_Sensor_2[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler_Sensor_2[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler_Sensor_2[2] * 180 / M_PI);
    Serial.print("\t");

    /// Serielle Ausgabe Euler ///
    // Serial.print(euler_ergebnis[0] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.print(euler_ergebnis[1] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.println(euler_ergebnis[2] * 180 / M_PI);

#endif

/////////////////////////////////////////////
/////////////////Raw Values /////////////////
/////////////////////////////////////////////
#ifdef ACCEL_GYRO

    Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);
    Sensor_2.getMotion6(&Acc_x_Sensor_2, &Acc_y_Sensor_2, &Acc_z_Sensor_2, &Gyro_x_Sensor_2, &Gyro_y_Sensor_2, &Gyro_z_Sensor_2);

    // calculate_Mean_Value();

    Serial.print((float)Acc_x_Sensor_1/16384.0);
    Serial.print("\t");
    Serial.print((float)Acc_y_Sensor_1 / 16384.0);
    Serial.print("\t");
    Serial.print((float)Acc_z_Sensor_1 / 16384.0);
    Serial.print("\t");

    Serial.print((float)Gyro_x_Sensor_1 / 131);
    Serial.print("\t");
    Serial.print((float)Gyro_y_Sensor_1 / 131);
    Serial.print("\t");
    Serial.print((float)Gyro_z_Sensor_1 / 131);
    Serial.print("\t");

    Serial.print((float)Acc_x_Sensor_2 / 2048.0);
    Serial.print("\t");
    Serial.print((float)Acc_y_Sensor_2 / 2048.0);
    Serial.print("\t");
    Serial.print((float)Acc_z_Sensor_2 / 2048.0);
    Serial.print("\t");

    Serial.print((float)Gyro_x_Sensor_2 / 131);
    Serial.print("\t");
    Serial.print((float)Gyro_y_Sensor_2 / 131);
    Serial.print("\t");
    Serial.println((float)Gyro_z_Sensor_2 / 131);
    
#endif

#ifdef RAW_VALUES_COMPLEMENTARY

    Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);                  // Liest die entsprechenden Register aus per I2C read()
    // Sensor_2.getMotion6(&Acc_x_Sensor_2, &Acc_y_Sensor_2, &Acc_z_Sensor_2, &Gyro_x_Sensor_2, &Gyro_y_Sensor_2, &Gyro_z_Sensor_2);

    unsigned long currentTime = millis();
    
    float scaled_gyro_x_sensor_1 = (Gyro_x_Sensor_1 - Gyro_x_mean_Sensor_1) / 131;
    float scaled_gyro_y_sensor_1 = (Gyro_y_Sensor_1 - Gyro_y_mean_Sensor_1) / 131;
    float scaled_gyro_z_sensor_1 = (Gyro_z_Sensor_1 - Gyro_z_mean_Sensor_1) / 131;
    float scaled_acc_x_sensor_1 = Acc_x_Sensor_1;
    float scaled_acc_y_sensor_1 = Acc_y_Sensor_1;
    float scaled_acc_z_sensor_1 = Acc_z_Sensor_1;

    float accel_angle_y = atan(-1 * scaled_acc_x_sensor_1 / sqrt(pow(scaled_acc_y_sensor_1, 2) + pow(scaled_acc_z_sensor_1, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_x = atan(scaled_acc_y_sensor_1 / sqrt(pow(scaled_acc_x_sensor_1, 2) + pow(scaled_acc_z_sensor_1, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_z = 0;

    // Compute the (filtered) gyro angles
    float dt = (currentTime - get_last_time(1)) / 1000.0;
    float gyro_angle_x = scaled_gyro_x_sensor_1 * dt + get_last_x_angle();
    float gyro_angle_y = scaled_gyro_y_sensor_1 * dt + get_last_y_angle();
    float gyro_angle_z = scaled_gyro_z_sensor_1 * dt + get_last_z_angle();

    // Compute the drifting gyro angles

    float unfiltered_gyro_angle_x = scaled_gyro_x_sensor_1 * dt + get_last_gyro_x_angle();
    float unfiltered_gyro_angle_y = scaled_gyro_y_sensor_1 * dt + get_last_gyro_y_angle();
    float unfiltered_gyro_angle_z = scaled_gyro_z_sensor_1 * dt + get_last_gyro_z_angle();


    ///// Complementär Filter /////
    const float alpha = 0.96;
    float angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
    float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
    float angle_z = gyro_angle_z; //Accelerometer doesn't give z-angle

    set_last_read_angle_data(currentTime, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

    Serial.print("CMP:");
    Serial.print(get_last_x_angle(), 2);
    Serial.print(":");
    Serial.print(get_last_y_angle(), 2);
    Serial.print(":");
    Serial.println(-get_last_z_angle(), 2);


#endif


    //delay(1000);                                                                                                                                // Sekündliche Prints
}

