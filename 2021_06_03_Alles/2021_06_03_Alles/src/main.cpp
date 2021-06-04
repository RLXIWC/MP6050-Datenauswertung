/* Titel: ATMega 2560 mit 2 MPU6050 - Test mit allen Möglichkeiten
*
* Version: v1.0
* Autor: Jeremy und Heiko
* Datum: 03.06.2021
*
* Kurze Beschreibung:
* Test aller möglichen Ausgaben und Offset Einstellungen
* Fusion aus 2 Sensoren
*
* Ausgabe:
* Quaternion, Eulerwinkel, YPR, Raw Values Gyro und Acc
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

//################################################################//
//########################## Defines #############################//
//################################################################//

////// Gyro Sensitivities /////
#define GYRO_SENS_FACTOR_250 131.0
#define GYRO_SENS_FACTOR_500 65.5
#define GYRO_SENS_FACTOR_1000 32.8
#define GYRO_SENS_FACTOR_2000 16.4

////// Acceleration Sensitivities /////
#define ACC_SENS_FACTOR_2 16384.0
#define ACC_SENS_FACTOR_4 8192.0
#define ACC_SENS_FACTOR_8 4096.0
#define ACC_SENS_FACTOR_16 2048.0

///// I2C Pins zuweisen /////
#define SDA_Pin 20 // Seriell Data Pin
#define SCL_Pin 21 // Seriell Clock Pin (Entwicklungsboard aber anderst verdrahtet)

/////////////////////////////////////////
//////// Mathematische DEFINES //////////
/////////////////////////////////////////
#define RADIANS_TO_DEGREES 57.2958 //180/3.14159
#define M_PI 3.14159265359

///////////////////////////////////
//////// AUSGABE DEFINES //////////
///////////////////////////////////

// #define QUATERNION_VALUES
// #define YAW_PITCH_ROLL
// #define EULER_VALUES
#define FILTERED_VALUES

///////////////////////////////////
//////// OFFSET DEFINES ///////////
///////////////////////////////////

// #define MANUAL_OFFSET
// #define PID_OFFSET // loop number einstellbar in globale Variablen - aktuell 15

///////////////////////////////////
//////// FILTER DEFINES ///////////
///////////////////////////////////

#define COMPLEMENT_FILTER
//#define MADGWICK_FILTER

//################################################################//
//###################### globale Variablen #######################//
//################################################################//

//// Sensorobjekte ////

MPU6050 Sensor_1(MPU6050_ADDRESS_AD0_LOW);  // Standard Adresse 0x68  -  ADO0 Low ist ein define in der MPU6050.h
MPU6050 Sensor_2(MPU6050_ADDRESS_AD0_HIGH); // Parameter ist neue Adresse 0x69   -   ebenso ein define

///// DMP Status/////
bool DMP_Status_Bool_Sensor_1 = false; // Status des Digital Motion Processor des Sensor 1 (zu Beginn auf false - nicht init)
bool DMP_Status_Bool_Sensor_2 = false; // Status des Digital Motion Processor des Sensor 2
uint8_t DMP_Status_Int_Sensor_1;       // Status des DMP als Wert Sensor 1
uint8_t DMP_Status_Int_Sensor_2;       // Status des DMP als Wert Sensor 2

///// Data Array aus FIFO /////
uint8_t Data_Array_Sensor_1[64]; // hier werden die Bytes aus dem FIFO von Sensor 1 eingelesen (kommen als Hex Werte an)
uint8_t Data_Array_Sensor_2[64]; // hier werden die Bytes aus dem FIFO von Sensor 2 eingelesen                                                            // evtl reichen 28 Byte

///// Beschleunigungswerte /////
int16_t Acc_x_Sensor_1; // X-Achsen Beschleunigungs Wert Sensor 1
int16_t Acc_y_Sensor_1; // Y-Achsen Beschleunigungs Wert Sensor 1
int16_t Acc_z_Sensor_1; // Z-Achsen Beschleunigungs Wert Sensor 1
int16_t Acc_x_Sensor_2; // X-Achsen Beschleunigungs Wert Sensor 2
int16_t Acc_y_Sensor_2; // Y-Achsen Beschleunigungs Wert Sensor 2
int16_t Acc_z_Sensor_2; // Z-Achsen Beschleunigungs Wert Sensor 2

///// Drehraten - Drehgeschwindigkeit /////

int16_t Gyro_x_Sensor_1; // X-Achsen Drehraten Wert Sensor 1
int16_t Gyro_y_Sensor_1; // Y-Achsen Drehraten Wert Sensor 1
int16_t Gyro_z_Sensor_1; // Z-Achsen Drehraten Wert Sensor 1
int16_t Gyro_x_Sensor_2; // X-Achsen Drehraten Wert Sensor 2
int16_t Gyro_y_Sensor_2; // Y-Achsen Drehraten Wert Sensor 2
int16_t Gyro_z_Sensor_2; // Z-Achsen Drehraten Wert Sensor 2

#ifdef COMPLEMENT_FILTER
///// Variablen für Mittelwert /////

int16_t Acc_x_mean_Sensor_1;  // X-Achsen Beschleunigungs Mittelwert
int16_t Acc_y_mean_Sensor_1;  // Y-Achsen Beschleunigungs Mittelwert
int16_t Acc_z_mean_Sensor_1;  // Z-Achsen Beschleunigungs Mittelwert
int16_t Gyro_x_mean_Sensor_1; // X-Achsen Drehraten Mittelwert
int16_t Gyro_y_mean_Sensor_1; // Y-Achsen Drehraten Mittelwert
int16_t Gyro_z_mean_Sensor_1; // Z-Achsen Drehraten Mittelwert

int16_t Acc_x_mean_Sensor_2;  // X-Achsen Beschleunigungs Mittelwert
int16_t Acc_y_mean_Sensor_2;  // Y-Achsen Beschleunigungs Mittelwert
int16_t Acc_z_mean_Sensor_2;  // Z-Achsen Beschleunigungs Mittelwert
int16_t Gyro_x_mean_Sensor_2; // X-Achsen Drehraten Mittelwert
int16_t Gyro_y_mean_Sensor_2; // Y-Achsen Drehraten Mittelwert
int16_t Gyro_z_mean_Sensor_2; // Z-Achsen Drehraten Mittelwert
#endif

///// Quaternionen Objekte /////
Quaternion quaternion_Sensor_1; // speichert Quaternionen Werte von Sensor 1 - Winkel und Drehachse in x, y, z Richtung
Quaternion quaternion_Sensor_2; // speichert Quaternionen Werte von Sensor 2 - Winkel und Drehachse in x, y, z Richtung

///// Gravity Vektor /////
VectorFloat gravity_Sensor_1; // Speichert Vektor der Richtung der Erdbeschleunigung
VectorFloat gravity_Sensor_2; // Speichert Vektor der Richtung der Erdbeschleunigung

///// YPR Vektor /////
float yaw_pitch_roll_Sensor_1[3]; // Speichert den jeweiligen Winkel zur z, y, x Achse - Sensor 1
float yaw_pitch_roll_Sensor_2[3]; // Speichert den jeweiligen Winkel zur z, y, x Achse - Sensor 2

///// EULER Vektor /////
float euler_Sensor_1[3]; // Speichert den jeweiligen Euler WInkel zur z,y,x Achse -Sensor 1
float euler_Sensor_2[3]; // Speichert den jeweiligen Euler WInkel zur z,y,x Achse -Sensor 2

#ifdef PID_OFFSET
///// PID OFFSET Variables /////
uint8_t pidloop = 15;
#endif

#ifdef COMPLEMENT_FILTER
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

float base_x_gyro_Sensor_1 = 0;
float base_y_gyro_Sensor_1 = 0;
float base_z_gyro_Sensor_1 = 0;
float base_x_accel_Sensor_1 = 0;
float base_y_accel_Sensor_1 = 0;
float base_z_accel_Sensor_1 = 0;

float base_x_gyro_Sensor_2 = 0;
float base_y_gyro_Sensor_2 = 0;
float base_z_gyro_Sensor_2 = 0;
float base_x_accel_Sensor_2 = 0;
float base_y_accel_Sensor_2 = 0;
float base_z_accel_Sensor_2 = 0;

#endif

//################################################################//
//######################### Functions ############################//
//################################################################//

#ifdef COMPLEMENT_FILTER

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

void set_last_read_angle_data_Sensor_1(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
{
    last_read_time_Sensor_1 = time;
    last_x_angle_Sensor_1 = x;
    last_z_angle_Sensor_1 = y;
    last_z_angle_Sensor_1 = z;
    last_gyro_x_angle_Sensor_1 = x_gyro;
    last_gyro_y_angle_Sensor_1 = y_gyro;
    last_gyro_z_angle_Sensor_1 = z_gyro;
}

void set_last_read_angle_data_Sensor_2(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
{
    last_read_time_Sensor_2 = time;
    last_x_angle_Sensor_2 = x;
    last_y_angle_Sensor_2 = y;
    last_z_angle_Sensor_2 = z;
    last_gyro_x_angle_Sensor_2 = x_gyro;
    last_gyro_y_angle_Sensor_2 = y_gyro;
    last_gyro_z_angle_Sensor_2 = z_gyro;
}

void calibrate_sensor_1()
{
    int num_readings = 10;

    // Discard the first reading (don't know if this is needed or
    // not, however, it won't hurt.)
    Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);
    // Read and average the raw values
    for (int i = 0; i < num_readings; i++)
    {
        Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);
        base_x_gyro_Sensor_1 += Gyro_x_Sensor_1;
        base_y_gyro_Sensor_1 += Gyro_y_Sensor_1;
        base_z_gyro_Sensor_1 += Gyro_z_Sensor_1;
        base_x_accel_Sensor_1 += Acc_x_Sensor_1;
        base_y_accel_Sensor_1 += Acc_y_Sensor_1;
        base_z_accel_Sensor_1 += Acc_z_Sensor_1;
    }

    base_x_gyro_Sensor_1 /= num_readings;
    base_y_gyro_Sensor_1 /= num_readings;
    base_z_gyro_Sensor_1 /= num_readings;
    base_x_accel_Sensor_1 /= num_readings;
    base_y_accel_Sensor_1 /= num_readings;
    base_z_accel_Sensor_1 /= num_readings;
}

void calibrate_sensor_2()
{
    int num_readings = 10;

    // Discard the first reading (don't know if this is needed or
    // not, however, it won't hurt.)
    Sensor_2.getMotion6(&Acc_x_Sensor_2, &Acc_y_Sensor_2, &Acc_z_Sensor_2, &Gyro_x_Sensor_2, &Gyro_y_Sensor_2, &Gyro_z_Sensor_2);
    // Read and average the raw values
    for (int i = 0; i < num_readings; i++)
    {
        Sensor_2.getMotion6(&Acc_x_Sensor_2, &Acc_y_Sensor_2, &Acc_z_Sensor_2, &Gyro_x_Sensor_2, &Gyro_y_Sensor_2, &Gyro_z_Sensor_2);
        base_x_gyro_Sensor_2 += Gyro_x_Sensor_2;
        base_y_gyro_Sensor_2 += Gyro_y_Sensor_2;
        base_z_gyro_Sensor_2 += Gyro_z_Sensor_2;
        base_x_accel_Sensor_2 += Acc_x_Sensor_2;
        base_y_accel_Sensor_2 += Acc_y_Sensor_2;
        base_z_accel_Sensor_2 += Acc_z_Sensor_2;
    }

    base_x_gyro_Sensor_2 /= num_readings;
    base_y_gyro_Sensor_2 /= num_readings;
    base_z_gyro_Sensor_2 /= num_readings;
    base_x_accel_Sensor_2 /= num_readings;
    base_y_accel_Sensor_2 /= num_readings;
    base_z_accel_Sensor_2 /= num_readings;
}

inline unsigned long get_last_time_Sensor_1() { return last_read_time_Sensor_1; }
inline float get_last_x_angle_Sensor_1() { return last_x_angle_Sensor_1; }
inline float get_last_y_angle_Sensor_1() { return last_y_angle_Sensor_1; }
inline float get_last_z_angle_Sensor_1() { return last_z_angle_Sensor_1; }
inline float get_last_gyro_x_angle_Sensor_1() { return last_gyro_x_angle_Sensor_1; }
inline float get_last_gyro_y_angle_Sensor_1() { return last_gyro_y_angle_Sensor_1; }
inline float get_last_gyro_z_angle_Sensor_1() { return last_gyro_z_angle_Sensor_1; }

#endif

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

    /// Sensoren aktivieren ///
    Serial.println("Initalisieren der Sensoren ueber I2C ....");
    Sensor_1.initialize(); // legt fest: clock Quelle für die Gyro -> Winkel Berechnung  // MPU sleep deaktivieren // g Range auf +- 2g // Gyro Range auf +# 250° / sec
    Sensor_2.initialize();

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
    if (Sensor_2.testConnection())
    {
        Serial.println("Verbindung zum zweiten Sensor erfogreich");
    }
    else
    {
        Serial.println("Verbindung zum zweiten Sensor NICHT erfogreich");
    }

    ////////////////////////////////////
    /////// Serial Buffer leeren ///////
    ////////////////////////////////////

    Serial.println("Bitte Zeichen eingeben");
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
    DMP_Status_Int_Sensor_1 = Sensor_1.dmpInitialize(); // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 1 // Gyro Range wird auf +- 250 °/sec gesetzt // FIFO Data available interrupt wird aktiviert
    DMP_Status_Int_Sensor_2 = Sensor_2.dmpInitialize(); // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 2 // Gyro Range wird auf +- 250 °/sec gesetzt // FIFO Data available interrupt wird aktiviert

    /////////////////////////////////////////////
    ////////////// Set Offsets //////////////////
    /////////////////////////////////////////////

#ifdef MANUAL_OFFSET
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
#endif

    ////////////////////////////////////////////
    ///// Sensor 1 PID Regler Kalibrierung /////
    ////////////////////////////////////////////

    if (DMP_Status_Int_Sensor_1 == 0)
    {
#ifdef PID_OFFSET
        Sensor_1.CalibrateAccel(pidloop); // Starten des PID Reglers
        Sensor_1.CalibrateGyro(pidloop);
#endif
        Serial.println();
        Sensor_1.PrintActiveOffsets(); // Zeige ermittelte Offset Werte

        /// Interrupt ///
        Sensor_1.setDMPEnabled(true); // Interrupts enablen Sensor 1

        DMP_Status_Bool_Sensor_1 = true; // alles geklappt -> setze DMP Status auf True

        Serial.println("DMP Sensor 1 Initializing Succeeded");
    }
    else
    {
        Serial.println("Initializing DMP Sensor 1 failed -> press RESET");
    }

    ////////////////////////////////////////////
    ///// Sensor 2 PID Regler Kalibrierung /////
    ////////////////////////////////////////////

    if (DMP_Status_Int_Sensor_2 == 0)
    {
#ifdef PID_OFFSET
        Sensor_2.CalibrateAccel(pidloop); // Starten des PID Reglers
        Sensor_2.CalibrateGyro(pidloop);
#endif
        Serial.println();
        Sensor_2.PrintActiveOffsets(); // Zeige ermittelte Offset Werte

        /// Interrupt ///
        Sensor_2.setDMPEnabled(true); // Interrupts enablen Sensor 1

        DMP_Status_Bool_Sensor_2 = true; // alles geklappt -> setze DMP Status auf True

        Serial.println("DMP Sensor 2 Initializing Succeeded");
    }
    else
    {
        Serial.println("Initializing DMP Sensor 2 failed -> press RESET");
    }

#ifdef COMPLEMENT_FILTER
    ///////////////////////////////
    ///// Komplementär Filter /////
    ///////////////////////////////
    calibrate_sensor_1();
    calibrate_sensor_2();
    set_last_read_angle_data_Sensor_1(millis(), 0, 0, 0, 0, 0, 0);
    set_last_read_angle_data_Sensor_2(millis(), 0, 0, 0, 0, 0, 0);

#endif

    ///////////////////////////////////////
    /////////// Startausgaben /////////////
    ///////////////////////////////////////

#ifdef QUATERNION_VALUES
    Serial.print("Quaternionen: Winkel, i, j, k \t");
#endif

#ifdef EULER_VALUES
    Serial.print("Winkel in: z-Achse, y-Achse, x-Achse \t");
#endif

#ifdef YAW_PITCH_ROLL
    Serial.print("Winkel Yaw , Pitch , Roll \t");
#endif

#ifdef FILTERED_VALUES
    Serial.print("Filtered Values: ");
#ifdef COMPLEMENT_FILTER
    Serial.print("Complement Filter: Yaw, Pitch, Roll ");
#endif
#ifdef MADGWICK_FILTER
    Serial.print("Madgwick Filter: Yaw, Pitch, Roll ");
#endif
#endif

    delay(3000); // Start abwarten bis eingependelt nach Kalibrierung
}

//################################################################//
//############################ Loop  #############################//
//################################################################//

void loop()
{

    if (!DMP_Status_Bool_Sensor_1 || !DMP_Status_Bool_Sensor_2) // Initialize nicht funktioniert -> kein Start
    {
        Serial.print("At least One DMP Status failed -> return im Loop");
        return;
    }

#ifdef QUATERNION_VALUES
    /////////////////////////////////////////////
    //////////////// Quaternionen ///////////////
    /////////////////////////////////////////////

    ///////////////////////
    ////// Get Data ///////
    ///////////////////////
    Sensor_1.dmpGetCurrentFIFOPacket(Data_Array_Sensor_1); // get Current FIFO Packet holt autpmatisch die richtige packetsize
    Sensor_2.dmpGetCurrentFIFOPacket(Data_Array_Sensor_2);

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
    Serial.print("\t");
#endif

#ifdef YAW_PITCH_ROLL
    /////////////////////////////////////////////
    ////////////// Yaw Pitch Roll ///////////////
    /////////////////////////////////////////////

    ///////////////////////
    ////// Get Data ///////
    ///////////////////////
    Sensor_1.dmpGetCurrentFIFOPacket(Data_Array_Sensor_1); // get Current FIFO Packet holt autpmatisch die richtige packetsize
    Sensor_2.dmpGetCurrentFIFOPacket(Data_Array_Sensor_2);

    Sensor_1.dmpGetQuaternion(&quaternion_Sensor_1, Data_Array_Sensor_1);
    Sensor_2.dmpGetQuaternion(&quaternion_Sensor_2, Data_Array_Sensor_2);

    Sensor_1.dmpGetGravity(&gravity_Sensor_1, &quaternion_Sensor_1);
    Sensor_2.dmpGetGravity(&gravity_Sensor_2, &quaternion_Sensor_2);

    Sensor_1.dmpGetYawPitchRoll(yaw_pitch_roll_Sensor_1, &quaternion_Sensor_1, &gravity_Sensor_1);
    Sensor_2.dmpGetYawPitchRoll(yaw_pitch_roll_Sensor_2, &quaternion_Sensor_2, &gravity_Sensor_2);

    Serial.print(yaw_pitch_roll_Sensor_1[0] * RADIANS_TO_DEGREES);
    Serial.print("\t");
    Serial.print(yaw_pitch_roll_Sensor_1[1] * RADIANS_TO_DEGREES);
    Serial.print("\t");
    Serial.print(yaw_pitch_roll_Sensor_1[2] * RADIANS_TO_DEGREES);
    Serial.print("\t");

    Serial.print(yaw_pitch_roll_Sensor_2[0] * RADIANS_TO_DEGREES);
    Serial.print("\t");
    Serial.print(yaw_pitch_roll_Sensor_2[1] * RADIANS_TO_DEGREES);
    Serial.print("\t");
    Serial.print(yaw_pitch_roll_Sensor_2[2] * RADIANS_TO_DEGREES);
    Serial.print("\t");
#endif

#ifdef EULER_VALUES
    /////////////////////////////////////////////
    /////////////// Euler Winkel ////////////////
    /////////////////////////////////////////////

    ///////////////////////
    ////// Get Data ///////
    ///////////////////////
    Sensor_1.dmpGetCurrentFIFOPacket(Data_Array_Sensor_1); // get Current FIFO Packet holt autpmatisch die richtige packetsize
    Sensor_2.dmpGetCurrentFIFOPacket(Data_Array_Sensor_2);

    Sensor_1.dmpGetQuaternion(&quaternion_Sensor_1, Data_Array_Sensor_1);
    Sensor_2.dmpGetQuaternion(&quaternion_Sensor_2, Data_Array_Sensor_2);

    Sensor_1.dmpGetEuler(euler_Sensor_1, &quaternion_Sensor_1);
    Sensor_2.dmpGetEuler(euler_Sensor_2, &quaternion_Sensor_2);

    Serial.print(euler_Sensor_1[2] * RADIANS_TO_DEGREES);
    Serial.print("\t");
    Serial.print(euler_Sensor_1[1] * RADIANS_TO_DEGREES);
    Serial.print("\t");
    Serial.print(euler_Sensor_1[0] * RADIANS_TO_DEGREES);
    Serial.print("\t");

    Serial.print(euler_Sensor_2[2] * RADIANS_TO_DEGREES);
    Serial.print("\t");
    Serial.print(euler_Sensor_2[1] * RADIANS_TO_DEGREES);
    Serial.print("\t");
    Serial.print(euler_Sensor_2[0] * RADIANS_TO_DEGREES);
    Serial.print("\t");
#endif

#ifdef COMPLEMENT_FILTER
    /////////////////////////////////////////////
    //////////// Complement Filter //////////////
    /////////////////////////////////////////////

    Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1); // Liest die entsprechenden Register aus per I2C read()
    Sensor_2.getMotion6(&Acc_x_Sensor_2, &Acc_y_Sensor_2, &Acc_z_Sensor_2, &Gyro_x_Sensor_2, &Gyro_y_Sensor_2, &Gyro_z_Sensor_2);
    unsigned long t_now = millis();

    // Remove offsets and scale gyro data
    float gyro_x_Sensor_1 = (Gyro_x_Sensor_1 - base_x_gyro_Sensor_1) / GYRO_SENS_FACTOR_250;
    float gyro_y_Sensor_1 = (Gyro_y_Sensor_1 - base_y_gyro_Sensor_1) / GYRO_SENS_FACTOR_250;
    float gyro_z_Sensor_1 = (Gyro_z_Sensor_1 - base_z_gyro_Sensor_1) / GYRO_SENS_FACTOR_250;
    float accel_x_Sensor_1 = Acc_x_Sensor_1; // - base_x_accel;
    float accel_y_Sensor_1 = Acc_y_Sensor_1; // - base_y_accel;
    float accel_z_Sensor_1 = Acc_z_Sensor_1; // - base_z_accel;

    float accel_angle_y_Sensor_1 = atan(-1 * accel_x_Sensor_1 / sqrt(pow(accel_y_Sensor_1, 2) + pow(accel_z_Sensor_1, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_x_Sensor_1 = atan(accel_y_Sensor_1 / sqrt(pow(accel_x_Sensor_1, 2) + pow(accel_z_Sensor_1, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_z_Sensor_1 = 0;

    // Compute the (filtered) gyro angles
    float dt = (t_now - get_last_time_Sensor_1()) / 1000.0;
    float gyro_angle_x_Sensor_1 = gyro_x_Sensor_1 * dt + get_last_x_angle_Sensor_1();
    float gyro_angle_y_Sensor_1 = gyro_y_Sensor_1 * dt + get_last_y_angle_Sensor_1();
    float gyro_angle_z_Sensor_1 = gyro_z_Sensor_1 * dt + get_last_z_angle_Sensor_1();

    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x_Sensor_1 = gyro_x_Sensor_1 * dt + get_last_gyro_x_angle_Sensor_1();
    float unfiltered_gyro_angle_y_Sensor_1 = gyro_y_Sensor_1 * dt + get_last_gyro_y_angle_Sensor_1();
    float unfiltered_gyro_angle_z_Sensor_1 = gyro_z_Sensor_1 * dt + get_last_gyro_z_angle_Sensor_1();

    // Apply the complementary filter to figure out the change in angle - choice of alpha is
    // estimated now.  Alpha depends on the sampling rate...
    const float alpha = 0.96;
    float angle_x_Sensor_1 = alpha * gyro_angle_x_Sensor_1 + (1.0 - alpha) * accel_angle_x_Sensor_1;
    float angle_y_Sensor_1 = alpha * gyro_angle_y_Sensor_1 + (1.0 - alpha) * accel_angle_y_Sensor_1;
    float angle_z_Sensor_1 = gyro_angle_z_Sensor_1; //Accelerometer doesn't give z-angle

    // Serial.print(yaw_sensor_1);
    // Serial.print("\t");
    Serial.print(angle_x_Sensor_1, 4);
    Serial.print("\t");
    Serial.print(angle_y_Sensor_1, 4);
    Serial.print("\t");
    set_last_read_angle_data_Sensor_1(t_now, angle_x_Sensor_1, angle_y_Sensor_1, angle_z_Sensor_1, unfiltered_gyro_angle_x_Sensor_1, unfiltered_gyro_angle_y_Sensor_1, unfiltered_gyro_angle_z_Sensor_1);

    // Serial.print(yaw_sensor_2);
    // Serial.print("\t");
    // Serial.print(pitch_sensor_2);
    // Serial.print("\t");
    // Serial.print(roll_sensor_2);
    // Serial.print("\t");

#endif

#ifdef MADGWICK_FILTER
    /////////////////////////////////////////////
    ///////////// Madgwick Filter ///////////////
    /////////////////////////////////////////////

    Serial.print(xx);
    Serial.print("\t");
    Serial.print(xx);
    Serial.print("\t");
    Serial.print(xx);
    Serial.print("\t");

    Serial.print(xx);
    Serial.print("\t");
    Serial.print(xx);
    Serial.print("\t");
    Serial.print(xx);
    Serial.print("\t");

#endif

    Serial.println("");
}
