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
#define GYRO_SENS_FACTOR_250 131
#define GYRO_SENS_FACTOR_500 65.5
#define GYRO_SENS_FACTOR_1000 32.8
#define GYRO_SENS_FACTOR_2000 16.4

////// Acceleration Sensitivities /////
#define ACC_SENS_FACTOR_2 16384
#define ACC_SENS_FACTOR_4 8192
#define ACC_SENS_FACTOR_8 4096
#define ACC_SENS_FACTOR_16 2048

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

#define QUATERNION_VALUES
#define YAW_PITCH_ROLL
#define EULER_VALUES
#define FILTERED_VALUES

///////////////////////////////////
//////// OFFSET DEFINES ///////////
///////////////////////////////////

// #define MANUAL_OFFSET
#define PID_OFFSET // loop number einstellbar in globale Variablen - aktuell 15
// #define MEANVALUE_OFFSET

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

#ifdef MEANVALUE_OFFSET
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
float yaw_sensor_1 = 0;
float pitch_sensor_1 = 0;
float roll_sensor_1 = 0;
/// Sensor 2 ///
unsigned long last_read_time_Sensor_2;
float yaw_sensor_2 = 0;
float pitch_sensor_2 = 0;
float roll_sensor_2 = 0;
#endif

//################################################################//
//######################### Functions ############################//
//################################################################//

#ifdef MEANVALUE_OFFSET

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
#endif

#ifdef COMPLEMENT_FILTER
void ComplementaryFilter_Sensor_1(int16_t accData[3], int16_t gyrData[3], float *yaw, float *pitch, float *roll)
{
    float pitchAcc, rollAcc;

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *yaw += ((float)gyrData[2] / GYRO_SENS_FACTOR_250) * (millis() - last_read_time_Sensor_1) * 1000;   // Winkel um z-Achse
    *pitch += ((float)gyrData[0] / GYRO_SENS_FACTOR_250) * (millis() - last_read_time_Sensor_1) * 1000; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYRO_SENS_FACTOR_250) * (millis() - last_read_time_Sensor_1) * 1000;  // Angle around the Y-axis

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * RADIANS_TO_DEGREES;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;

        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * RADIANS_TO_DEGREES;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
}

void ComplementaryFilter_Sensor_2(int16_t accData[3], int16_t gyrData[3], float *yaw, float *pitch, float *roll)
{
    float pitchAcc, rollAcc;

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *yaw += ((float)gyrData[2] / GYRO_SENS_FACTOR_250) * (millis() - last_read_time_Sensor_1) * 1000;   // Winkel um z-Achse
    *pitch += ((float)gyrData[0] / GYRO_SENS_FACTOR_250) * (millis() - last_read_time_Sensor_2) * 1000; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYRO_SENS_FACTOR_250) * (millis() - last_read_time_Sensor_2) * 1000;  // Angle around the Y-axis

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * RADIANS_TO_DEGREES;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;

        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * RADIANS_TO_DEGREES;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
}

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

#ifdef PID_OFFSET
    ////////////////////////////////////////////
    ///// Sensor 1 PID Regler Kalibrierung /////
    ////////////////////////////////////////////

    if (DMP_Status_Int_Sensor_1 == 0)
    {

        Sensor_1.CalibrateAccel(pidloop); // Starten des PID Reglers
        Sensor_1.CalibrateGyro(pidloop);
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
        Sensor_2.CalibrateAccel(pidloop); // Starten des PID Reglers
        Sensor_2.CalibrateGyro(pidloop);
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
#endif

#ifdef COMPLEMENT_FILTER
    ///////////////////////////////
    ///// Komplementär Filter /////
    ///////////////////////////////
    last_read_time_Sensor_1 = millis();
    last_read_time_Sensor_2 = millis();

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
    int16_t Acc_Array_Sensor_1[3] = {Acc_x_Sensor_1, Acc_y_Sensor_1, Acc_z_Sensor_1};
    int16_t Acc_Array_Sensor_2[3] = {Acc_x_Sensor_2, Acc_y_Sensor_2, Acc_z_Sensor_2};
    int16_t Gyro_Array_Sensor_1[3] = {Gyro_x_Sensor_1, Gyro_y_Sensor_1, Gyro_z_Sensor_1};
    int16_t Gyro_Array_Sensor_2[3] = {Gyro_x_Sensor_2, Gyro_y_Sensor_2, Gyro_z_Sensor_2};

    ComplementaryFilter_Sensor_1(Acc_Array_Sensor_1, Gyro_Array_Sensor_1, &yaw_sensor_1, &pitch_sensor_1, &roll_sensor_1);
    ComplementaryFilter_Sensor_2(Acc_Array_Sensor_2, Gyro_Array_Sensor_2, &yaw_sensor_2, &pitch_sensor_2, &roll_sensor_2);
    last_read_time_Sensor_1 = millis();
    last_read_time_Sensor_2 = millis();

    Serial.print(yaw_sensor_1);
    Serial.print("\t");
    Serial.print(pitch_sensor_1);
    Serial.print("\t");
    Serial.print(roll_sensor_1);
    Serial.print("\t");

    Serial.print(yaw_sensor_2);
    Serial.print("\t");
    Serial.print(pitch_sensor_2);
    Serial.print("\t");
    Serial.print(roll_sensor_2);
    Serial.print("\t");

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
