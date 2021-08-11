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
#include "GeekMum_Complement_Filter.h"
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_I2CDevice.h>
#include <TFT_ST7735.h> // Bodmer's graphics and font library for ST7735 driver chip

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
#define RADIANS_TO_DEGREES 57.2958  //180/3.14159
#define DEGREE_TO_RADIANS 0.0174533 //3.14159/180
#define M_PI 3.14159265359

///////////////////////////////////
//////// AUSGABE DEFINES //////////
///////////////////////////////////

#define QUATERNION_VALUES
// #define YAW_PITCH_ROLL
#define EULER_VALUES
#define FILTERED_VALUES
#define RAW_VALUES

///////////////////////////////////
//////// OFFSET DEFINES ///////////
///////////////////////////////////

// #define MANUAL_OFFSET
#define PID_OFFSET // loop number einstellbar in globale Variablen - aktuell 15

///////////////////////////////////
//////// FILTER DEFINES ///////////
///////////////////////////////////

// #define COMPLEMENT_FILTER
#define MADGWICK_FILTER

///////////////////////////////////
//////// SD Card Option ///////////
///////////////////////////////////

#define SD_LOGGING

///////////////////////////////////
///////// OLED Output /////////////
///////////////////////////////////

#define OLED_OUTPUT

#ifdef OLED_OUTPUT
#define REDRAW_DELAY 16 // minimum delay in milliseconds between display updates

#define HOR 205 // Horizon circle outside radius (205 is corner to corner

#define BROWN 0x5140    //0x5960
#define SKY_BLUE 0x02B5 //0x0318 //0x039B //0x34BF
#define DARK_RED 0x8000
#define DARK_GREY 0x39C7

#define XC 80 // x coord of centre of horizon
#define YC 64 // y coord of centre of horizon

#define ANGLE_INC 1 // Angle increment for arc segments, 1 will give finer resolution, 2 or more gives faster rotation

#define DEG2RAD 0.0174532925

#endif

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

float last_euler_Sensor_1[3] = {0, 0, 0}; //Speichert die vorherigen Euler Winkel z,y,x Sensor 1
float last_euler_Sensor_2[3] = {0, 0, 0}; //Speichert die vorherigen Euler Winkel z,y,x Sensor 2
float euler_result[3] = {0, 0, 0};        // Speicher die Mittelwerte

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
// unsigned long last_read_time_Sensor_1;
// float last_x_angle_Sensor_1; // These are the filtered angles
// float last_y_angle_Sensor_1;
// float last_z_angle_Sensor_1;
// float last_gyro_x_angle_Sensor_1; // Store the gyro angles to compare drift
// float last_gyro_y_angle_Sensor_1;
// float last_gyro_z_angle_Sensor_1;

/// Sensor 2 ///
// unsigned long last_read_time_Sensor_2;
// float last_x_angle_Sensor_2; // These are the filtered angles
// float last_y_angle_Sensor_2;
// float last_z_angle_Sensor_2;
// float last_gyro_x_angle_Sensor_2; // Store the gyro angles to compare drift
// float last_gyro_y_angle_Sensor_2;
// float last_gyro_z_angle_Sensor_2;

// float base_x_gyro_Sensor_1 = 0;
// float base_y_gyro_Sensor_1 = 0;
// float base_z_gyro_Sensor_1 = 0;
// float base_x_accel_Sensor_1 = 0;
// float base_y_accel_Sensor_1 = 0;
// float base_z_accel_Sensor_1 = 0;

// float base_x_gyro_Sensor_2 = 0;
// float base_y_gyro_Sensor_2 = 0;
// float base_z_gyro_Sensor_2 = 0;
// float base_x_accel_Sensor_2 = 0;
// float base_y_accel_Sensor_2 = 0;
// float base_z_accel_Sensor_2 = 0;

#endif

#ifdef MADGWICK_FILTER
float q_Sensor_1[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
float q_Sensor_2[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
float GyroMeasDrift = PI * (2.0f / 180.0f);     // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float GyroMeasError = PI * (40.0f / 180.0f);    // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;                            // integration interval for both filter schemes
uint32_t lastUpdate = 0;
uint32_t firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate
float pitch_Sensor_1;
float yaw_Sensor_1;
float roll_Sensor_1;
float pitch_Sensor_2;
float yaw_Sensor_2;
float roll_Sensor_2;
#endif

#ifdef SD_LOGGING
File myFile;
static SPISettings settings;
unsigned long Quat_counter = 0;
#endif

#ifdef OLED_OUTPUT
TFT_ST7735 tft = TFT_ST7735(); // Invoke library, pins defined in User_Setup.h
int roll_angle = 180;          // These must be initialed to 180 so updateHorizon(0); in setup() draws

int last_roll = 0; // the whole horizon graphic
int last_pitch = 0;

int roll_delta = 90; // This is used to set arc drawing direction, must be set to 90 here

// Variables for test only
int test_angle = 0;
int delta = ANGLE_INC;

unsigned long redrawTime = 0;
#endif

//################################################################//
//######################### Functions ############################//
//################################################################//

void fill_Last_euler(int runvar)
{
    switch (runvar)
    {
    case 1:
        for (int i = 0; i < 3; i++) // Die aktuellen Werte sind ok, und werden als neue "last" Values gespeichert
        {
            last_euler_Sensor_1[i] = euler_Sensor_1[i];
        }
        break;
    case 2:

        for (int i = 0; i < 3; i++) // Die aktuellen Werte sind ok, und werden als neue "last" Values gespeichert
        {
            last_euler_Sensor_2[i] = euler_Sensor_2[i];
        }
        break;
    default:
        for (int i = 0; i < 3; i++) // Die aktuellen Werte sind ok, und werden als neue "last" Values gespeichert
        {
            last_euler_Sensor_1[i] = euler_Sensor_1[i];
            last_euler_Sensor_2[i] = euler_Sensor_2[i];
        }
        break;
    }
}

bool getMeanValue_Quaternion(Quaternion &pQuat_Result) // ermittelt den Mittelwert w,x,y,z der  Quaternionen beider Sensoren
{
    pQuat_Result.w = (quaternion_Sensor_1.w + quaternion_Sensor_2.w) / 2.0;
    pQuat_Result.x = (quaternion_Sensor_1.x + quaternion_Sensor_2.x) / 2.0;
    pQuat_Result.y = (quaternion_Sensor_1.y + quaternion_Sensor_2.y) / 2.0;
    pQuat_Result.z = (quaternion_Sensor_1.z + quaternion_Sensor_2.z) / 2.0;

    return true;
}

bool getMeanValue_Eulerwinkel(float *euler_result) // ermittelt den Mittelwert der Eulerwinkel beider Sensoren    //z,y,x
{
    int runvar = 0;
    // float delta_pitch;
    // float delta_roll;

    // delta_roll = abs(euler_Sensor_1[2] * RADIANS_TO_DEGREES - euler_Sensor_2[2] * RADIANS_TO_DEGREES);
    // delta_pitch = abs(euler_Sensor_1[1] * RADIANS_TO_DEGREES - euler_Sensor_2[1] * RADIANS_TO_DEGREES);

    // if (delta_roll >= 10.0)
    // {
    //     Serial.println("Innerhalb roll pitch");
    //     float delta_last_roll_Sensor_1 = abs(euler_Sensor_1[2] * RADIANS_TO_DEGREES - last_euler_Sensor_1[2] * RADIANS_TO_DEGREES);
    //     float delta_last_roll_Sensor_2 = abs(euler_Sensor_2[2] * RADIANS_TO_DEGREES - last_euler_Sensor_2[2] * RADIANS_TO_DEGREES);

    //     if (delta_last_roll_Sensor_1 >= delta_last_roll_Sensor_2)
    //     {
    //         euler_result[0] = euler_Sensor_2[0]; // Wenn delta last pitch von Sensor 1 zu groß ist, nimm Sensor 2
    //         euler_result[1] = euler_Sensor_2[1];
    //         euler_result[2] = euler_Sensor_2[2];
    //         runvar = 2;
    //         Serial.println("Fehler Sensor 1 - Roll");
    //     }
    //     else
    //     {
    //         euler_result[0] = euler_Sensor_1[0]; // Wenn delta last pitch von Sensor 2 zu groß ist, nimm Sensor 1
    //         euler_result[1] = euler_Sensor_1[1];
    //         euler_result[2] = euler_Sensor_1[2];
    //         runvar = 1;
    //         Serial.println("Fehler Sensor 2 - Roll");
    //     }
    // }

    // if (delta_pitch >= 10.0)
    // {
    //     Serial.println("Innerhalb delta pitch");

    //     float delta_last_pitch_Sensor_1 = abs(euler_Sensor_1[1] - last_euler_Sensor_1[1]);
    //     float delta_last_pitch_Sensor_2 = abs(euler_Sensor_2[1] - last_euler_Sensor_2[1]);

    //     if (delta_last_pitch_Sensor_1 >= delta_last_pitch_Sensor_2)
    //     {
    //         euler_result[0] = euler_Sensor_2[0]; // Wenn delta last pitch von Sensor 1 zu groß ist, nimm Sensor 2
    //         euler_result[1] = euler_Sensor_2[1];
    //         euler_result[2] = euler_Sensor_2[2];
    //         runvar = 2;
    //         Serial.println("Fehler Sensor 1 - Pitch");
    //     }
    //     else
    //     {
    //         euler_result[0] = euler_Sensor_1[0]; // Wenn delta last pitch von Sensor 2 zu groß ist, nimm Sensor 1
    //         euler_result[1] = euler_Sensor_1[1];
    //         euler_result[2] = euler_Sensor_1[2];
    //         runvar = 1;
    //         Serial.println("Fehler Sensor 2 - Pitch");
    //     }
    //     runvar = 2;
    // }

    if (runvar == 0)
    {
        euler_result[0] = (euler_Sensor_1[0] + euler_Sensor_2[0]) / 2.0;
        euler_result[1] = (euler_Sensor_1[1] + euler_Sensor_2[1]) / 2.0;
        euler_result[2] = (euler_Sensor_1[2] + euler_Sensor_2[2]) / 2.0;
    }

    // fill_Last_euler(runvar);

    return true;
}

#ifdef COMPLEMENT_FILTER

// void calibrate_sensors()
// {
//     int num_readings = 10;

//     // Discard the first reading (don't know if this is needed or
//     // not, however, it won't hurt.)
//     Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);

//     // Read and average the raw values
//     for (int i = 0; i < num_readings; i++)
//     {
//         Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);
//         Gyro_x_mean_Sensor_1 += Gyro_x_Sensor_1;
//         Gyro_y_mean_Sensor_1 += Gyro_y_Sensor_1;
//         Gyro_z_mean_Sensor_1 += Gyro_z_Sensor_1;
//         Acc_x_mean_Sensor_1 += Acc_x_Sensor_1;
//         Acc_y_mean_Sensor_1 += Acc_y_Sensor_1;
//         Acc_z_mean_Sensor_1 += Acc_z_Sensor_1;
//     }

//     Gyro_x_mean_Sensor_1 /= num_readings;
//     Gyro_y_mean_Sensor_1 /= num_readings;
//     Gyro_z_mean_Sensor_1 /= num_readings;
//     Acc_x_mean_Sensor_1 /= num_readings;
//     Acc_y_mean_Sensor_1 /= num_readings;
//     Acc_z_mean_Sensor_1 /= num_readings;
// }

// void set_last_read_angle_data_Sensor_1(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
// {
//     last_read_time_Sensor_1 = time;
//     last_x_angle_Sensor_1 = x;
//     last_z_angle_Sensor_1 = y;
//     last_z_angle_Sensor_1 = z;
//     last_gyro_x_angle_Sensor_1 = x_gyro;
//     last_gyro_y_angle_Sensor_1 = y_gyro;
//     last_gyro_z_angle_Sensor_1 = z_gyro;
// }

// void set_last_read_angle_data_Sensor_2(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
// {
//     last_read_time_Sensor_2 = time;
//     last_x_angle_Sensor_2 = x;
//     last_y_angle_Sensor_2 = y;
//     last_z_angle_Sensor_2 = z;
//     last_gyro_x_angle_Sensor_2 = x_gyro;
//     last_gyro_y_angle_Sensor_2 = y_gyro;
//     last_gyro_z_angle_Sensor_2 = z_gyro;
// }

// void calibrate_sensor_1()
// {
//     int num_readings = 10;

//     // Discard the first reading (don't know if this is needed or
//     // not, however, it won't hurt.)
//     Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);
//     // Read and average the raw values
//     for (int i = 0; i < num_readings; i++)
//     {
//         Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);
//         base_x_gyro_Sensor_1 += Gyro_x_Sensor_1;
//         base_y_gyro_Sensor_1 += Gyro_y_Sensor_1;
//         base_z_gyro_Sensor_1 += Gyro_z_Sensor_1;
//         base_x_accel_Sensor_1 += Acc_x_Sensor_1;
//         base_y_accel_Sensor_1 += Acc_y_Sensor_1;
//         base_z_accel_Sensor_1 += Acc_z_Sensor_1;
//     }

//     base_x_gyro_Sensor_1 /= num_readings;
//     base_y_gyro_Sensor_1 /= num_readings;
//     base_z_gyro_Sensor_1 /= num_readings;
//     base_x_accel_Sensor_1 /= num_readings;
//     base_y_accel_Sensor_1 /= num_readings;
//     base_z_accel_Sensor_1 /= num_readings;
// }

// void calibrate_sensor_2()
// {
//     int num_readings = 10;

//     // Discard the first reading (don't know if this is needed or
//     // not, however, it won't hurt.)
//     Sensor_2.getMotion6(&Acc_x_Sensor_2, &Acc_y_Sensor_2, &Acc_z_Sensor_2, &Gyro_x_Sensor_2, &Gyro_y_Sensor_2, &Gyro_z_Sensor_2);
//     // Read and average the raw values
//     for (int i = 0; i < num_readings; i++)
//     {
//         Sensor_2.getMotion6(&Acc_x_Sensor_2, &Acc_y_Sensor_2, &Acc_z_Sensor_2, &Gyro_x_Sensor_2, &Gyro_y_Sensor_2, &Gyro_z_Sensor_2);
//         base_x_gyro_Sensor_2 += Gyro_x_Sensor_2;
//         base_y_gyro_Sensor_2 += Gyro_y_Sensor_2;
//         base_z_gyro_Sensor_2 += Gyro_z_Sensor_2;
//         base_x_accel_Sensor_2 += Acc_x_Sensor_2;
//         base_y_accel_Sensor_2 += Acc_y_Sensor_2;
//         base_z_accel_Sensor_2 += Acc_z_Sensor_2;
//     }

//     base_x_gyro_Sensor_2 /= num_readings;
//     base_y_gyro_Sensor_2 /= num_readings;
//     base_z_gyro_Sensor_2 /= num_readings;
//     base_x_accel_Sensor_2 /= num_readings;
//     base_y_accel_Sensor_2 /= num_readings;
//     base_z_accel_Sensor_2 /= num_readings;
// }

// inline unsigned long get_last_time_Sensor_1() { return last_read_time_Sensor_1; }
// inline float get_last_x_angle_Sensor_1() { return last_x_angle_Sensor_1; }
// inline float get_last_y_angle_Sensor_1() { return last_y_angle_Sensor_1; }
// inline float get_last_z_angle_Sensor_1() { return last_z_angle_Sensor_1; }
// inline float get_last_gyro_x_angle_Sensor_1() { return last_gyro_x_angle_Sensor_1; }
// inline float get_last_gyro_y_angle_Sensor_1() { return last_gyro_y_angle_Sensor_1; }
// inline float get_last_gyro_z_angle_Sensor_1() { return last_gyro_z_angle_Sensor_1; }

#endif

#ifdef MADGWICK_FILTER
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, int pSensor)
{

    float q1, q2, q3, q4; // short name local variable for readability

    if (pSensor == 1)
    {
        q1 = q_Sensor_1[0], q2 = q_Sensor_1[1], q3 = q_Sensor_1[2], q4 = q_Sensor_1[3];
    }
    else
    {
        q1 = q_Sensor_2[0], q2 = q_Sensor_2[1], q3 = q_Sensor_2[2], q4 = q_Sensor_2[3];
    }

    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz; // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 - (beta * hatDot1)) * deltat;
    q2 += (qDot2 - (beta * hatDot2)) * deltat;
    q3 += (qDot3 - (beta * hatDot3)) * deltat;
    q4 += (qDot4 - (beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
    norm = 1.0f / norm;

    if (pSensor == 1)
    {
        q_Sensor_1[0] = q1 * norm;
        q_Sensor_1[1] = q2 * norm;
        q_Sensor_1[2] = q3 * norm;
        q_Sensor_1[3] = q4 * norm;
    }
    else
    {
        q_Sensor_2[0] = q1 * norm;
        q_Sensor_2[1] = q2 * norm;
        q_Sensor_2[2] = q3 * norm;
        q_Sensor_2[3] = q4 * norm;
    }
}

#endif

#ifdef OLED_OUTPUT

// #########################################################################
// Draw a circular or elliptical arc with a defined thickness
// #########################################################################

// x,y == coords of centre of arc
// start_angle = 0 - 359
// seg_count = number of 3 degree segments to draw (120 => 360 degree arc)
// rx = x axis radius
// yx = y axis radius
// w  = width (thickness) of arc in pixels
// colour = 16 bit colour value
// Note if rx and ry are the same then an arc of a circle is drawn

int fillArc2(int x, int y, int start_angle, int seg_count, int rx, int ry, int w, unsigned int colour)
{

    byte seg = 3; // Segments are 3 degrees wide = 120 segments for 360 degrees
    byte inc = 3; // Draw segments every 3 degrees, increase to 6 for segmented ring

    // Calculate first pair of coordinates for segment start
    float sx = cos((start_angle - 90) * DEG2RAD);
    float sy = sin((start_angle - 90) * DEG2RAD);
    uint16_t x0 = sx * (rx - w) + x;
    uint16_t y0 = sy * (ry - w) + y;
    uint16_t x1 = sx * rx + x;
    uint16_t y1 = sy * ry + y;

    // Draw colour blocks every inc degrees
    for (int i = start_angle; i < start_angle + seg * seg_count; i += inc)
    {

        // Calculate pair of coordinates for segment end
        float sx2 = cos((i + seg - 90) * DEG2RAD);
        float sy2 = sin((i + seg - 90) * DEG2RAD);
        int x2 = sx2 * (rx - w) + x;
        int y2 = sy2 * (ry - w) + y;
        int x3 = sx2 * rx + x;
        int y3 = sy2 * ry + y;

        tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);

        // Copy segment end to sgement start for next segment
        x0 = x2;
        y0 = y2;
        x1 = x3;
        y1 = y3;
    }
}

void drawHorizon(int roll, int pitch)
{
    // Calculate coordinates for line start
    float sx = cos(roll * DEG2RAD);
    float sy = sin(roll * DEG2RAD);

    // Calculate coordinates for line start
    float cos_roll = cos(-roll * DEG2RAD);
    float sin_roll = sin(-roll * DEG2RAD);

    // Calculate coordinates for line start
    float cos_last_roll = cos(-last_roll * DEG2RAD);
    float sin_last_roll = sin(-last_roll * DEG2RAD);

    /// Horizontal Linie

    int16_t x0 = sx * HOR;
    int16_t y0 = sy * HOR;
    int16_t xd = 0;
    int16_t yd = 1;
    int16_t xdn = 0;
    int16_t ydn = 0;

    if (roll > 45 && roll < 135)
    {
        xd = -1;
        yd = 0;
    }
    if (roll >= 135)
    {
        xd = 0;
        yd = -1;
    }
    if (roll < -45 && roll > -135)
    {
        xd = 1;
        yd = 0;
    }
    if (roll <= -135)
    {
        xd = 0;
        yd = -1;
    }

    if ((roll != last_roll) && ((abs(roll) > 35) || (pitch != last_pitch)))
    {
        xdn = 4 * xd;
        ydn = 4 * yd;
        tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
        tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
        xdn = 3 * xd;
        ydn = 3 * yd;
        tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
        tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);

        // // Roll Pfeil Markierung
        // tft.drawCircle(80 - last_sy * 58, 64 - last_sx * 58, 2, SKY_BLUE); // alte Markierung übermalen
        // // Roll Pfeil Markierung
        // tft.drawCircle(80 - sy * 58, 64 - sx * 58, 2, TFT_RED);
    }
    xdn = 2 * xd;
    ydn = 2 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);

    tft.drawLine(XC - x0 - xd, YC - y0 - yd - pitch, XC + x0 - xd, YC + y0 - yd - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xd, YC - y0 + yd - pitch, XC + x0 + xd, YC + y0 + yd - pitch, BROWN);

    tft.drawLine(XC - x0, YC - y0 - pitch, XC + x0, YC + y0 - pitch, TFT_WHITE);

    /// Draw Chevron Line
    tft.drawLine(80 - sin_last_roll * 60, 64 - cos_last_roll * 60, (80 - sin_last_roll * 60) + sin_last_roll * 10, (64 - cos_last_roll * 60) + cos_last_roll * 10, SKY_BLUE);
    tft.drawLine(80 - sin_roll * 60, 64 - cos_roll * 60, (80 - sin_roll * 60) + sin_roll * 10, (64 - cos_roll * 60) + cos_roll * 10, TFT_RED);

    last_roll = roll;
    last_pitch = pitch;
}

// #########################################################################
// Draw the information
// #########################################################################

void drawInfo(void)
{
    // Update things near middle of screen first (most likely to get obscured)

    // Level wings graphic
    tft.fillRect(80 - 1, 64 - 1, 3, 3, TFT_RED);
    tft.drawFastHLine(80 - 30, 64, 24, TFT_RED);
    tft.drawFastHLine(80 + 30 - 24, 64, 24, TFT_RED);
    tft.drawFastVLine(80 - 30 + 24, 64, 3, TFT_RED);
    tft.drawFastVLine(80 + 30 - 24, 64, 3, TFT_RED);

    tft.drawFastHLine(80 - 12, 64 - 40, 24, TFT_WHITE);
    tft.drawFastHLine(80 - 6, 64 - 30, 12, TFT_WHITE);
    tft.drawFastHLine(80 - 12, 64 - 20, 24, TFT_WHITE);
    tft.drawFastHLine(80 - 6, 64 - 10, 12, TFT_WHITE);

    tft.drawFastHLine(80 - 6, 64 + 10, 12, TFT_WHITE);
    tft.drawFastHLine(80 - 12, 64 + 20, 24, TFT_WHITE);
    tft.drawFastHLine(80 - 6, 64 + 30, 12, TFT_WHITE);
    tft.drawFastHLine(80 - 12, 64 + 40, 24, TFT_WHITE);

    tft.setTextColor(TFT_WHITE);
    tft.setCursor(80 - 12 - 13, 64 - 20 - 3);
    tft.print("20");
    tft.setCursor(80 + 12 + 1, 64 - 20 - 3);
    tft.print("20");
    tft.setCursor(80 - 12 - 13, 64 + 20 - 3);
    tft.print("20");
    tft.setCursor(80 + 12 + 1, 64 + 20 - 3);
    tft.print("20");

    tft.setCursor(80 - 12 - 13, 64 - 40 - 3);
    tft.print("40");
    tft.setCursor(80 + 12 + 1, 64 - 40 - 3);
    tft.print("40");
    tft.setCursor(80 - 12 - 13, 64 + 40 - 3);
    tft.print("40");
    tft.setCursor(80 + 12 + 1, 64 + 40 - 3);
    tft.print("40");

    // Display justified angle value near bottom of screen
    tft.setTextColor(TFT_YELLOW, BROWN); // Text with background
    tft.setTextDatum(MC_DATUM);          // Centre middle justified
    tft.setTextPadding(24);              // Padding width to wipe previous number

    tft.setTextSize(1);
    tft.setCursor(5, 105);
    tft.print("Roll");
    tft.drawNumber(-last_roll, 15, 120, 1);

    tft.setCursor(125, 105);
    tft.print("Pitch");
    tft.drawNumber(last_pitch, 140, 120, 1);

    // 120° Kreis für Roll Winkel / Chevron Anzeige
    fillArc2(80, 64, -60, 40, 60, 60, 1, TFT_WHITE);

    /// 0° Linie
    tft.drawFastVLine(80, 0, 4, TFT_WHITE);

    /// 30° Linie
    tft.drawLine(50, 12, 46, 8, TFT_WHITE);
    tft.drawLine(28, 34, 24, 30, TFT_WHITE);
    /// 60° Linie
    tft.drawLine(110, 12, 114, 8, TFT_WHITE);
    tft.drawLine(132, 34, 136, 30, TFT_WHITE);
}

// #########################################################################
// Update the horizon with a new angle (angle in range -180 to +180)
// #########################################################################
void updateHorizon(int roll, int pitch)
{

    bool draw = 1;
    float delta_pitch = 0;
    int pitch_error = 0;
    float delta_roll = 0;
    while ((last_pitch != pitch) || (last_roll != roll))
    {
        delta_pitch = 0;
        delta_roll = 0;

        if (last_pitch < pitch)
        {
            delta_pitch = 1;
            pitch_error = pitch - last_pitch;
        }
        if (last_pitch > pitch)
        {
            delta_pitch = -1;
            pitch_error = last_pitch - pitch;
        }

        if (last_roll < roll)
            delta_roll = 1;
        if (last_roll > roll)
            delta_roll = -1;

        if (delta_roll == 0)
        {
            if (pitch_error > 1)
                delta_pitch *= 2;
        }
        drawHorizon(last_roll + delta_roll, last_pitch + delta_pitch);
        drawInfo();
    }
}

#endif

//###############################################################################################################################//
//###############################################################################################################################//
//######################################################### SETUP  ###############################################################//
//###############################################################################################################################//
//###############################################################################################################################//

void setup()
{
    // LED festlegen //

    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

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

    // Serial.println("Bitte Zeichen eingeben");
    // while (Serial.available() && Serial.read()) // empty buffer
    // {
    //     Serial.println("emtpy buffer");
    // }
    // while (!Serial.available()) // wait for data
    // {
    //     Serial.println("Bitte Zeichen eingeben");
    //     delay(500);
    // }
    // while (Serial.available() && Serial.read()) // empty buffer again
    // {
    //     Serial.println("emtpy again");
    // }

    /////////////////////////////////////////////
    /////// Initialisieren der Sensor DMP ///////
    /////////////////////////////////////////////

    Serial.println(F("Initalisieren der DMP..."));
    DMP_Status_Int_Sensor_1 = Sensor_1.dmpInitialize(); // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 1 // Gyro Range wird auf +- 250 °/sec gesetzt // FIFO Data available interrupt wird aktiviert
    DMP_Status_Int_Sensor_2 = Sensor_2.dmpInitialize(); // wenn funktioniert wird Status = 0   // beschreiben der DMP Register Sensor 2 // Gyro Range wird auf +- 250 °/sec gesetzt // FIFO Data available interrupt wird aktiviert

    /////////////////////////////////////////////
    //////////////  OLED Startup ////////////////
    /////////////////////////////////////////////

#ifdef OLED_OUTPUT

    tft.begin();
    tft.setRotation(1); // Setzen auf Querformat 160 x 128

    tft.fillScreen(0x0000);
    tft.setTextColor(0xFFFF);
    tft.setTextPadding(24); // Padding width to wipe previous number

#ifdef PID_OFFSET

    tft.setCursor(80 - 55, 64);
    tft.print("Sensor initializing"); // Informiere über Sensor PID initialising

#endif

#endif

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
        digitalWrite(4, HIGH);            // Kontroll LED anschalten
        Sensor_1.CalibrateAccel(pidloop); // Starten des PID Reglers
        Sensor_1.CalibrateGyro(pidloop);
#endif
        Serial.println();
        Sensor_1.PrintActiveOffsets(); // Zeige ermittelte Offset Werte

        /// Interrupt ///
        Sensor_1.setDMPEnabled(true); // Interrupts enablen Sensor 1

        DMP_Status_Bool_Sensor_1 = true; // alles geklappt -> setze DMP Status auf True

        Serial.println("DMP Sensor 1 Initializing Succeeded");
        digitalWrite(4, LOW); // Kontroll LED ausschalten
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
        digitalWrite(4, HIGH);            // Kontroll LED anschalten
        Sensor_2.CalibrateAccel(pidloop); // Starten des PID Reglers
        Sensor_2.CalibrateGyro(pidloop);
#endif
        Serial.println();
        Sensor_2.PrintActiveOffsets(); // Zeige ermittelte Offset Werte

        /// Interrupt ///
        Sensor_2.setDMPEnabled(true); // Interrupts enablen Sensor 1

        DMP_Status_Bool_Sensor_2 = true; // alles geklappt -> setze DMP Status auf True

        Serial.println("DMP Sensor 2 Initializing Succeeded");
        digitalWrite(4, LOW); // Kontroll LED ausschalten
    }
    else
    {
        Serial.println("Initializing DMP Sensor 2 failed -> press RESET");
    }

#ifdef COMPLEMENT_FILTER
    ///////////////////////////////
    ///// Komplementär Filter /////
    ///////////////////////////////
    calibrate_Sensor_1();
    calibrate_Sensor_2();
    set_last_read_angle_data_Sensor_1(millis(), 0, 0, 0, 0, 0, 0);
    set_last_read_angle_data_Sensor_2(millis(), 0, 0, 0, 0, 0, 0);

#endif

#ifdef SD_LOGGING

#ifdef PID_OFFSET
    tft.fillScreen(0x0000);
    tft.setCursor(80 - 55, 64);
    tft.print("SD Card initializing"); // Informiere über Sensor PID initialising
    delay(1000);
#endif

    Serial.print("Initializing SD card...");

    if (!SD.begin(29)) // Channel Select des SPI liegt auf Pi n 29 am ATMega Entwicklungsboard
    {
        Serial.println("initialization failed!");
        while (1)
            ;
    }
    Serial.println("SD Card SPI initialization done.");

#endif

#ifdef OLED_OUTPUT

#ifdef PID_OFFSET
    tft.fillScreen(0x0000);
    tft.setCursor(80 - 53, 64);
    tft.print("Initializing done!"); // Informiere über Sensor PID initialising
    delay(2000);
    tft.fillScreen(0x0000);

#endif

    tft.fillRect(0, 0, 160, 64, SKY_BLUE);
    tft.fillRect(0, 64, 160, 64, BROWN);
    delay(1000);
    drawHorizon(0, 0);

    delay(1000);
    drawInfo();
    delay(500); // Wait to permit visual check

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

//###############################################################################################################################//
//###############################################################################################################################//
//######################################################### Loop  ###############################################################//
//###############################################################################################################################//
//###############################################################################################################################//

void loop()
{
    digitalWrite(5, HIGH); // Kontroll LED anschalten

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

    Quaternion quaternion_result;
    getMeanValue_Quaternion(quaternion_result);

    // Serial.print(quaternion_Sensor_1.w, 4);
    // Serial.print("\t");
    // Serial.print(quaternion_Sensor_1.x, 4);
    // Serial.print("\t");
    // Serial.print(quaternion_Sensor_1.y, 4);
    // Serial.print("\t");
    // Serial.print(quaternion_Sensor_1.z, 4);
    // Serial.print("\t");

    // Serial.print(quaternion_Sensor_2.w, 4);
    // Serial.print("\t");
    // Serial.print(quaternion_Sensor_2.x, 4);
    // Serial.print("\t");
    // Serial.print(quaternion_Sensor_2.y, 4);
    // Serial.print("\t");
    // Serial.print(quaternion_Sensor_2.z, 4);
    // Serial.print("\t");

    // Serial.print(quaternion_result.w, 4);
    // Serial.print("\t");
    // Serial.print(quaternion_result.x, 4);
    // Serial.print("\t");
    // Serial.print(quaternion_result.y, 4);
    // Serial.print("\t");
    // Serial.print(quaternion_result.z, 4);
    // Serial.print("\t \t");

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
    Serial.println("\t");
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
    getMeanValue_Eulerwinkel(euler_result);

    // Serial.print(euler_Sensor_1[2] * RADIANS_TO_DEGREES, 4); // x
    // Serial.print("\t");
    // Serial.print(euler_Sensor_1[1] * RADIANS_TO_DEGREES, 4); // y
    // Serial.print("\t");
    // Serial.print(euler_Sensor_1[0] * RADIANS_TO_DEGREES, 4); // z
    // Serial.print("\t");

    // Serial.print(euler_Sensor_2[2] * RADIANS_TO_DEGREES, 4);
    // Serial.print("\t");
    // Serial.print(euler_Sensor_2[1] * RADIANS_TO_DEGREES, 4);
    // Serial.print("\t");
    // Serial.print(euler_Sensor_2[0] * RADIANS_TO_DEGREES, 4);
    // Serial.print("\t");

    // Serial.print(euler_result[2] * RADIANS_TO_DEGREES, 4);
    // Serial.print("\t");
    // Serial.print(euler_result[1] * RADIANS_TO_DEGREES, 4);
    // Serial.print("\t");
    // Serial.print(euler_result[0] * RADIANS_TO_DEGREES, 4);
    // Serial.print("\t \t");

#endif

#ifdef COMPLEMENT_FILTER
    /////////////////////////////////////////////
    //////////// Complement Filter //////////////
    /////////////////////////////////////////////
    accel_t_gyro_union accel_t_gyro_Sensor_1;
    accel_t_gyro_union accel_t_gyro_Sensor_2;

    // Read the raw values.
    read_gyro_accel_vals((uint8_t *)&accel_t_gyro_Sensor_1);
    read_gyro_accel_vals((uint8_t *)&accel_t_gyro_Sensor_2);

    unsigned long t_now = millis();
    float FS_SEL = 131;

    float gyro_x_Sensor_1 = (accel_t_gyro_Sensor_1.value.x_gyro - base_x_gyro_Sensor_1) / FS_SEL;
    float gyro_y_Sensor_1 = (accel_t_gyro_Sensor_1.value.y_gyro - base_y_gyro_Sensor_1) / FS_SEL;
    float gyro_z_Sensor_1 = (accel_t_gyro_Sensor_1.value.z_gyro - base_z_gyro_Sensor_1) / FS_SEL;

    float accel_x_Sensor_1 = accel_t_gyro_Sensor_1.value.x_accel;
    float accel_y_Sensor_1 = accel_t_gyro_Sensor_1.value.y_accel;
    float accel_z_Sensor_1 = accel_t_gyro_Sensor_1.value.z_accel;

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
    float alpha = 0.96;
    float angle_x_Sensor_1 = alpha * gyro_angle_x_Sensor_1 + (1.0 - alpha) * accel_angle_x_Sensor_1;
    float angle_y_Sensor_1 = alpha * gyro_angle_y_Sensor_1 + (1.0 - alpha) * accel_angle_y_Sensor_1;
    float angle_z_Sensor_1 = gyro_angle_z_Sensor_1; //Accelerometer doesn't give z-angle

    set_last_read_angle_data_Sensor_1(t_now, angle_x_Sensor_1, angle_y_Sensor_1, angle_z_Sensor_1, unfiltered_gyro_angle_x_Sensor_1, unfiltered_gyro_angle_y_Sensor_1, unfiltered_gyro_angle_z_Sensor_1);
    // set_last_read_angle_data_Sensor_2(t_now, angle_x_Sensor_2, angle_y_Sensor_2, angle_z_Sensor_2, unfiltered_gyro_angle_x_Sensor_2, unfiltered_gyro_angle_y_Sensor_2, unfiltered_gyro_angle_z_Sensor_2);

    // Serial.print(yaw_sensor_1);
    // Serial.print("\t");

    Serial.print(F("#FIL YPR:")); //Filtered angle
    Serial.print("\t");
    Serial.print(angle_z_Sensor_1, 2);
    Serial.print("\t");
    Serial.print(angle_y_Sensor_1, 2);
    Serial.print("\t");
    Serial.print(angle_x_Sensor_1, 2);
    Serial.println(F(""));

    // Serial.print(yaw_sensor_2);
    // Serial.print("\t");
    // Serial.print(pitch_sensor_2);
    // Serial.print("\t");
    // Serial.print(roll_sensor_2);
    // Serial.print("\t");
    delay(5);
#endif

#ifdef MADGWICK_FILTER
    /////////////////////////////////////////////
    ///////////// Madgwick Filter ///////////////
    /////////////////////////////////////////////

    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    Sensor_1.getMotion6(&Acc_x_Sensor_1, &Acc_y_Sensor_1, &Acc_z_Sensor_1, &Gyro_x_Sensor_1, &Gyro_y_Sensor_1, &Gyro_z_Sensor_1);
    Sensor_2.getMotion6(&Acc_x_Sensor_2, &Acc_y_Sensor_2, &Acc_z_Sensor_2, &Gyro_x_Sensor_2, &Gyro_y_Sensor_2, &Gyro_z_Sensor_2);

    float Acc_x_scaled_Sensor_1 = (float)Acc_x_Sensor_1 / ACC_SENS_FACTOR_2;
    float Acc_y_scaled_Sensor_1 = (float)Acc_y_Sensor_1 / ACC_SENS_FACTOR_2;
    float Acc_z_scaled_Sensor_1 = (float)Acc_z_Sensor_1 / ACC_SENS_FACTOR_2;

    float Acc_x_scaled_Sensor_2 = (float)Acc_x_Sensor_2 / ACC_SENS_FACTOR_2;
    float Acc_y_scaled_Sensor_2 = (float)Acc_y_Sensor_2 / ACC_SENS_FACTOR_2;
    float Acc_z_scaled_Sensor_2 = (float)Acc_z_Sensor_2 / ACC_SENS_FACTOR_2;

    float Gyro_x_scaled_Sensor_1 = (float)Gyro_x_Sensor_1 / GYRO_SENS_FACTOR_250;
    float Gyro_y_scaled_Sensor_1 = (float)Gyro_y_Sensor_1 / GYRO_SENS_FACTOR_250;
    float Gyro_z_scaled_Sensor_1 = (float)Gyro_z_Sensor_1 / GYRO_SENS_FACTOR_250;

    float Gyro_x_scaled_Sensor_2 = (float)Gyro_x_Sensor_2 / GYRO_SENS_FACTOR_250;
    float Gyro_y_scaled_Sensor_2 = (float)Gyro_y_Sensor_2 / GYRO_SENS_FACTOR_250;
    float Gyro_z_scaled_Sensor_2 = (float)Gyro_z_Sensor_2 / GYRO_SENS_FACTOR_250;

    MadgwickQuaternionUpdate(Acc_x_scaled_Sensor_1, Acc_y_scaled_Sensor_1, Acc_z_scaled_Sensor_1, Gyro_x_scaled_Sensor_1 * DEGREE_TO_RADIANS, Gyro_y_scaled_Sensor_1 * DEGREE_TO_RADIANS, Gyro_z_scaled_Sensor_1 * DEGREE_TO_RADIANS, 1); // Gyro Werte müssen als rad/sec reingegeben werden
    MadgwickQuaternionUpdate(Acc_x_scaled_Sensor_2, Acc_y_scaled_Sensor_2, Acc_z_scaled_Sensor_2, Gyro_x_scaled_Sensor_2 * DEGREE_TO_RADIANS, Gyro_y_scaled_Sensor_2 * DEGREE_TO_RADIANS, Gyro_z_scaled_Sensor_2 * DEGREE_TO_RADIANS, 2); // Gyro Werte müssen als rad/sec reingegeben werden
    delt_t = millis() - count;
    if (delt_t > 100)
    {
        yaw_Sensor_1 = atan2(2.0f * (q_Sensor_1[1] * q_Sensor_1[2] + q_Sensor_1[0] * q_Sensor_1[3]), q_Sensor_1[0] * q_Sensor_1[0] + q_Sensor_1[1] * q_Sensor_1[1] - q_Sensor_1[2] * q_Sensor_1[2] - q_Sensor_1[3] * q_Sensor_1[3]);
        pitch_Sensor_1 = -asin(2.0f * (q_Sensor_1[1] * q_Sensor_1[3] - q_Sensor_1[0] * q_Sensor_1[2]));
        roll_Sensor_1 = atan2(2.0f * (q_Sensor_1[0] * q_Sensor_1[1] + q_Sensor_1[2] * q_Sensor_1[3]), q_Sensor_1[0] * q_Sensor_1[0] - q_Sensor_1[1] * q_Sensor_1[1] - q_Sensor_1[2] * q_Sensor_1[2] + q_Sensor_1[3] * q_Sensor_1[3]);

        yaw_Sensor_2 = atan2(2.0f * (q_Sensor_2[1] * q_Sensor_2[2] + q_Sensor_2[0] * q_Sensor_2[3]), q_Sensor_2[0] * q_Sensor_2[0] + q_Sensor_2[1] * q_Sensor_2[1] - q_Sensor_2[2] * q_Sensor_2[2] - q_Sensor_2[3] * q_Sensor_2[3]);
        pitch_Sensor_2 = -asin(2.0f * (q_Sensor_2[1] * q_Sensor_2[3] - q_Sensor_2[0] * q_Sensor_2[2]));
        roll_Sensor_2 = atan2(2.0f * (q_Sensor_2[0] * q_Sensor_2[1] + q_Sensor_2[2] * q_Sensor_2[3]), q_Sensor_2[0] * q_Sensor_2[0] - q_Sensor_2[1] * q_Sensor_2[1] - q_Sensor_2[2] * q_Sensor_2[2] + q_Sensor_2[3] * q_Sensor_2[3]);

        yaw_Sensor_1 *= RADIANS_TO_DEGREES;
        pitch_Sensor_1 *= RADIANS_TO_DEGREES;
        roll_Sensor_1 *= RADIANS_TO_DEGREES;

        yaw_Sensor_2 *= RADIANS_TO_DEGREES;
        pitch_Sensor_2 *= RADIANS_TO_DEGREES;
        roll_Sensor_2 *= RADIANS_TO_DEGREES;

        // Serial.print(yaw_Sensor_1);
        // Serial.print("\t");
        // Serial.print(pitch_Sensor_1);
        // Serial.print("\t");
        // Serial.print(roll_Sensor_1);
        // Serial.print("\t");

        // Serial.print(yaw_Sensor_2);
        // Serial.print("\t");
        // Serial.print(pitch_Sensor_2);
        // Serial.print("\t");
        // Serial.print(roll_Sensor_2);
        // Serial.println("\t \t");

        count = millis();
    }

#endif

#ifdef SD_LOGGING
    SD.begin(29); // Starte erneut den SPI wenn er ihn braucht
    // pinMode(29, OUTPUT);
    // digitalWrite(29, HIGH);

    // settings = SPISettings(250000, MSBFIRST, SPI_MODE0);
    // SPI.beginTransaction(settings);
    // SPI.endTransaction();
    // digitalWrite(27, LOW);

    myFile = SD.open("test.txt", FILE_WRITE); // file öffnen und file descriptor auslesen
    if (myFile)
    {
        myFile.print(Quat_counter);
        myFile.print(",");
        myFile.print(quaternion_Sensor_1.w, 4);
        myFile.print(",");
        myFile.print(quaternion_Sensor_1.x, 4);
        myFile.print(",");
        myFile.print(quaternion_Sensor_1.y, 4);
        myFile.print(",");
        myFile.print(quaternion_Sensor_1.z, 4);
        myFile.print(",");
        myFile.print(quaternion_Sensor_2.w, 4);
        myFile.print(",");
        myFile.print(quaternion_Sensor_2.x, 4);
        myFile.print(",");
        myFile.print(quaternion_Sensor_2.y, 4);
        myFile.print(",");
        myFile.print(quaternion_Sensor_2.z, 4);
        myFile.print(",");
        myFile.print(quaternion_result.w, 4);
        myFile.print(",");
        myFile.print(quaternion_result.x, 4);
        myFile.print(",");
        myFile.print(quaternion_result.y, 4);
        myFile.print(",");
        myFile.print(quaternion_result.z, 4);
        myFile.print(",");

        myFile.print(euler_Sensor_1[2] * RADIANS_TO_DEGREES, 4); // x
        myFile.print(",");
        myFile.print(euler_Sensor_1[1] * RADIANS_TO_DEGREES, 4); // y
        myFile.print(",");
        myFile.print(euler_Sensor_1[0] * RADIANS_TO_DEGREES, 4); // z
        myFile.print(",");

        myFile.print(euler_Sensor_2[2] * RADIANS_TO_DEGREES, 4);
        myFile.print(",");
        myFile.print(euler_Sensor_2[1] * RADIANS_TO_DEGREES, 4);
        myFile.print(",");
        myFile.print(euler_Sensor_2[0] * RADIANS_TO_DEGREES, 4);
        myFile.print(",");

        myFile.print(euler_result[2] * RADIANS_TO_DEGREES, 4);
        myFile.print(",");
        myFile.print(euler_result[1] * RADIANS_TO_DEGREES, 4);
        myFile.print(",");
        myFile.print(euler_result[0] * RADIANS_TO_DEGREES, 4);
        myFile.print(",");

        myFile.print(yaw_Sensor_1);
        myFile.print(",");
        myFile.print(pitch_Sensor_1);
        myFile.print(",");
        myFile.print(roll_Sensor_1);
        myFile.print(",");

        myFile.print(yaw_Sensor_2);
        myFile.print(",");
        myFile.print(pitch_Sensor_2);
        myFile.print(",");
        myFile.print(roll_Sensor_2);
        myFile.print(",");

        myFile.print(Acc_x_Sensor_1);
        myFile.print(",");
        myFile.print(Acc_y_Sensor_1);
        myFile.print(",");
        myFile.print(Acc_z_Sensor_1);
        myFile.print(",");
        myFile.print(Gyro_x_Sensor_1);
        myFile.print(",");
        myFile.print(Gyro_y_Sensor_1);
        myFile.print(",");
        myFile.print(Gyro_z_Sensor_1);
        myFile.print(",");

        myFile.print(Acc_x_Sensor_2);
        myFile.print(",");
        myFile.print(Acc_y_Sensor_2);
        myFile.print(",");
        myFile.print(Acc_z_Sensor_2);
        myFile.print(",");
        myFile.print(Gyro_x_Sensor_2);
        myFile.print(",");
        myFile.print(Gyro_y_Sensor_2);
        myFile.print(",");
        myFile.print(Gyro_z_Sensor_2);
        myFile.println("");
        myFile.close();
        Quat_counter++;
        delay(10); // Ausgabe etwa in 0,01 sec Schritten
    }
    else
    {
        Serial.println("error opening test.txt"); //Fehlerfall Ausgabe
        digitalWrite(5, LOW);                     // Kontroll LED ausschalten -> schreiben fehlgeschlagen
    }
#endif

#ifdef OLED_OUTPUT

    updateHorizon(euler_result[2] * RADIANS_TO_DEGREES, -euler_result[1] * RADIANS_TO_DEGREES);

#endif
}
