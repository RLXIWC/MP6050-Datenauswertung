/* Titel: Quaternionen mit 2 Sensoren am ESP32
*
* Version: v1.0
* Autor: Jeremy Kunz
* Datum: 23.04.2021
*
* Kurze Beschreibung:
* Auslesen von 2 MPU6050 Sensordaten, Speichern in Quaternionen, Multiplikation und Ausgabe
*
* Ausgabe:
* Quaternionen und Produkt
*
* Anschluss weiterer Elektronik:
* 2x MPU6050 per I2C anschließen, VCC, GND, SCL an PIN 23, SDA an PIN 21 (ESP32)
* zweiter MPU erhält Spannungsversorgung über AD0 statt über VCC (andere Adresse im I2C Bus)
*
* Changelog:
* 24.04.2021 Kommentare (Heiko)
*  
*/


//################################################################//
//########################## Includes ############################//
//################################################################//

#include "I2Cdev.h"                                                                                                     // Library für I2C Sensor MPU6050 und anderen Sensoren
#include "MPU6050_6Axis_MotionApps_V6_12.h"                                                                             // MU6050 Library von I2CDev

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"                                                                                                       // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
#endif                                                                                                                  // is used in I2Cdev.h


//// Sensorobjekte ////
MPU6050 firstMPUSensor;                                                                                                 // default Adresse is 0x68
MPU6050 secondMPUSensor(0x69);                                                                                          // zweite Adresse auf 0x69 setzen


//################################################################//
//########################## Defines #############################//
//################################################################//


//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

//// Definieren der Pins fuer den I2C ////
#define SDA 21
#define SCL 23


//################################################################//
//###################### globale Variablen #######################//
//################################################################//

//// MPU Status und Kontrollevariablen /////
bool dmpReady = false;                                                                                                    // DPM Status, wenn Init erfolgreich -> auf true setzen
uint8_t mpuIntStatus;                                                                                                     // Interrupt Status Byte vom MPU
uint8_t mpuIntStatus2;                                                                                                    // weiteres Interrupt Status Byte vom MPU
uint8_t devStatus;                                                                                                        // Status des Device nach Ausführung einer Operation (0 = success, !0 = error)
uint8_t devStatus2;                                                                                                       // Status des Device nach Ausführung einer Operation (0 = success, !0 = error)
uint16_t packetSize, packetSize2;                                                                                         // DMP packet size (default sind 42 bytes)
uint16_t fifoCount;                                                                                                       // Aktuelle Byte Anzahl in der FIFO Queue
uint8_t fifoBuffer[64];                                                                                                   // FIFO Storage Buffer 1
uint8_t fifoBuffer2[64];                                                                                                  // FIFO Storage Buffer2

//// Quaternionen Objekte ////
Quaternion firstQuaternion, secondQuaternion, productResult;                                                              // [w, x, y, z]         Quatenrionen Objekt
VectorInt16 aa, aa2;                                                                                                      // [x, y, z]            Acellerometer Sensor Werte
VectorInt16 gy, gy2;                                                                                                      // [x, y, z]            Gyrometer Sensor Werte
VectorInt16 aaReal, aaReal2;                                                                                              // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld, aaWorld2;                                                                                            // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity, gravity2;                                                                                            // [x, y, z]            gravity vector
float euler[3];                                                                                                           // [psi, theta, phi]    Euler angle container
float ypr[3];                                                                                                             // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//################################################################//
//########################### SETUP ##############################//
//################################################################//

void setup()
{
  //  I2C hinzuguegen sowie einstellen der Adresse sowie Taktfrequenz
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(SDA, SCL, 400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  // initialize device
  Serial.println(F("Initalisieren der Sensoren ueber I2C ...."));
  firstMPUSensor.initialize();
  secondMPUSensor.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(firstMPUSensor.testConnection() ? F("Verbindung zum ersten Sensor erfogreich") : F("Verbindung zum ersten Sensor nicht erfolgreich"));
  Serial.println(secondMPUSensor.testConnection() ? F("Verbindung zum zweiten Sensor erfogreich") : F("Verbindung zum zweiten Sensor nicht erfolgreich"));

  // load and configure the DMP
  Serial.println(F("Initalisieren der DMP..."));
  devStatus = firstMPUSensor.dmpInitialize();
  devStatus = secondMPUSensor.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  firstMPUSensor.setXGyroOffset(51);
  firstMPUSensor.setYGyroOffset(8);
  firstMPUSensor.setZGyroOffset(21);
  firstMPUSensor.setXAccelOffset(1150);
  firstMPUSensor.setYAccelOffset(-50);
  firstMPUSensor.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
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
}

//################################################################//
//############################ Loop  #############################//
//################################################################//

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (firstMPUSensor.dmpGetCurrentFIFOPacket(fifoBuffer) && secondMPUSensor.dmpGetCurrentFIFOPacket(fifoBuffer2))
  { // Get the Latest packet

#ifdef OUTPUT_READABLE_QUATERNION

    // display quaternion values in easy matrix form: w x y z
    firstMPUSensor.dmpGetQuaternion(&firstQuaternion, fifoBuffer);
    secondMPUSensor.dmpGetQuaternion(&secondQuaternion, fifoBuffer2);

    productResult = firstQuaternion.getProduct(secondQuaternion);

    Serial.print("Quaternion 1\t");
    Serial.print(firstQuaternion.w);
    Serial.print("\t");
    Serial.print(firstQuaternion.x);
    Serial.print("\t");
    Serial.print(firstQuaternion.y);
    Serial.print("\t");
    Serial.println(firstQuaternion.z);

    Serial.print("Quaternion 2\t");
    Serial.print(secondQuaternion.w);
    Serial.print("\t");
    Serial.print(secondQuaternion.x);
    Serial.print("\t");
    Serial.print(secondQuaternion.y);
    Serial.print("\t");
    Serial.println(secondQuaternion.z);

    Serial.print("Produkt der Quaternion\t");
    Serial.print(productResult.w);
    Serial.print("\t");
    Serial.print(productResult.x);
    Serial.print("\t");
    Serial.print(productResult.y);
    Serial.print("\t");
    Serial.println(productResult.z);

    delay(500);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    firstMPUSensor.dmpGetQuaternion(&firstQuaternion, fifoBuffer);
    firstMPUSensor.dmpGetEuler(euler, &firstQuaternion); 
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    firstMPUSensor.dmpGetQuaternion(&firstQuaternion, fifoBuffer);
    firstMPUSensor.dmpGetGravity(&gravity, &firstQuaternion);
    firstMPUSensor.dmpGetYawPitchRoll(ypr, &firstQuaternion, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    /*
      mpu.dmpGetAccel(&aa, fifoBuffer);
      Serial.print("\tRaw Accl XYZ\t");
      Serial.print(aa.x);
      Serial.print("\t");
      Serial.print(aa.y);
      Serial.print("\t");
      Serial.print(aa.z);
      mpu.dmpGetGyro(&gy, fifoBuffer);
      Serial.print("\tRaw Gyro XYZ\t");
      Serial.print(gy.x);
      Serial.print("\t");
      Serial.print(gy.y);
      Serial.print("\t");
      Serial.print(gy.z);
    */
    Serial.println();

#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    firstMPUSensor.dmpGetQuaternion(&firstQuaternion, fifoBuffer);
    firstMPUSensor.dmpGetAccel(&aa, fifoBuffer);
    firstMPUSensor.dmpGetGravity(&gravity, &firstQuaternion);
    firstMPUSensor.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    firstMPUSensor.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &firstQuaternion);
    secondMPUSensor.dmpGetQuaternion(&secondQuaternion, fifoBuffer2);
    secondMPUSensor.dmpGetAccel(&aa2, fifoBuffer2);
    secondMPUSensor.dmpGetGravity(&gravity2, &secondQuaternion);
    secondMPUSensor.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
    secondMPUSensor.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &secondQuaternion);
    Serial.print("aworld\t");
    Serial.print(aaWorld2.x);
    Serial.print("\t");
    Serial.print(aaWorld2.y);
    Serial.print("\t");
    Serial.println(aaWorld2.z);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif
  }
}
