//Lib fuer I2C Sensor MPU6050 und anderen Sensoren
#include "I2Cdev.h"

//MU6050 Lib von I2CDev
#include "MPU6050_6Axis_MotionApps_V6_12.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

//Erstellen eines Sensorobjektes, default ist Standardadresse 0x68, 0x69 ist zweite Adresse
MPU6050 firstMPUSensor;
MPU6050 secondMPUSensor(0x69);

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

//Definieren der Pins fuer den I2C
#define SDA 21
#define SCL 23

// MPU Status und Kontrollevariablen
bool dmpReady = false;            // set true if DMP init was successful
uint8_t mpuIntStatus;             // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus2;            // holds actual interrupt status byte from MPU
uint8_t devStatus;                // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;               // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize, packetSize2; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;               // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];           // FIFO storage buffer
uint8_t fifoBuffer2[64];          // FIFO storage buffer

// orientation/motion vars
Quaternion firstQuaternion, secondQuaternion, productResult; // [w, x, y, z]         quaternion container
VectorInt16 aa, aa2;                                         // [x, y, z]            accel sensor measurements
VectorInt16 gy, gy2;                                         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal, aaReal2;                                 // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld, aaWorld2;                               // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity, gravity2;                               // [x, y, z]            gravity vector
float euler[3];                                              // [psi, theta, phi]    Euler angle container
float ypr[3];                                                // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

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

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

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
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
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
