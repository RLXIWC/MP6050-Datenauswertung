/*
//  Name:             MPU6050 mit ESP32 Test
//  Datum:            09.04.2021
//  Beschreibung:     Erste Versuche den MPU6050 mit einem ESP32 zum Laufen zu bringen.
//                    Verbindung über I2C, Anzeigen der Daten auf der Seriellen Konsole  
//                    Der Pin GPIO22 ist der I2C SCL beim ESP32
//                    Der Pin GPIO21 ist der I2C SDA beim ESP32
*/

////// Includes ///////

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


/////// Functions //////////

void correctValues(sensors_event_t &a, sensors_event_t &g);


///// Globale Daten //////

Adafruit_MPU6050 mpu;                                                                     // Erzeugen eines Objekts der MPU6050 Klasse

void setup() 
{
  Serial.begin(115200);    
  while (!Serial)
  {
      delay(10);                                                                         // warte bis Serielle Konsole offen     
  }

  Serial.println("Adafruit MPU6050 Test!");

  //// Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {                                                                           // Warte bis Verbindung steht
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  ///// Range des Accelerometers einstellen ///////

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) 
  {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }


///// Range des Gyro einstellen ///////


  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) 
  {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

///// Bandbreite des Filters einstellen ///////

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) 
  {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(1000);

}



/////// Main /////////

void loop() {

  sensors_event_t a, g, temp;                                                       // Sensor Event Variable für Beschleunigung, Gyrowert und Temperatur
  mpu.getEvent(&a, &g, &temp);                                                       // Hole die Daten per Referenzparameter
  //correctValues(a,g);

//// Ausgabe der Daten /////

  //Serial.println("Acceleration:");
  //Serial.print("X: ");
  Serial.print(a.acceleration.x);
  Serial.print("\t");
  //Serial.println(" m/s^2");
  //Serial.print("Y: ");
  Serial.print(a.acceleration.y);
  Serial.print("\t");
  //Serial.println(" m/s^2");
  //Serial.print("Z: ");
  Serial.print(a.acceleration.z);
    Serial.print("\t");
  //Serial.println(" m/s^2");

  //Serial.println("Rotation:");
  //Serial.print("X: ");
  Serial.print(g.gyro.x);
  Serial.print("\t");
  //Serial.println(" rad/s");
  //Serial.print("Y: ");
  Serial.print(g.gyro.y);
  Serial.print("\t");
  //Serial.println(" rad/s");
  //Serial.print("Z: ");
  Serial.println(g.gyro.z);                                                           // sollte im Stillstand 9.81 m*s^2 anzeigen

  //Serial.println(" rad/s");

  //Serial.print("Temperature: ");
  //Serial.print(temp.temperature);
  //Serial.println(" degC");

  //Serial.println("");
  delay(1000);


}


/// Korrigieren der Offsets ///
// Bitte anpassen je nach angeschlossenem Sensor
void correctValues(sensors_event_t &a, sensors_event_t &g)
{
    // Acceleretor Werte anpassen
    a.acceleration.x -= 0.66;
    a.acceleration.y += 0.36;
    a.acceleration.z += 1.51;

    // Gyro Werte anpassen
    g.gyro.x +=0.01;
    g.gyro.y +=0.03;
    g.gyro.z +=0.05;
    
}