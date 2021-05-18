/*
//  Name:             MPU6050 Darstellung der Daten auf einem Webserver
//  Datum:            09.04.2021
//  Beschreibung:     
*/

////// Includes ///////

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-vs-code-platformio-spiffs/  
*********/

#include <Arduino.h>
//#include <WiFi.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
//#include "SPIFFS.h"


#define M_PI 3.141592
 
// Replace with your network credentials
//const char* ssid = "Maier-Gerber";
//const char* password = "qc004buzzz2ocjrjcd3ss";

// Create AsyncWebServer object on port 80
//AsyncWebServer server(80);

// Create an Event Source on /events
//AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long generalTime = 0;
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.00;
float gyroYerror = 0.00;
float gyroZerror = 0.00;



// Init MPU6050
void initMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}
/*
void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}
*/
// Initialize WiFi
/*
void initWiFi() {
  WiFi.mode(WIFI_STA);
  //WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
}
*/
void getGyroReadings(){
  mpu.getEvent(&a, &g, &temp);// Wenn ja rechne in Winkel um --> Das ist quasi die Integration

  float gyroX_temp = g.gyro.x;                                    
  if(abs(gyroX_temp) > gyroXerror)  {                                     // Prüfen ob neue Werte größer sind wie Offset
    gyroX_temp += 0.01;
    gyroX_temp = gyroX_temp/50.00;
    gyroX_temp = (gyroX_temp *180) / M_PI;
    gyroX += gyroX_temp;
        // gyroX = gyroX_temp;
  }
  
  float gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY_temp +=0.02;
    gyroY_temp = gyroY_temp/70.00;
    gyroY_temp = (gyroY_temp *180) / M_PI;
    gyroY += gyroY_temp;
        // gyroY = gyroY_temp;
  }

  float gyroZ_temp = g.gyro.z;
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ_temp += 0.05;
    gyroZ_temp = gyroZ_temp/90.00;
    gyroZ_temp = (gyroZ_temp *180) / M_PI;
    gyroZ += gyroZ_temp;
    // gyroZ = gyroZ_temp;
  }

  // gyroX += 0.01;
  // gyroY += 0.02;
  // gyroZ += 0.05;

}

void getAccReadings() {
  mpu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  // Acceleretor Werte anpassen
  
  accX += 0.20;
  accY += 0.36;
  accZ += 1.46;


}

String getTemperature(){
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  return String(temperature);
}

void setup() {
  Serial.begin(115200);
  //initWiFi();
  //initSPIFFS();
  initMPU();
  Wire.setClock(400000);                                                                          // I2C auf 400 kHz setzen

  // Handle Web Server
  /*
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });
*/

  //server.serveStatic("/", SPIFFS, "/");

/*
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    gyroY=0;
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroY=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
  */
}

void loop() {
  if ((millis() - lastTime) > gyroDelay) {
    // Send Events to the Web Server with the Sensor Readings
    getGyroReadings();
    //events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    // Send Events to the Web Server with the Sensor Readings
    getAccReadings();
    //events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }

  if((millis() - generalTime)>1000)
  {
    Serial.print(accX);
    Serial.print("\t");
    Serial.print(accY);
    Serial.print("\t");
    Serial.print(accZ);  
    Serial.print("\t");

    Serial.print(gyroX);
    Serial.print("\t");
    Serial.print(gyroY);
    Serial.print("\t");
    Serial.println(gyroZ);  
    generalTime = millis();
  }


}