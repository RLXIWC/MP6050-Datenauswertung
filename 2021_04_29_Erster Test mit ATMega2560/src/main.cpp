/* Titel: Erster Test mit ATMega2560
*
* Version: v1.0
* Autor: Heiko und Jeremy
* Datum: 29.04.2021
*
* Kurze Beschreibung:
* Switch einlesen und Zustand ändern
*
* Ausgabe:
* Zustand auf serielle Konsole
*
* Anschluss weiterer Elektronik:
* Button an ATMega anschließen
*
* Changelog:
* 
*  
*/

// set pin numbers
const int buttonPin = 8;                                                                    // the number of the pushbutton pin


//################################################################//
//########################## Includes ############################//
//################################################################//

#include <Arduino.h>



//################################################################//
//###################### globale Variablen #######################//
//################################################################//

int buttonState = 0;


//################################################################//
//########################### SETUP ##############################//
//################################################################//

void setup() {

  Serial.begin(115200);  
  pinMode(buttonPin, INPUT);


}



//################################################################//
//############################ Loop  #############################//
//################################################################//

void loop() {

  buttonState = digitalRead(buttonPin);

 

  if (buttonState == HIGH)
  {
      Serial.println("1");
  } 
  else 
  {
      Serial.println("0");
  }

  delay(20);


}