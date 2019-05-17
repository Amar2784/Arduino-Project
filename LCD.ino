
#include "DHT.h"

#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

 //Define address for the Serial LCD display.
#define I2C_ADDR  0x3f  //If 0x3f doesn't work change this to 0x27
#define BACKLIGHT_PIN  3

//Initialise the Serial LCD.
LiquidCrystal_I2C lcd(I2C_ADDR, 2,1,0,4,5,6,7); //These pin numbers are hard coded in on the serial backpack board.
int tempPin = A2;     //The temperature sensor is plugged into pin A2 of the Arduino.
int data;             //This will store the data from the sensor.
int waitTime = 10000;  //Wait
int RGBRedPin = 9;    //The red RGB LED is connected pin 9 of the Arduino.
int RGBGreenPin = 10; //The green RGB LED is connected pin 10 of the Arduino.
int RGBBluePin = 11;
int DHT11Pin = 4; 

DHT dht(DHT11Pin, DHT11);

void setup()
 {   pinMode(RGBRedPin, OUTPUT);    //Setup red RGB LED pin as an output pin.
  pinMode(RGBGreenPin, OUTPUT);  //Setup green RGB LED pin as an output pin.
  pinMode(RGBBluePin, OUTPUT); 
    lcd.begin (16,2);     //Initalize the LCD.
    lcd.setBacklightPin(3,POSITIVE);//Setup the backlight.
    lcd.setBacklight(HIGH); //Switch on the backlight.
    lcd.clear();
    Serial.begin(9600);
    dht.begin();
 }
 
void loop(){
 
     data = analogRead(tempPin);        //Read from the temperature sensor
    Serial.println(data);                //This is the raw analog reading

  // Calculate and display temperature.
  float temp = (5.0 * data * 100.0) / 1024;
  float humidity = dht.readHumidity();
  lcd.setCursor(1,0);
 lcd.print("Temp:"); lcd.print(temp); 
 lcd.setCursor(0,1);
 lcd.print("Humid:"); lcd.print(humidity); lcd.print("%");
  delay(waitTime);
  lcd.clear();
}

