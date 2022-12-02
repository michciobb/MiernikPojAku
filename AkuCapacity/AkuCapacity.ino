/*!
 *@file getVoltageCurrentPower.ino
 *@brief Get the current, voltage, and power of electronic devices.
 *@copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [fengli](li.feng@dfrobot.com)
 *@version  V1.0
 *@date  2022-3-1
 *@url https://github.com/DFRobot/DFRobot_INA219
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
#include "DFRobot_INA219.h"
//#include "SSD1306Wire.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//#define USE_SERIAL

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32 - mój LCD duży działa z adresem 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

#define RELAY_PIN 10

// Initialize the OLED display using Arduino Wire:
//SSD1306Wire display(0x3c, SDA, SCL);   // SDA jest pod IO8, SCL jest  pod IO9, ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h



/**
 * @fn DFRobot_INA219_IIC
 * @brief pWire I2C controller pointer
 * @param i2caddr  I2C address
 * @n INA219_I2C_ADDRESS1  0x40   A0 = 0  A1 = 0
 * @n INA219_I2C_ADDRESS2  0x41   A0 = 1  A1 = 0
 * @n INA219_I2C_ADDRESS3  0x44   A0 = 0  A1 = 1
 * @n INA219_I2C_ADDRESS4  0x45   A0 = 1  A1 = 1	 
  */
DFRobot_INA219_IIC     ina219(&Wire, INA219_I2C_ADDRESS1);

// Revise the following two paramters according to actual reading of the INA219 and the multimeter
// for linearly calibration
float ina219Reading_mA = 1000;
float extMeterReading_mA = 98;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 1000;

float voltage = 0;
float current = 0;

int dischargingTime_sec_ = 0;

float batCapacity = 0;
bool measurmentFlag = 1;

void drawString(int x, int y, char* buf){
  display.setCursor(x, y);     // Start at top-left corner
  display.print(buf);

}

void setup() {
  #if defined (USE_SERIAL)
    Serial.begin(115200);
    //Open the serial port
    while(!Serial);
  #endif

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
    #if defined (USE_SERIAL) 
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    #endif


  display.clearDisplay();

  //display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setFont(&FreeSans9pt7b);



  display.clearDisplay();
/*
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
*/
 

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  
  Serial.println();
  //Initialize the sensor
  while(ina219.begin() != true) {
      #if defined (USE_SERIAL) 
        Serial.println("INA219 begin faild");
      #endif
      delay(2000);
  }
  //Linear calibration
  ina219.linearCalibrate(/*The measured current before calibration*/ina219Reading_mA, /*The current measured by other current testers*/extMeterReading_mA);
  #if defined (USE_SERIAL) 
    Serial.println();
  #endif

}

void loop() {
  

  if ((millis() - lastTime) > timerDelay) {
    display.clearDisplay();

    voltage = ina219.getBusVoltage_V() + 1.2; //1.2V spadek na dwóch diodach prostowniczych
    current = -ina219.getCurrent_mA();

    //uwzględniamy nieliniowość mostka Gretza
    voltage +=(current * 0.5) / 1000;

    if (voltage < 11.1) {
      measurmentFlag = false;
      digitalWrite(RELAY_PIN, LOW);
    }
    
    #if defined (USE_SERIAL)
      Serial.print("BusVoltage:   ");
      Serial.println(voltage, 2);

      Serial.print("BusCurrent:   ");
      Serial.println(current, 4);
      Serial.println("");
    #endif

    char buff[13];
    sprintf(buff, "%9.2f V", voltage);

    drawString(0, 15, buff);

    sprintf(buff, "%.0f mA", current);
    drawString(30, 31, buff);




    char timeBuff[10];
    int mins = dischargingTime_sec_/60;                                        //Number of seconds in an hour
    int sec = (dischargingTime_sec_-mins*60);                                             //Remove the number of hours and calculate the minutes.
    if (measurmentFlag){
      sprintf(timeBuff, "ON     %d:%02d", mins, sec);                               //formatowanie teksyu 0:00
    }
    else{
      sprintf(timeBuff, "OFF    %d:%02d", mins, sec);
    }  
    drawString(0,47,timeBuff);


    
    if (measurmentFlag) batCapacity += current / 3600;
    sprintf(buff, "%04.0f mAh", batCapacity);
    drawString(0, 63, buff);

    //if (measurmentFlag) drawString(0, 32, "ON"); else  drawString(0, 32, "OFF");

    

    display.display();

    
    lastTime = millis();
    if (measurmentFlag) dischargingTime_sec_++;    
  }

}
