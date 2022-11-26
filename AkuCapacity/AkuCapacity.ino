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

#include <Wire.h>
#include "DFRobot_INA219.h"
#include "SSD1306Wire.h"

#define RELAY_PIN 0

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, SDA, SCL);   // SDA jest pod IO8, SCL jest  pod IO9, ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h


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

void setup() {
  Serial.begin(115200);
  //Open the serial port
  while(!Serial);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  
  Serial.println();
  //Initialize the sensor
  while(ina219.begin() != true) {
      Serial.println("INA219 begin faild");
      delay(2000);
  }
  //Linear calibration
  ina219.linearCalibrate(/*The measured current before calibration*/ina219Reading_mA, /*The current measured by other current testers*/extMeterReading_mA);
  Serial.println();

  // Initialising the UI will init the display too.
  display.init();
  display.clear();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);// były też _10 i _16 i _24
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
}

void loop() {
  

  if ((millis() - lastTime) > timerDelay) {
    display.clear();

    voltage = ina219.getBusVoltage_V();
    current = -ina219.getCurrent_mA();

    if (voltage < 11.1) {
      measurmentFlag = false;
      digitalWrite(RELAY_PIN, LOW);
    }
    

    //Serial.print("BusVoltage:   ");
    //Serial.println(voltage, 2);

    char buff[10];
    sprintf(buff, "%.2f V", voltage);
    display.drawString(80, 0, buff);

    sprintf(buff, "%.0f mA", current);
    display.drawString(100, 16, buff);


    char timeBuff[10];
    int mins = dischargingTime_sec_/60;                                        //Number of seconds in an hour
    int sec = (dischargingTime_sec_-mins*60);                                             //Remove the number of hours and calculate the minutes.
    sprintf(timeBuff, "%d:%02d", mins, sec);                               //formatowanie teksyu 0:00
    display.drawString(110,32,timeBuff);


    
    if (measurmentFlag) batCapacity += current / 3600;
    sprintf(buff, "%.0f mAh", batCapacity);
    display.drawString(120, 48, buff);

    if (measurmentFlag) display.drawString(50, 32, "ON"); else  display.drawString(50, 32, "OFF");

    display.display();

    
    lastTime = millis();
    if (measurmentFlag) dischargingTime_sec_++;    
  }

}
