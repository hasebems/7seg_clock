 /* ========================================
 *
 *  Multi Display 
 *    description: Main Loop
 *    for ESP32 DevKitC
 *
 *  Copyright(c)2022- Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Arduino_JSON.h>
#include "time.h"

#include "Zanshin_BME680.h"  // Include the BME680 Sensor library
/**************************************************************************************************
** Declare all program constants                                                                 **
**************************************************************************************************/
const uint32_t SERIAL_SPEED{115200};  ///< Set the baud rate for Serial I/O

/**************************************************************************************************
** Declare global variables and instantiate classes                                              **
**************************************************************************************************/
BME680_Class BME680;  ///< Create an instance of the BME680 class
///< Forward function declaration with default value for sea level
float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  /*!
  @brief     This converts a pressure measurement into a height in meters
  @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
  @param[in] press    Pressure reading from BME680
  @param[in] seaLevel Sea-Level pressure in millibars
  @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()


//------------------------------------------------------------------
const char* ssid       = "xxxxxxxx";
const char* password   = "xxxxxxxx";

const char* ntpServer = "ntp.nict.jp";      //"pool.ntp.org";
constexpr long  gmtOffset_sec = 9 * 3600;   // time difference 9hours
constexpr int   daylightOffset_sec = 0;     // no summer time

constexpr uint8_t COM_DIG1 = 15;
constexpr uint8_t COM_DIG2 = 2;
constexpr uint8_t COM_DIG3 = 0;
constexpr uint8_t COM_SW   = 4;
constexpr uint8_t DRIVE_A  = 14;
constexpr uint8_t DRIVE_B  = 13;
constexpr uint8_t DRIVE_C  = 27;
constexpr uint8_t DRIVE_D  = 26;
constexpr uint8_t DRIVE_E  = 33;
constexpr uint8_t DRIVE_F  = 25;
constexpr uint8_t DRIVE_G  = 12;
constexpr uint8_t DRIVE_DP = 32;

#define MAX_DIGIT       4                 //  Digits
#define MATRIX_COM      (MAX_DIGIT+1)     //  LED Matrix Common Cathode
#define LED_CNT         2                 //  7seg LED
#define MAX_MATRIX_COM  (MATRIX_COM*LED_CNT)  //  Max Matrix Common Cathode

const uint8_t COM_TABLE[MAX_MATRIX_COM][4] = {
  { LOW, LOW, LOW, LOW },
  { LOW, LOW, HIGH,LOW },
  { HIGH,LOW, LOW, LOW },
  { HIGH,LOW,HIGH, LOW },
  { LOW,HIGH, LOW, LOW },
  { LOW, LOW, LOW, HIGH},
  { LOW, LOW,HIGH, HIGH},
  { HIGH,LOW, LOW, HIGH},
  { HIGH,LOW,HIGH, HIGH},
  { LOW,HIGH, LOW, HIGH},
};
#define MAX_MATRIX_DRV  8               // Max Matrix Drive
const uint8_t DRV_TABLE[MAX_MATRIX_DRV] = {
  DRIVE_A,DRIVE_B,DRIVE_C,DRIVE_D,DRIVE_E,DRIVE_F,DRIVE_G,DRIVE_DP
};
const uint8_t CHARACTER_TABLE[33][MAX_MATRIX_DRV] = {
  {HIGH,HIGH,HIGH,HIGH,HIGH,HIGH, LOW, LOW}, // 0
  { LOW,HIGH,HIGH, LOW, LOW, LOW, LOW, LOW}, // 1
  {HIGH,HIGH, LOW,HIGH,HIGH, LOW,HIGH, LOW}, // 2
  {HIGH,HIGH,HIGH,HIGH, LOW, LOW,HIGH, LOW}, // 3
  { LOW,HIGH,HIGH, LOW, LOW,HIGH,HIGH, LOW}, // 4
  {HIGH, LOW,HIGH,HIGH, LOW,HIGH,HIGH, LOW}, // 5
  {HIGH, LOW,HIGH,HIGH,HIGH,HIGH,HIGH, LOW}, // 6
  {HIGH,HIGH,HIGH, LOW, LOW, LOW, LOW, LOW}, // 7
  {HIGH,HIGH,HIGH,HIGH,HIGH,HIGH,HIGH, LOW}, // 8
  {HIGH,HIGH,HIGH,HIGH, LOW,HIGH,HIGH, LOW}, // 9  
  { LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}, // 10: ''  nothing
  { LOW, LOW, LOW, LOW, LOW, LOW,HIGH, LOW}, // 11: '-'
  {HIGH,HIGH, LOW, LOW, LOW, LOW, LOW, LOW}, // 12: ':' no 7seg Digits
  { LOW, LOW,HIGH, LOW, LOW, LOW, LOW, LOW}, // 13: '°' no 7seg Digits
  { LOW, LOW, LOW,HIGH,HIGH, LOW,HIGH, LOW}, // 14: 'c'
  {HIGH,HIGH, LOW, LOW,HIGH,HIGH,HIGH, LOW}, // 15: 'P'
  { LOW, LOW,HIGH, LOW,HIGH,HIGH,HIGH, LOW}, // 16: 'h'
  { LOW, LOW,HIGH,HIGH,HIGH, LOW,HIGH, LOW}, // 17: 'o'
  {HIGH, LOW, LOW,HIGH,HIGH,HIGH,HIGH, LOW}, // 18: 'E'
  {HIGH,HIGH,HIGH, LOW,HIGH,HIGH,HIGH, LOW}, // 19: 'A'
  {HIGH,HIGH,HIGH,HIGH,HIGH,HIGH, LOW, HIGH}, // 20: 0. with dot ↓
  { LOW,HIGH,HIGH, LOW, LOW, LOW, LOW, HIGH}, // 21: 1.
  {HIGH,HIGH, LOW,HIGH,HIGH, LOW,HIGH, HIGH}, // 22: 2.
  {HIGH,HIGH,HIGH,HIGH, LOW, LOW,HIGH, HIGH}, // 23: 3.
  { LOW,HIGH,HIGH, LOW, LOW,HIGH,HIGH, HIGH}, // 24: 4.
  {HIGH, LOW,HIGH,HIGH, LOW,HIGH,HIGH, HIGH}, // 25: 5.
  {HIGH, LOW,HIGH,HIGH,HIGH,HIGH,HIGH, HIGH}, // 26: 6.
  {HIGH,HIGH,HIGH, LOW, LOW, LOW, LOW, HIGH}, // 27: 7.
  {HIGH,HIGH,HIGH,HIGH,HIGH,HIGH,HIGH, HIGH}, // 28: 8.
  {HIGH,HIGH,HIGH,HIGH, LOW,HIGH,HIGH, HIGH}, // 29: 9. 
  { LOW,HIGH,HIGH, LOW,HIGH,HIGH,HIGH, LOW},  // 30: 'H'
  { LOW, LOW, LOW, LOW, LOW, LOW, LOW,HIGH},  // 31: '.' only dot
  { LOW,HIGH, LOW, LOW,HIGH, LOW,HIGH,HIGH}   // 32: '/.' % with index13
};
//------------------------------------------------------------------
hw_timer_t * timer = NULL;
uint8_t led_dynamic_cnt;
uint8_t led_num[LED_CNT][MATRIX_COM] = {0};
uint8_t flow_counter;
//------------------------------------------------------------------
// Interrupt
void ARDUINO_ISR_ATTR onTimer(){
  for (int i=0; i<MAX_MATRIX_DRV; ++i){
    digitalWrite(DRV_TABLE[i], LOW);
  }
  digitalWrite(COM_DIG1, COM_TABLE[led_dynamic_cnt][0]);
  digitalWrite(COM_DIG2, COM_TABLE[led_dynamic_cnt][1]);
  digitalWrite(COM_DIG3, COM_TABLE[led_dynamic_cnt][2]);
  digitalWrite(COM_SW, COM_TABLE[led_dynamic_cnt][3]);

  uint8_t chara = led_num[led_dynamic_cnt/MATRIX_COM][led_dynamic_cnt%MATRIX_COM];
  for (int j=0; j<MAX_MATRIX_DRV; ++j){
    digitalWrite(DRV_TABLE[j], CHARACTER_TABLE[chara][j]);
  }
  led_dynamic_cnt++;
  if (led_dynamic_cnt == MAX_MATRIX_COM){led_dynamic_cnt = 0;}
}
//------------------------------------------------------------------
void setup() {

  // GPIO setting
  pinMode(COM_DIG1, OUTPUT);
  pinMode(COM_DIG2, OUTPUT);
  pinMode(COM_DIG3, OUTPUT);
  pinMode(COM_SW, OUTPUT);
  pinMode(DRIVE_A, OUTPUT);
  pinMode(DRIVE_B, OUTPUT);
  pinMode(DRIVE_C, OUTPUT);
  pinMode(DRIVE_D, OUTPUT);
  pinMode(DRIVE_E, OUTPUT);
  pinMode(DRIVE_F, OUTPUT);
  pinMode(DRIVE_G, OUTPUT);
  pinMode(DRIVE_DP, OUTPUT);

  digitalWrite(COM_DIG1, LOW);
  digitalWrite(COM_DIG2, LOW);
  digitalWrite(COM_DIG3, LOW);
  digitalWrite(COM_SW, LOW);
  digitalWrite(DRIVE_A, LOW);
  digitalWrite(DRIVE_B, LOW);
  digitalWrite(DRIVE_C, LOW);
  digitalWrite(DRIVE_D, LOW);
  digitalWrite(DRIVE_E, LOW);
  digitalWrite(DRIVE_F, LOW);
  digitalWrite(DRIVE_G, LOW);
  digitalWrite(DRIVE_DP, LOW);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      digitalWrite(DRIVE_DP, HIGH);
  }
  digitalWrite(DRIVE_DP, LOW);

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  //  BME680
  //Serial.begin(SERIAL_SPEED);  // Start serial port at Baud rate
  while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    //Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  //Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  //Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "C" symbols
  BME680.setGas(320, 150);  // 320c for 150 milliseconds

  //  variables
  led_dynamic_cnt = 0;
  flow_counter = 0;

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler 
  //(see ESP32 Technical Reference Manual for more info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every 1m second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000, true);

  // Start an alarm
  timerAlarmEnable(timer);
}
//------------------------------------------------------------------
void loop() {
  struct tm timeinfo;
  static int32_t  temp=0, humidity=0, pressure=0, gas=0;  // BME readings

  if (!getLocalTime(&timeinfo)){
    for (int i=0; i<MAX_MATRIX_COM; ++i){ led_num[i/MATRIX_COM][i%MATRIX_COM] = 11;}
    led_num[0][4] = 10;
  }

  else {
    if (timeinfo.tm_sec%10 == 0){
      //  every 10sec
      BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
    }

    //  7seg Display 
    led_num[0][0] = timeinfo.tm_hour/10;
    led_num[0][1] = timeinfo.tm_hour%10;
    led_num[0][2] = timeinfo.tm_min/10;
    led_num[0][3] = timeinfo.tm_min%10;
    led_num[0][4] = (timeinfo.tm_sec%2)?10:12;
    flow_counter = timeinfo.tm_sec/2;
    switch (flow_counter%5){
      default:{
        for(int i=0; i<MAX_DIGIT; ++i){led_num[1][i] = 11;}
        led_num[1][4] = 10;
        break;
      }
      case 1:{  //  temperature
        //sprintf(buf, "%4d %3d.%02d", (loopCounter - 1) % 9999,  // Clamp to 9999,
        //        (int8_t)(temp / 100), (uint8_t)(temp % 100));   // Temp in decidegrees
        if (temp > 0){
          led_num[1][0] = (temp/1000)!=0?(temp/1000):10;
          led_num[1][1] = ((temp/100)%10)+20; // +20 -> dot
          led_num[1][2] = (temp%100)/10;
        }
        else if (temp > -1000){ // -0.1 .. -9.9
          led_num[1][0] = 11; // '-'
          led_num[1][1] = ((-temp/100)%10)+20; // +20 -> dot
          led_num[1][2] = (-temp%100)/10;
        }
        else {
          led_num[1][0] = led_num[1][1] = led_num[1][2] = 18; //'EEE'
        }
        led_num[1][3] = 14;
        led_num[1][4] = 13;
        break;
      }
      case 2:{  //  humidity
        //sprintf(buf, "%3d.%03d", (int8_t)(humidity / 1000),
        //      (uint16_t)(humidity % 1000));  // Humidity milli-pct
        led_num[1][0] = (humidity/10000)!=0?(humidity/10000):10;
        led_num[1][1] = ((humidity/1000)%10)+20; // +20 -> dot
        led_num[1][2] = (humidity%1000)/100;
        led_num[1][3] = 32;
        led_num[1][4] = 13;
        break;
      }
      case 3:{  //  Air Pressure (100times of Hect Pascal)
        //sprintf(buf, "%7d.%02d", (int16_t)(pressure / 100),
        //      (uint8_t)(pressure % 100));  // Pressure Pascals
        led_num[1][0] = (pressure/100000)!=0?pressure/100000:10;
        led_num[1][1] = (pressure/10000)%10;
        led_num[1][2] = (pressure/1000)%10;
        led_num[1][3] = (pressure/100)%10;
        led_num[1][4] = 10;
        break;
      }
//      case 4:{  // Gasセンサー
          //sprintf(buf, "%4d.%02d\n", (int16_t)(gas / 100), (uint8_t)(gas % 100));  // Resistance milliohms
//        led_num[1][0] = (gas/100000)!=0?gas/100000:10;
//        led_num[1][1] = (gas/10000)%10;
//        led_num[1][2] = (gas/1000)%10;
//        led_num[1][3] = 17;
//        led_num[1][4] = 10;
//        break;
//      }
    }
  }
}
