#include <SPI.h>
#include <MFRC522.h>
//#include <SoftwareSerial.h>
#include <MAX3010x.h>
#include "filters.h"

#define SS_PIN 10
#define RST_PIN 9
#define FORCE_SENSOR_PIN A0 
#define lm35_pin A1
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
//SoftwareSerial esp(6,7);              //ESP8266

float iTEMP;
float iPR;
float iBPM;
float iO2;

//boolean thingSpeakWrite(float value1, float value2, float value3, float value4);
/*************************************************************************************************************
String ssid="ESP32";                                 // Wifi network SSID
String password ="1234567890";                         // Wifi network password
String apikey="7C5PKB2F4EGJJJ76";
boolean DEBUG=true;
int count=0;
***********************************************************************************
void showResponse(int waitTime)
{
    long t=millis();
    char c;
    while (t+waitTime>millis())
    {
      if (esp.available())
      {
        c = esp.read();
        if (DEBUG) 
        {
          Serial.print(c);
        }
      }
    }               
}
***********************************************************************************/
// Sensor (adjust to your sensor type)
MAX30102 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;
void setup() 
{
  Serial.begin(9600);  
  /**********************************************************************
  
   esp.begin(115200);
   delay(500);   
  esp.println("AT+CWMODE=1");                                             // set esp8266 as client
  showResponse(1000);
  esp.println("AT+CWJAP=\""+ssid+"\",\""+password+"\"");                  // set your home router SSID and password
  showResponse(5000);
  Serial.println();
  if (DEBUG)
  {
      Serial.println("Setup completed");
  }
      
      //***************************************************/
      
     // Initiate a serial communication
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Sensor initialized");
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }
  SPI.begin();      // Initiate  SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522
  Serial.println("Approximate your card to the reader...");
  Serial.println();


}


/****************************************************************************/

void temp(){
   int temp_adc_val;
   float temp_val;
   temp_adc_val = analogRead(lm35_pin);
   temp_val = (temp_adc_val * 4.88);	
   temp_val = (temp_val/10);	
   Serial.print("Temperature = ");
   Serial.print(temp_val);
   Serial.print(" Degree Celsius\n");
   iTEMP= temp_val;
   delay(1000);
   delay(3000);
}

/****************************************************************************/
// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void spo(){
  
  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  
  // Detect Finger using raw sensor value
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    // Reset values if the finger is removed
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if(!isnan(current_diff) && !isnan(last_diff)) {
      
      // Detect Heartbeat - Zero-Crossing
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      if(current_diff > 0) {
        crossed = false;
      }
  
      // Detect Heartbeat - Falling Edge Threshold
      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Show Results
          int bpm = 60000/(crossed_time - last_heartbeat);
          float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
          float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
          float r = rred/rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          
          if(bpm > 50 && bpm < 250) {
            // Average?
            if(kEnableAveraging) {
              int average_bpm = averager_bpm.process(bpm);
              int average_r = averager_r.process(r);
              int average_spo2 = averager_spo2.process(spo2);
  
              // Show if enough samples have been collected
              if(averager_bpm.count() >= kSampleThreshold) {
                Serial.print("Time (ms): ");
                Serial.println(millis()); 
                Serial.print("Heart Rate (avg, bpm): ");
                Serial.println(average_bpm);
                Serial.print("R-Value (avg): ");
                Serial.println(average_r);  
                Serial.print("SpO2 (avg, %): ");
                Serial.println(average_spo2);  
              }
            }
            else {
              Serial.print("Time (ms): ");
              Serial.println(millis()); 
              Serial.print("Heart Rate (current, bpm): ");
              Serial.println(bpm);  
              Serial.print("R-Value (current): ");
              Serial.println(r);
              Serial.print("SpO2 (current, %): ");
              Serial.println(spo2);   
              iBPM=bpm;
              iO2=spo2;
            }
          }

          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }
  
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }
}

/******************************************************************************/

void Press() {
  int analogReading = analogRead(FORCE_SENSOR_PIN);

  Serial.print("Force sensor reading = ");
  Serial.print(analogReading); // print the raw analog reading

  if (analogReading < 10)       // from 0 to 9
    Serial.println(" -> no pressure");
  else if (analogReading < 200) // from 10 to 199
    Serial.println(" -> light touch");
  else if (analogReading < 500) // from 200 to 499
    Serial.println(" -> light squeeze");
  else if (analogReading < 800) // from 500 to 799
    Serial.println(" -> medium squeeze");
  else // from 800 to 1023
    Serial.println(" -> big squeeze");
  
 iPR = analogReading; 
  delay(1000);
}

/*******************************************************************/
/**********************************************************************************************

boolean thingSpeakWrite( float iTEMP,float iPR, float iBPM, float iO2 )
{
  String cmd = "AT+CIPSTART=\"TCP\",\"";                                                    
  cmd += "184.106.153.149";                                                                
  cmd += "\",80";
  
  esp.println(cmd);
  if (DEBUG)
  {
    //Serial.println(cmd);
  }
  if(esp.find("Error"))
  {
    if (DEBUG) 
    {
      //Serial.println("AT+CIPSTART error");
    }
    return false;
  }
  
  String getStr = "GET /update?api_key=";             // prepare GET string
  getStr += apikey;
  
  getStr +="&field1=";
  getStr += float(iTEMP);
  getStr +="&field2=";
  getStr += float(iPR);
  getStr +="&field3=";
  getStr += float(iBPM);
  getStr +="&field4=";
  getStr += float(iO2);
  getStr += "\r\n\r\n";
  
  cmd = "AT+CIPSEND=";
  cmd += String(getStr.length());
  esp.println(cmd);
  if (DEBUG)  //Serial.println(cmd);
  
  delay(100);
  if(esp.find(">"))
  {
    esp.print(getStr);
    if (DEBUG) 
    {
      //Serial.print(getStr);
    }
  }
  else
  {
    esp.println("AT+CIPCLOSE");
    if (DEBUG)   
    {
      //Serial.println("AT+CIPCLOSE");
    }
    return false;
  }
  return true;
}

******************************************************************************************/


void loop() 
{
  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return;
  }
  //Show UID on serial monitor
  Serial.print("UID tag :");
  String content= "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) 
  {
     Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
     Serial.print(mfrc522.uid.uidByte[i], HEX);
     content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  Serial.println();
  Serial.print("Message : ");
  content.toUpperCase();
  if (content.substring(1) == "0C EF 08 38") //change here the UID of the card/cards that you want to give access
  {
   Serial.println("Authorized access");
     Serial.println("SPO2");
     for(int i =0;i<10000;i++){
       spo();
     }
   temp();
   
   Press();
/*
  thingSpeakWrite(iTEMP,iPR,iBPM,iO2);
  Serial.println("DATA UPLOADED TO CLOUD");
  Serial.println("");
  delay(2000);
*/  
  }
 
 else   {
    Serial.println(" Access denied");
    delay(3000);
  }
//******************************************************************************************
  
  
} 