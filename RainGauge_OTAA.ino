/**
   '########:::::'###::::'####:'##::: ##::'######:::'##::::'##::::'###:::::'######:::'########:
    ##.... ##:::'## ##:::. ##:: ###:: ##:'##... ##:: ##:::: ##:::'## ##:::'##... ##:: ##.....::
    ##:::: ##::'##:. ##::: ##:: ####: ##: ##:::..::: ##:::: ##::'##:. ##:: ##:::..::: ##:::::::
    ########::'##:::. ##:: ##:: ## ## ##: ##::'####: ##:::: ##:'##:::. ##: ##::'####: ######:::
    ##.. ##::: #########:: ##:: ##. ####: ##::: ##:: ##:::: ##: #########: ##::: ##:: ##...::::
    ##::. ##:: ##.... ##:: ##:: ##:. ###: ##::: ##:: ##:::: ##: ##.... ##: ##::: ##:: ##:::::::
    ##:::. ##: ##:::: ##:'####: ##::. ##:. ######:::. #######:: ##:::: ##:. ######::: ########:
   ..:::::..::..:::::..::....::..::::..:::......:::::.......:::..:::::..:::......::::........::
   
   @file RainGauge.ino
   @author minhxl, binhphuong, fferrero, ksprin

   @brief This sketch help controlling on-board sensors, measuring battery level. For detail description, 
   please visit: https://github.com/XuanMinh201/RainGauge

   @version 0.0.2
   @date 2024-06-18

   @copyright Copyright (c) 2024

*/

//converting the code from ABP activation to OTAA by Kayla Sprincis

#include "Adafruit_SHTC3.h" //http://librarymanager/All#Adafruit_SHTC3

// Set pin number
#define buttonPin PB5  

// Rain Gauge battery
#define ADC_AREF 3.0f
#define BATVOLT_R1 1.0f 
#define BATVOLT_R2 2.0f
#define BATVOLT_PIN PB4

uint16_t voltage_adc;
uint16_t voltage;

// Set Interrupt
int ledToggle;
int previousState = HIGH;
unsigned int previousPress;
volatile int buttonFlag, buttonFlag_falseDetect;
const int buttonDebounce = 50;
volatile int lastDetect = 0;

volatile int rainFlag;    // turn on this flag if it is rain
volatile int notRainFlag; // turn on this flag if it is not rain
volatile unsigned int rainGaugeCount = 0;
unsigned long time1 = 0;

uint32_t estimatedNextUplink = 0;
int rain_count;
// Set sensor variables
int16_t temper;
uint8_t humi;

// Rain Stop Time
uint64_t lastRain = 0; // the last time when it was rain
uint64_t elapsedRain;
uint64_t spendTime; // the remaining time before wake up in period OTAA

bool bucketPositionA = false; // one of the two positions of tipping-bucket
// const double bucketAmount = 0.01610595;   // inches equivalent of ml to trip tipping-bucket
//#define ABP_PERIOD   (1800000) //  sleep cycle 30m
#define OTAA_PERIOD   (360000) //  sleep cycle 3m

//time in case of heavy rain.
//#define t_rain   (300000)
#define t_rain   (60000)

// #define RAIN_STOP_TIME   (6000)
/*************************************

   LoRaWAN band setting:
     RAK_REGION_EU433
     RAK_REGION_CN470
     RAK_REGION_RU864
     RAK_REGION_IN865
     RAK_REGION_EU868
     RAK_REGION_US915
     RAK_REGION_AU915
     RAK_REGION_KR920
     RAK_REGION_AS923
     9 (REGION_AS923_2) recommand for vietnam 
 *************************************/

#define OTAA_BAND     (RAK_REGION_US915)

#define OTAA_DEVEUI  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}//8 byte 
#define OTAA_APPEUI  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}//8 byte 
#define OTAA_APPKEY  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}//16 byte



/** Packet buffer for sending */
//sending 64 bytes
uint8_t collected_data[64] = { 0 };

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

void recvCallback(SERVICE_LORA_RECEIVE_T * data)
{
  if (data->BufferSize > 0) {
    Serial.println("Something received!");
    for (int i = 0; i < data->BufferSize; i++) {
      Serial.printf("%x", data->Buffer[i]);
    }
    Serial.print("\r\n");
  }
}

void joinCallback(int32_t status)
{
    Serial.printf("Join status: %d\r\n", status);
}

void sendCallback(int32_t status)
{
  if (status == 0) {
    Serial.println("Successfully sent");
  } else {
    Serial.println("Sending failed");
  }
}

void setup()
{
  Serial.begin(115200, RAK_AT_MODE);

  // Initialize Interrupt
  Serial.println("RAKwireless Arduino Interrupt Example");
  Serial.println("------------------------------------------------------");
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), button_ISR, FALLING);

  buttonFlag = 0;
  buttonFlag_falseDetect = 0;
  lastDetect = 0;

  analogReadResolution(10);
  
  // Initialize SHTC3
  Serial.println("SHTC3 test");
  if (!shtc3.begin())
  {
    Serial.println("Couldn't find SHTC3");
     //while (1) delay(1);
  }
  Serial.println("Found SHTC3 sensor");

   // Wake-up
  api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, PA0);

  // OTAA Device EUI MSB first
  uint8_t node_device_eui[8] = OTAA_DEVEUI;
  // OTAA Application EUI MSB first
  uint8_t node_app_eui[8] = OTAA_APPEUI;
  // OTAA Application Key MSB first
  uint8_t node_app_key[16] = OTAA_APPKEY;

  if (!api.lorawan.appeui.set(node_app_eui, 8)) {
      Serial.printf("LoRaWan OTAA - set application EUI is incorrect! \r\n");
      return;
  }
  if (!api.lorawan.appkey.set(node_app_key, 16)) {
      Serial.printf("LoRaWan OTAA - set application key is incorrect! \r\n");
      return;
  }
  if (!api.lorawan.deui.set(node_device_eui, 8)) {
      Serial.printf("LoRaWan OTAA - set device EUI is incorrect! \r\n");
      return;
  }

  if (!api.lorawan.band.set(OTAA_BAND)) {
      Serial.printf("LoRaWan OTAA - set band is incorrect! \r\n");
      return;
  }
  if (!api.lorawan.deviceClass.set(RAK_LORA_CLASS_A)) {
      Serial.printf("LoRaWan OTAA - set device class is incorrect! \r\n");
      return;
  }
  if (!api.lorawan.njm.set(RAK_LORA_OTAA))	// Set the network join mode to OTAA
  {
      Serial.printf("LoRaWan OTAA - set network join mode is incorrect! \r\n");
      return;
  }
  if (!api.lorawan.join())	// Join to Gateway
  {
      Serial.printf("LoRaWan OTAA - join fail! \r\n");
      return;
  }

  /** Wait for Join success */
  while (api.lorawan.njs.get() == 0) {
      Serial.print("Wait for LoRaWAN join...");
      api.lorawan.join();
      delay(10000);
  }

  if (!api.lorawan.adr.set(true)) {
      Serial.printf("LoRaWan OTAA - set adaptive data rate is incorrect! \r\n");
      return;
  }
  if (!api.lorawan.rety.set(1)) {
      Serial.printf("LoRaWan OTAA - set retry times is incorrect! \r\n");
      return;
  }
  if (!api.lorawan.cfm.set(1)) {
      Serial.printf("LoRaWan OTAA - set confirm mode is incorrect! \r\n");
      return;
  }
  
  /** Check LoRaWan Status*/
  Serial.printf("Duty cycle is %s\r\n", api.lorawan.dcs.get()? "ON" : "OFF"); // Check Duty Cycle status
  Serial.printf("Packet is %s\r\n", api.lorawan.cfm.get()? "CONFIRMED" : "UNCONFIRMED");	// Check Confirm status
  uint8_t assigned_dev_addr[4] = { 0 };
  api.lorawan.daddr.get(assigned_dev_addr, 4);
  Serial.printf("Device Address is %02X%02X%02X%02X\r\n", assigned_dev_addr[0], assigned_dev_addr[1], assigned_dev_addr[2], assigned_dev_addr[3]);  // Check Device Address
  Serial.printf("Uplink period is %ums\r\n", OTAA_PERIOD);
  Serial.println("");
  api.lorawan.registerRecvCallback(recvCallback);
  api.lorawan.registerJoinCallback(joinCallback);
  api.lorawan.registerSendCallback(sendCallback);
}

void uplink_routine()
{
  /** Payload of Uplink */
  uint8_t data_len = 0;
  collected_data[data_len++] = (uint8_t)buttonFlag;
  collected_data[data_len++] = temper >> 8;
  collected_data[data_len++] = temper & 0xFF;
  collected_data[data_len++] = (uint8_t)humi & 0xFF;
  collected_data[data_len++] = voltage >> 8;
  collected_data[data_len++] = voltage & 0xFF;

  Serial.println("Data Packet:");
  for (int i = 0; i < data_len; i++)
  {
    Serial.printf("0x%02X ", collected_data[i]);
  }
  Serial.println("");

  /** Send the data package */
  if (api.lorawan.send(data_len, (uint8_t *)&collected_data, 2, true, 1))
  {
    Serial.println("Sending is requested");
  }
  else
  {
    Serial.println("Sending failed");
  }
}

void loop()
{
  // Read SHTC3
 
  
  //
  sensors_event_t humidity, temp;
  shtc3.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  temper = (int)(temp.temperature * 100) ;
  humi = (humidity.relative_humidity*2);
  Serial.print("Sensors values : temp = ");
  Serial.print(temper/100);
  Serial.println("deg");
  Serial.print("hum= ");
  Serial.print(humi/2);
  Serial.println("%");

  
  Serial.print("Value: ");
  Serial.println(buttonFlag);
  Serial.print(" | False Detect: ");
  Serial.println(buttonFlag_falseDetect);

  //Battery variables
  voltage_adc = (uint16_t)analogRead(BATVOLT_PIN);
  voltage = (int16_t)(voltage_adc* ADC_AREF / 1.024) * (BATVOLT_R1 + BATVOLT_R2) / (BATVOLT_R2);

  Serial.print("Voltage: ");
  Serial.println(voltage);

  Serial.print("Voltage ADC: ");
  Serial.println(voltage_adc);


  // LoRaWAN Uplink
  uplink_routine();
  buttonFlag = 0;
  rain_count = 0;

  // Set sleep until the next LoRaWAN Uplink
  Serial.printf("Try sleep %ums..", OTAA_PERIOD);
  estimatedNextUplink = millis() + OTAA_PERIOD;
  api.system.sleep.all(OTAA_PERIOD);

  // Re-check the wake up reason. If the wakeup caused by External Interrupt, go back to sleep mode
  while (estimatedNextUplink > millis())
  {
    uint32_t remainingSleepTime = estimatedNextUplink - millis();
    api.system.sleep.all(remainingSleepTime);
  }

  Serial.println("Wakeup..");
}

void button_ISR()
{
  if (rain_count == 0){
    estimatedNextUplink = estimatedNextUplink - (OTAA_PERIOD - t_rain);
  }
  rain_count++;
  int _now = millis();
  if ((_now - lastDetect) > buttonDebounce)
  {
    lastDetect = _now;
    buttonFlag++;
  }
  else
  {
    buttonFlag_falseDetect++;
  }
}

