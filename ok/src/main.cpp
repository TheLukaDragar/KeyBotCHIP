// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include <Arduino.h>
// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEPeripheral.h>
#include "nrf_soc.h"
#include "nrf_nvic.h"



// define pins (varies per shield/board)
#define BLE_REQ 10 //nrf518822 unused
#define BLE_RDY 2  //nrf518822 unused
#define BLE_RST 9  //nrf518822 unused

// LED pin
#define LED_PIN 18
#define MOTOR_1 20
#define MOTOR_2 22
#define KEY1_PIN 16
//#define ANALOG_IN_PIN 6

const uint8_t sin_table[] = {0, 0,1,2,4,6,9,12,16,20,24,29,35,40,	46,	53,	59,	66,	74,	81,	88,	96,	104,112,120,128,136,144,152,160,168,175,182,190,197,203,210,216,221,227,
               232,236,240,244,247,250,252,254,255,255,255,255,255,254,252,250,247,244,240,236,232,227,221,216,210,203,197,190,182,175,168,160,152,144,136,128,120,112,104,
               96,88,81,74,66,59,	53,	46,	40,	35,	29,24,	20,	16,	12,	9,	6,	4,	2,1,0};
uint32_t counter = 0;

char PASSWORD[9] = "12345678";
char WrittenPass[9];

char remoteDeviceName[6];
char myname[7] = "Dragar";
String tempstring;
int state = 0;

int startsensing=0;
int goback=0;
long gobacktime=0;



bool someoneconnected = false;
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0; // last time update
unsigned long previousMillis4 = 0; // last time update
long interval = 2000; 
long ForceSensingInterval = 100;
int sensingTime=12000;
float sensitivity_voltage_limit=1.5;


long timeaccum=0;              // interval at which to do something (milliseconds)

// create peripheral instance, see pinouts above

BLEPeripheral blePeripheral = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);
//BLEBondStore bleBondStore;

// create service
BLEService ledService = BLEService("00001815-0000-1000-8000-00805F9B34FB");
BLEService PASSService = BLEService("0000181c-0000-1000-8000-00805F9B34FB");

// create switch characteristic
BLECharCharacteristic switchCharacteristic = BLECharCharacteristic("2a57", BLERead | BLEWrite | BLENotify);

BLEDoubleCharacteristic PASSCharacteristic = BLEDoubleCharacteristic("2a3d", BLEWrite);

BLEDescriptor SWITCHDescriptor = BLEDescriptor("2901", "LED ON/OFF");

// create remote services
BLERemoteService remoteUserDataAttributeService = BLERemoteService("181C");
// create remote characteristics
BLERemoteCharacteristic remoteDeviceKeyCharacteristic = BLERemoteCharacteristic("2a3d", BLERead);

BLEService batteryService("180F");
BLEUnsignedCharCharacteristic batteryLevelCharacteristic("2A19", BLERead);

BLEService temperatureService("180A");
BLEUnsignedIntCharacteristic temperatureCharacteristic("2A6E", BLERead);

BLEService ForceSensorService("4ccd3f02-b066-458c-af72-f1ed81c61a00");
BLEFloatCharacteristic ForceSensorCharacteristic("4ccd3f02-b066-458c-af72-f1ed81c61a01", BLERead | BLENotify);
BLEDescriptor ForceDescriptor = BLEDescriptor("2901", "Analog Value in V");
BLECharCharacteristic ForceSensorLimitCharacteristic("4ccd3f02-b066-458c-af72-f1ed81c61a02", BLERead | BLEWrite | BLENotify);

void blePeripheralConnectHandler(BLECentral &central);
void blePeripheralDisconnectHandler(BLECentral &central);
void switchCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
void SubscribedToForce(BLECentral &central, BLECharacteristic &characteristic);
void ForceSettingWritten(BLECentral &central, BLECharacteristic &characteristic);
void PASSCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
void ForceSensorLimitWritten(BLECentral &central, BLECharacteristic &characteristic);

void blinkLed(int times, int timedelay);
void blePeripheralBondedHandler(BLECentral &central);
void blePeripheralRemoteServicesDiscoveredHandler(BLECentral &central);
void bleRemoteDeviceNameCharacteristicValueUpdatedHandle(BLECentral &central, BLERemoteCharacteristic &characteristic);
unsigned char getBatteryLevel(void);
unsigned int getAnalog(void);
float getForceSensorVoltage();

int32_t temperature_data_get(void)
{
  int32_t temp;

  sd_temp_get(&temp);

  return temp / 4;
}

void setup()
{
  Serial.begin(115200);

  

  

  // set LED pin to output mode
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(PIN_A4, INPUT);
  digitalWrite(PIN_A4, LOW);


/*
  pinMode(KEY1_PIN,INPUT);
  pinMode(PIN_A1,OUTPUT);
  analogWrite(PIN_A1,255);
  */
  

  

  //bleBondStore.clearData();

  //blePeripheral.setBondStore(bleBondStore);

  blePeripheral.setAppearance(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);

  // set advertised local name and service UUID
  blePeripheral.setLocalName("LED2");
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

  // add service and characteristic
  blePeripheral.addAttribute(batteryService);
  blePeripheral.addAttribute(batteryLevelCharacteristic);

  blePeripheral.addAttribute(temperatureService);
  blePeripheral.addAttribute(temperatureCharacteristic);

  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(switchCharacteristic);
  blePeripheral.addAttribute(SWITCHDescriptor);

  blePeripheral.addAttribute(ForceSensorService);
  blePeripheral.addAttribute(ForceSensorCharacteristic);
  blePeripheral.addAttribute(ForceDescriptor);
  blePeripheral.addAttribute(ForceSensorLimitCharacteristic);

  blePeripheral.addAttribute(PASSService);
  blePeripheral.addAttribute(PASSCharacteristic);

  blePeripheral.addRemoteAttribute(remoteUserDataAttributeService);
  blePeripheral.addRemoteAttribute(remoteDeviceKeyCharacteristic);

  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  blePeripheral.setEventHandler(BLEBonded, blePeripheralBondedHandler);
  blePeripheral.setEventHandler(BLERemoteServicesDiscovered, blePeripheralRemoteServicesDiscoveredHandler);
  remoteDeviceKeyCharacteristic.setEventHandler(BLEValueUpdated, bleRemoteDeviceNameCharacteristicValueUpdatedHandle);



  // assign event handlers for characteristic

  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);

  PASSCharacteristic.setEventHandler(BLEWritten, PASSCharacteristicWritten);

  ForceSensorCharacteristic.setEventHandler(BLESubscribed, SubscribedToForce);

  ForceSensorLimitCharacteristic.setEventHandler(BLEWritten,ForceSettingWritten);

  // begin initialization
  blePeripheral.begin();

  Serial.println(F("BLE LED Peripheral"));
  blinkLed(1, 1000);

  state=0;
  switchCharacteristic.setValue('33');
  ForceSensorLimitCharacteristic.setValue(0x5);
  
  // enable low power mode and interrupt
  //sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

  
  
}

void loop()
{
  // wait for event/interrupt (low power mode)
  // Enter Low power mode
  //Serial.println(F("Sleep"));
  //sd_app_evt_wait();
  // Exit Low power mode
   //Serial.println(F("WakeUp"));
  //sd_nvic_ClearPendingIRQ(SWI2_IRQn);
  // poll peripheral
   unsigned long currentMillis = millis();
  

  if (someoneconnected == true)
  {
   

    if (currentMillis - previousMillis > interval)
    {

      

      previousMillis = currentMillis;
      if (strcmp(WrittenPass, PASSWORD) != 0)
      {
       // blePeripheral.central().disconnect();
        //someoneconnected=false;
      }
    }

    
    //press and stop
    if (currentMillis - previousMillis2 > ForceSensingInterval && startsensing!=0)
    {

      float sensorvoltage = getForceSensorVoltage();

      
       

      if (sensorvoltage>sensitivity_voltage_limit && startsensing==1)
      {

        digitalWrite(LED_PIN, HIGH);
        digitalWrite(MOTOR_1,LOW);
        switchCharacteristic.setValue('32');
        state=1;//LED ON LOCKED
        startsensing=0;
        goback=2;
        gobacktime=timeaccum;
        timeaccum=0;
       
      }

      if (sensorvoltage>sensitivity_voltage_limit && startsensing==2)
      {
        digitalWrite(LED_PIN, LOW);
        digitalWrite(MOTOR_2,LOW);
        switchCharacteristic.setValue('33');
        state=0;
        startsensing=0;
        goback=1;
        gobacktime=timeaccum;
        timeaccum=0;
      }
       ForceSensorCharacteristic.setValueLE(sensorvoltage);

      timeaccum+=ForceSensingInterval;
      if (timeaccum>=sensingTime && startsensing!=0)
      {
        digitalWrite(MOTOR_1,LOW);
        digitalWrite(MOTOR_2,LOW);
        switchCharacteristic.setValue('34');

        if (startsensing==1)
        {
          goback=2;
        }
         if (startsensing==2)
        {
          goback=1;
        }
        
        startsensing=0;
        gobacktime=timeaccum;
        timeaccum=0;
      }

      
      

      previousMillis2 = currentMillis;
     
    } 
  }

  //go back and stop doesent need someone connected to work
    if (currentMillis - previousMillis3 > ForceSensingInterval && goback!=0)
    {

      Serial.println("going back"+String(gobacktime));

      float sensorvoltage = getForceSensorVoltage();

      if (goback==1)
      {

        delay(10);
        digitalWrite(MOTOR_1,HIGH);
        goback=3;
      }
       if (goback==2)
      {
        delay(10);
        digitalWrite(MOTOR_2,HIGH);
        goback=4;
      }
      

      if (sensorvoltage<sensitivity_voltage_limit && goback==3 && gobacktime<=0 )
      {
        digitalWrite(MOTOR_1,LOW);
        goback=0;
        gobacktime=0;
        
        
      
      }
      if (sensorvoltage<sensitivity_voltage_limit && goback==4 && gobacktime<=0)
      {
        digitalWrite(MOTOR_2,LOW);
        goback=0;
        gobacktime=0;
      }
       
       
      ForceSensorCharacteristic.setValueLE(sensorvoltage);


      gobacktime-=ForceSensingInterval;
      if (sensorvoltage>=sensitivity_voltage_limit && goback!=0 && gobacktime<=0)
      {
        digitalWrite(MOTOR_1,LOW);
        digitalWrite(MOTOR_2,LOW);
        switchCharacteristic.setValue('34');
        goback=0;
        gobacktime=0;
      }
      

      previousMillis3 = currentMillis;
     
    }

  

  if (!someoneconnected && startsensing==0 && goback == 0)
  {
    // wait for event/interrupt (low power mode)
  // Enter Low power mode
  Serial.println(F("Sleep"));
  sd_app_evt_wait();
   Serial.println(F("WakeUp"));
  sd_nvic_ClearPendingIRQ(SWI2_IRQn);
  // poll peripheral
  }
  blePeripheral.poll();
  


/*
   if (currentMillis - previousMillis4 > 10 ){
     previousMillis4=currentMillis;
       analogWrite(PIN_A1,sin_table[counter]);
      counter = (counter + 1) % 100;
      Serial.println(sin_table[counter]);
   }


*/
  /*
  reading = digitalRead(KEY1_PIN);

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (state == HIGH)
      sstate = LOW;
    else
      sstate = HIGH;

    time = millis();    
  }

  analogWrite(PIN_A1, sstate);

  previous = reading;
  */
}

void blePeripheralConnectHandler(BLECentral &central)
{
  // central connected event handler
  Serial.print(F("Connected event, central: "));

  Serial.println(central.address());

  blinkLed(5, 200);
  previousMillis = millis();
  someoneconnected = true;

  unsigned char batteryLevel = getBatteryLevel();

  Serial.println("Battery level: " + String(batteryLevel) + "%");

  batteryLevelCharacteristic.setValue(batteryLevel);

  int8_t temp_c = (int8_t)(temperature_data_get() - 10);

  temperatureCharacteristic.setValue(temp_c);
  Serial.println("Chip Temperature:" + String(temp_c) + " C");
}

void blePeripheralDisconnectHandler(BLECentral &central)
{
  // central disconnected event handler
  Serial.print(F("Disconnected event, central: "));
  someoneconnected = false;
  Serial.println(central.address());
  blinkLed(8, 100);

  memset(WrittenPass, 0, sizeof(WrittenPass));
  PASSCharacteristic.setValue(0);
}

void PASSCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{

  memset(WrittenPass, 0, sizeof(WrittenPass));
  memcpy(WrittenPass, characteristic.value(), characteristic.valueLength());

  if (strcmp(WrittenPass, PASSWORD) == 0)
  {
    blinkLed(4, 100);
  }
  else
  {
    blinkLed(1, 100);
    central.disconnect();
  }
}

void switchCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  // central wrote new value to characteristic, update LED
  Serial.print(F("Characteristic event, writen: "));

  if (strcmp(WrittenPass, PASSWORD) == 0)
  {

    Serial.println((char)switchCharacteristic.value());

    if (switchCharacteristic.value() == '1')
    {

     
      Serial.println(F("LED on"));
      digitalWrite(MOTOR_1,HIGH);
      startsensing=1;
     
      //digitalWrite(LED_PIN, HIGH);
      //state=1;
     

      //delay(3000);
      //digitalWrite(LED_PIN, LOW);
      //switchCharacteristic.setValue(0);
    }
    if (switchCharacteristic.value() == '0')
    {
      Serial.println(F("LED off"));
      digitalWrite(MOTOR_2,HIGH);
      startsensing=2;
      //digitalWrite(LED_PIN, LOW);
      //state=0;
    }
    
  }
    else
  {

    central.disconnect();
  }
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}



void ForceSettingWritten(BLECentral &central, BLECharacteristic &characteristic)
{

 

  unsigned char a;
  memcpy(&a, characteristic.value(), characteristic.valueLength());

  if (0<(int)a && (int) a <=10)
  {
    sensitivity_voltage_limit=mapfloat((float)a,0.0f,10.0f,0.0f,3.0f);
  }

   Serial.println("Sensitivity set to: "+String(sensitivity_voltage_limit));
  




  /*
  if (strcmp(WrittenPass, PASSWORD) == 0)
  {
    blinkLed(2, 50);
    
  }
  else
  {
    blinkLed(1, 100);
    central.disconnect();
  }
  */
}


void SubscribedToForce(BLECentral &central, BLECharacteristic &characteristic)
{

  Serial.print(F("Notifications enabled for force "));
}

void blinkLed(int times, int delaytime)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(delaytime);
    digitalWrite(LED_PIN, LOW);
    delay(delaytime);
    digitalWrite(LED_PIN, state);
  }
}

void blePeripheralBondedHandler(BLECentral &central)
{
  // central bonded event handler
  Serial.print(F("Remote bonded event, central: "));
  Serial.println(central.address());

  blinkLed(4, 100);
}

void blePeripheralRemoteServicesDiscoveredHandler(BLECentral &central)
{
  // central remote services discovered event handler
  Serial.print(F("Remote services discovered event, central: "));
  Serial.println(central.address());

  if (remoteDeviceKeyCharacteristic.canRead())
  {
    remoteDeviceKeyCharacteristic.read();
    //blinkLed(10,800);
  }
  else
  {
    blinkLed(5, 100);
  }
}

void bleRemoteDeviceNameCharacteristicValueUpdatedHandle(BLECentral &central, BLERemoteCharacteristic &characteristic)
{

  memset(remoteDeviceName, 0, sizeof(remoteDeviceName));
  memcpy(remoteDeviceName, remoteDeviceKeyCharacteristic.value(), remoteDeviceKeyCharacteristic.valueLength());
  tempstring = reinterpret_cast<const char *>(remoteDeviceKeyCharacteristic.value());

  if (strcmp(remoteDeviceName, myname) == 0)
  {
    blinkLed(2, 10000);
  }
  else
  {
    //blinkLed(4, 1000);
    central.disconnect();
  }
  Serial.print(F("Remote device name: "));
  Serial.println(remoteDeviceName);
}

float getForceSensorVoltage()
{

  int sensorValue = analogRead(A4);
  float voltage = sensorValue * (3.3 / 1023.0);
  Serial.println("Analog pin 6: " + String(voltage) + " V");

  return voltage;
}

unsigned char getBatteryLevel(void)
{
  // Configure ADC
  NRF_ADC->CONFIG = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;

  NRF_ADC->EVENTS_END = 0; // Stop any running conversions.
  NRF_ADC->TASKS_START = 1;

  while (!NRF_ADC->EVENTS_END)
  {
  }

  uint16_t vbg_in_mv = 1200;
  uint8_t adc_max = 255;
  uint16_t vbat_current_in_mv = (NRF_ADC->RESULT * 3 * vbg_in_mv) / adc_max;

  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->TASKS_STOP = 1;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;

  return (unsigned char)((vbat_current_in_mv * 100) / 3300);
}
