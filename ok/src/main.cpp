// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include <Arduino.h>
// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEPeripheral.h>

// define pins (varies per shield/board)
#define BLE_REQ 10 //nrf518822 unused
#define BLE_RDY 2 //nrf518822 unused
#define BLE_RST 9//nrf518822 unused

// LED pin
#define LED_PIN 18



char PASSWORD[9]="12345678";
char WrittenPass[9];

char remoteDeviceName[6];
char myname[7] = "Dragar";
String tempstring;
int state=0;

bool someoneconnected=false;
unsigned long previousMillis = 0; // last time update
long interval = 2000; // interval at which to do something (milliseconds)


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

    void blePeripheralConnectHandler(BLECentral &central);
    void blePeripheralDisconnectHandler(BLECentral &central);
    void switchCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
    void PASSCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);

    void blinkLed(int times, int timedelay);
    void blePeripheralBondedHandler(BLECentral &central);
    void blePeripheralRemoteServicesDiscoveredHandler(BLECentral &central);
    void bleRemoteDeviceNameCharacteristicValueUpdatedHandle(BLECentral &central, BLERemoteCharacteristic &characteristic);

    void setup()
{
  Serial.begin(115200);

  // set LED pin to output mode
  pinMode(LED_PIN, OUTPUT);

  
 
  

  //bleBondStore.clearData();

  //blePeripheral.setBondStore(bleBondStore);



  blePeripheral.setAppearance(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);

  // set advertised local name and service UUID
  blePeripheral.setLocalName("LED2");
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

  // add service and characteristic
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(switchCharacteristic);
  blePeripheral.addAttribute(SWITCHDescriptor);

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

  //if (remoteDeviceNameCharacteristic.canSubscribe())
//
  //{
   // remoteDeviceNameCharacteristic.subscribe();
 // }
  

      // assign event handlers for characteristic

  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);

  PASSCharacteristic.setEventHandler(BLEWritten, PASSCharacteristicWritten);

  // begin initialization
  blePeripheral.begin();

  Serial.println(F("BLE LED Peripheral"));
  blinkLed(1,1000);
}

void loop()
{
  // poll peripheral
  blePeripheral.poll();


  if (someoneconnected==true)
  {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis > interval)
    {
      previousMillis = currentMillis;
      if (strcmp(WrittenPass, PASSWORD) != 0){
        blePeripheral.central().disconnect();
        someoneconnected=false;
      }

    }
  }

}

void blePeripheralConnectHandler(BLECentral &central)
{
  // central connected event handler
  Serial.print(F("Connected event, central: "));
  
  Serial.println(central.address());

  
  
 

    blinkLed(5,200);
    previousMillis = millis();
    someoneconnected = true;
}

void blePeripheralDisconnectHandler(BLECentral &central)
{
  // central disconnected event handler
  Serial.print(F("Disconnected event, central: "));
  someoneconnected=false;
  Serial.println(central.address());
  blinkLed(8,100);

  memset(WrittenPass, 0, sizeof(WrittenPass));
  PASSCharacteristic.setValue(0);
}

void PASSCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic){

  
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
    if (switchCharacteristic.value())
    {
      Serial.println(F("LED on"));
      digitalWrite(LED_PIN, HIGH);
      state=1;
      //delay(3000);
      //digitalWrite(LED_PIN, LOW);
      //switchCharacteristic.setValue(0);
    }
    else
    {
      Serial.println(F("LED off"));
      digitalWrite(LED_PIN, LOW);
      state=0;
    }
  }
  else
  {
  
    central.disconnect();
  }
  

}

void blinkLed(int times,int delaytime){
  for (int i=0;i<times;i++){
    digitalWrite(LED_PIN,HIGH);
    delay(delaytime);
    digitalWrite(LED_PIN,LOW);
    delay(delaytime);
    digitalWrite(LED_PIN, state);
    
  }
}

void blePeripheralBondedHandler(BLECentral &central)
{
  // central bonded event handler
  Serial.print(F("Remote bonded event, central: "));
  Serial.println(central.address());

  blinkLed(4,100);

 
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
      
  }else
  {
    blinkLed(5,100);
  }
  
}


void bleRemoteDeviceNameCharacteristicValueUpdatedHandle(BLECentral &central, BLERemoteCharacteristic &characteristic)
{
  
  memset(remoteDeviceName, 0, sizeof(remoteDeviceName));
  memcpy(remoteDeviceName, remoteDeviceKeyCharacteristic.value(), remoteDeviceKeyCharacteristic.valueLength());
  tempstring = reinterpret_cast<const char *>(remoteDeviceKeyCharacteristic.value());
  
  if (strcmp(remoteDeviceName,myname)==0)
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

