// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include <Arduino.h>
// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEPeripheral.h>
#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "nrf_sdm.h"
extern "C"
{
#include "fstorage.h"
#include "softdevice_handler.h"
#include "app_error.h"
#include "nordic_common.h"
#include "ble_stack_handler_types.h"
#include "ant_stack_handler_types.h"
#include "nrf_assert.h"
#include "sdk_errors.h"
#include "section_vars.h"
#include "fstorage_internal_defs.h"
}
#define NUM_PAGES 4
#if defined(NRF51)
#define PAGE_SIZE_WORDS 256
#endif
// define pins (varies per shield/board)
#define BLE_REQ 10 // nrf518822 unused
#define BLE_RDY 2  // nrf518822 unused
#define BLE_RST 9  // nrf518822 unused
// LED pin
#define LED_PIN 18
#define MOTOR_1 20
#define MOTOR_2 22
#define MID_SENSOR 21
//#define MID_SENSOR 16 // FOR TESTING ONLY key 1
//#define ANALOG_IN_PIN 6
#define LED_0 23
#define LED_1 24
#define LED_2 25
const uint8_t sin_table[] = {0, 0, 1, 2, 4, 6, 9, 12, 16, 20, 24, 29, 35, 40, 46, 53, 59, 66, 74, 81, 88, 96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 175, 182, 190, 197, 203, 210, 216, 221, 227,
                             232, 236, 240, 244, 247, 250, 252, 254, 255, 255, 255, 255, 255, 254, 252, 250, 247, 244, 240, 236, 232, 227, 221, 216, 210, 203, 197, 190, 182, 175, 168, 160, 152, 144, 136, 128, 120, 112, 104,
                             96, 88, 81, 74, 66, 59, 53, 46, 40, 35, 29, 24, 20, 16, 12, 9, 6, 4, 2, 1, 0};
uint32_t counter = 0;
char PASSWORD[9] = "12345678";
char WrittenPass[9];
char remoteDeviceName[6];
char myname[7] = "Dragar";
String tempstring;
int state = 0;
int check_mid = 0;
int mid_motor = 0;
int startsensing = 0;
int goback = 0;
long gobacktime = 0;
bool someoneconnected = false;
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0; // last time update
unsigned long previousMillis4 = 0; // last time update
long interval = 2000;              // auth interval
long ForceSensingInterval = 100;
int sensingTime = 7000;
int sensingTimeMID = 7000;
float sensitivity_voltage_limit = 1.5;
float sensitivity_voltage_limit2 = 1.5;
long MIN_LOCKING_TIME = 1000;
long MIN_UNLOCKING_TIME = 1000;
long MAX_LOCKING_TIME = 5000;
long MAX_UNLOCKING_TIME = 5000;
long LOCKING_TIME = 1000;
long UNLOCKING_TIME = 1000; //pazi more bit toliko da vsaj spremeni mid sensor
int calibrationinprogress = 0;
int calibrationstatus = 0;
uint32_t *addr;
static uint8_t fs_callback_flag;
long timeaccum = 0;
long timeaccumMID1 = 0;
long timeaccumMID2 = 0; // interval at which to do something (milliseconds)
int mid = 0;          // the current reading from the input pin
int lastMidState = LOW; 
int midBeforeBack = 0;
long MaxgObacktime = 8000;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 10; 
int midBefore=0;
int mid_reading=0;
// create peripheral instance, see pinouts above
BLEPeripheral blePeripheral = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);
// BLEBondStore bleBondStore;
//  create service
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
BLECharCharacteristic ForceSensorLimitCharacteristic2("4ccd3f02-b066-458c-af72-f1ed81c61a04", BLERead | BLEWrite | BLENotify);
BLECharCharacteristic ManualCharacteristic("4ccd3f02-b066-458c-af72-f1ed81c61a03", BLERead | BLEWrite | BLENotify);
BLEDescriptor CalibrationDescriptor = BLEDescriptor("2901", "Used for calibration");
BLECharCharacteristic CalibrateCharacteristic("4ccd3f02-b066-458c-af72-f1ed81c61a05", BLERead | BLEWrite | BLENotify);


void blePeripheralConnectHandler(BLECentral &central);
void blePeripheralDisconnectHandler(BLECentral &central);
void switchCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
void SubscribedToForce(BLECentral &central, BLECharacteristic &characteristic);
void SubscribedToCalibrate(BLECentral &central, BLECharacteristic &characteristic);

void ForceSettingWritten(BLECentral &central, BLECharacteristic &characteristic);
void ForceSettingWritten2(BLECentral &central, BLECharacteristic &characteristic);
void PASSCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic);
void ForceSensorLimitWritten(BLECentral &central, BLECharacteristic &characteristic);
void ManualWritten(BLECentral &central, BLECharacteristic &characteristic);
void CalibrateWritten(BLECentral &central, BLECharacteristic &characteristic);
void blinkLed(int times, int timedelay);
void blePeripheralBondedHandler(BLECentral &central);
void blePeripheralRemoteServicesDiscoveredHandler(BLECentral &central);
void bleRemoteDeviceNameCharacteristicValueUpdatedHandle(BLECentral &central, BLERemoteCharacteristic &characteristic);
unsigned char getBatteryLevel(void);
unsigned int getAnalog(void);
float getForceSensorVoltage();
/** @brief Function for erasing a page in flash.
 *
 * @param page_address Address of the first word in the page to be erased.
 */
static void flash_page_erase(uint32_t *page_address)
{
  // Turn on flash erase enable and wait until the NVMC is ready:
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
    // Do nothing.
  }
  // Erase page:
  NRF_NVMC->ERASEPAGE = (uint32_t)page_address;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
    // Do nothing.
  }
  // Turn off flash erase enable and wait until the NVMC is ready:
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
    // Do nothing.
  }
}
/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
static void flash_word_write(uint32_t *address, uint32_t value)
{
  // Turn on flash write enable and wait until the NVMC is ready:
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
  }
  *address = value;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
    // Do nothing.
  }
  // Turn off flash write enable and wait until the NVMC is ready:
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
    // Do nothing.
  }
}
int32_t temperature_data_get(void)
{
  int32_t temp;
  sd_temp_get(&temp);
  return temp / 4;
}
void eraseflash()
{
  int32_t pageNo = (uint32_t)addr / NRF_FICR->CODEPAGESIZE;
  Serial.println("erasing PAGE" + String(pageNo));
  uint32_t ret = sd_flash_page_erase(pageNo);
  while (ret == NRF_ERROR_BUSY)
  {
    Serial.println("erasing flash");
  }
  Serial.println("OK erased" + String(ret == NRF_SUCCESS));
}
void writetoflash(uint32_t *addrr)
{
  uint32_t t = 1;
  uint8_t tt = 2;
  uint8_t ttt = 3;
  uint32_t *_flashPageStartAddress = ((uint32_t *)(NRF_FICR->CODEPAGESIZE * (NRF_FICR->CODESIZE - 1 - (uint32_t)0)));
  Serial.println((unsigned int)addrr, HEX);
  uint32_t ret = sd_flash_write(addrr, (uint32_t *)t, sizeof(t));
  while (ret == NRF_ERROR_BUSY)
  {
    Serial.println(String(ret));
  }
  Serial.println("ok" + String(ret == NRF_ERROR_INVALID_ADDR));
}
static void sys_evt_dispatch(uint32_t sys_evt)
{
  // ble_advertising_on_sys_evt(sys_evt);
  fs_sys_event_handler(sys_evt);
}
static void fs_evt_handler(fs_evt_t const *const evt, fs_ret_t result)
{
  Serial.println("CALLBACK");
  if (result != FS_SUCCESS)
  {
    Serial.println("fs handler error");
    fs_callback_flag = 0;
  }
  else
  {
    Serial.println("fs handler SUCESS");
    fs_callback_flag = 0;
  }
}
FS_REGISTER_CFG(fs_config_t fs_config) =
    {
        .p_start_addr = NULL,
        .p_end_addr = NULL,
        .callback = fs_evt_handler, // Function for event callbacks.
        .num_pages = NUM_PAGES,     // Number of physical flash pages required.
        .priority = 0xFE            // Priority for flash usage.
};
static void power_manage(void)
{
  uint32_t err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
}
static uint32_t const *address_of_page(uint16_t page_num)
{
  return fs_config.p_start_addr + (page_num * PAGE_SIZE_WORDS);
}
void *p1()
{
  uint32_t forcelimit = (uint32_t) * (fs_config.p_start_addr);
  Serial.println("Now " + String(forcelimit));
}
void *w1(uint32_t a)
{
  Serial.println("W1 dela");
  uint32_t bb = a;
  fs_ret_t ret2 = fs_store(&fs_config, address_of_page(1), &bb, 1, p1());
  Serial.println("stored w1 " + String(ret2) + " " + String(bb));
  if (ret2 != FS_SUCCESS)
  {
    Serial.println("error storing");
  }
  // while(fs_callback_flag == 1)  {  power_manage(); }
}
void *w2(uint32_t a)
{
  Serial.println("W2 dela");
  uint32_t bb = a;
  fs_ret_t ret2 = fs_store(&fs_config, address_of_page(2), &bb, 1, p1());
  Serial.println("stored w2 " + String(ret2) + " " + String(bb));
  if (ret2 != FS_SUCCESS)
  {
    Serial.println("error storing");
  }
  // while(fs_callback_flag == 1)  {  power_manage(); }
}


void setup()
{
  Serial.begin(115200);
  // set LED pin to output mode
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(PIN_A4, INPUT);
  pinMode(MID_SENSOR, INPUT);
  digitalWrite(PIN_A4, LOW);
  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  /*
    pinMode(KEY1_PIN,INPUT);
    pinMode(PIN_A1,OUTPUT);
    analogWrite(PIN_A1,255);
    */
  // bleBondStore.clearData();
  // blePeripheral.setBondStore(bleBondStore);
  blePeripheral.setAppearance(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);
  // set advertised local name and service UUID
  blePeripheral.setLocalName("LED3");
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());
  // add service and characteristic
  blePeripheral.addAttribute(batteryService);
  blePeripheral.addAttribute(batteryLevelCharacteristic);
  blePeripheral.addAttribute(temperatureService);
  blePeripheral.addAttribute(temperatureCharacteristic);
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(switchCharacteristic);
  blePeripheral.addAttribute(SWITCHDescriptor);
  blePeripheral.addAttribute(CalibrateCharacteristic);
  blePeripheral.addAttribute(CalibrationDescriptor);
  blePeripheral.addAttribute(ForceSensorService);
  blePeripheral.addAttribute(ForceSensorCharacteristic);
  blePeripheral.addAttribute(ForceDescriptor);
  blePeripheral.addAttribute(ForceSensorLimitCharacteristic);
  blePeripheral.addAttribute(ForceSensorLimitCharacteristic2);
  blePeripheral.addAttribute(ManualCharacteristic);
  
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
  ForceSensorLimitCharacteristic.setEventHandler(BLEWritten, ForceSettingWritten);
  ForceSensorLimitCharacteristic2.setEventHandler(BLEWritten, ForceSettingWritten2);
  ManualCharacteristic.setEventHandler(BLEWritten, ManualWritten);
  
  CalibrateCharacteristic.setEventHandler(BLEWritten, CalibrateWritten);
  CalibrateCharacteristic.setEventHandler(BLESubscribed, SubscribedToCalibrate);

  // begin initialization
  blePeripheral.begin();
  Serial.println(F("BLE LED Peripheral"));
  blinkLed(1, 1000);
  state = 0;
  switchCharacteristic.setValue('33');
  CalibrateCharacteristic.setValue('69');
  if (fs_init() != FS_SUCCESS)
  {
    Serial.println("FS error");
  }
  else
  {
    Serial.println("FS Init sucess");
  }
  uint32_t forcelimit = (uint32_t) * (address_of_page(1));
  uint32_t forcelimit2 = (uint32_t) * (address_of_page(2));
  Serial.println(String(forcelimit));
  Serial.println(String(forcelimit2));
  if (forcelimit == 0xFFFFFFFF)
  {
    static uint32_t defaultt = 0x5;
    Serial.println("Erasing a flash page");
    fs_ret_t ret = fs_erase(&fs_config, address_of_page(1), 1, NULL);
    // Serial.println("stored default value"+String(ret));
    if (ret != FS_SUCCESS)
    {
      Serial.println("errrror storing default value");
    }
    ForceSensorLimitCharacteristic.setValue(0x5);
  }
  else
  {
    Serial.println("FS has entry using it");
    ForceSensorLimitCharacteristic.setValue((char)forcelimit);
  }
  if (forcelimit2 == 0xFFFFFFFF)
  {
    static uint32_t defaultt = 0x5;
    Serial.println("Erasing a flash page");
    fs_ret_t ret = fs_erase(&fs_config, address_of_page(2), 1, NULL);
    // Serial.println("stored default value"+String(ret));
    if (ret != FS_SUCCESS)
    {
      Serial.println("errrror storing default value");
    }
    ForceSensorLimitCharacteristic2.setValue(0x5);
  }
  else
  {
    Serial.println("FS has entry using it");
    ForceSensorLimitCharacteristic2.setValue((char)forcelimit2);
  }
  // uint32_t err_code;
  // err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
  // Serial.println(err_code);
  // APP_ERROR_CHECK(err_code);
  /*
  Serial.println("Erasing a flash page at address 0x%X"+ String( (uint32_t)fs_config.p_start_addr));
  Serial.println(String(fs_config.p_start_addr==NULL));
    fs_callback_flag = 1;
    int a =fs_erase(&fs_config, fs_config.p_start_addr, 1,w1());
    Serial.println(String(a));
    if ( a != FS_SUCCESS)
    {
      Serial.println("errrror22");
    }
    */
  // while(fs_callback_flag == 1)  {  power_manage(); }
  /*
  static uint32_t dd = 0xAAAAAAAA;
  fs_ret_t ret = fs_store(&fs_config, fs_config.p_start_addr, &dd, 1,w2());
   Serial.println("stored"+String(ret));
  if (ret != FS_SUCCESS)
  {
        Serial.println("errrror33");
  }
  */
  // w2();
  // w3();
  // enable low power mode and interrupt
  // sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  /*
  uint8_t    patwr;
  uint8_t    patrd;
  uint8_t    patold;
  uint32_t   i;
  uint32_t   pg_size;
  uint32_t   pg_num;
printf("Flashwrite example\n\r");
  patold  = 0;
  pg_size = NRF_FICR->CODEPAGESIZE;
  pg_num  = NRF_FICR->CODESIZE - 1;  // Use last page in flash
  Serial.println(String(pg_size));
    Serial.println(String(pg_num));
      // Start address:
      addr = (uint32_t *)(pg_size * pg_num);
      //flash_page_erase(addr);
        uint8_t    a=(uint8_t)*(addr);
         Serial.println( String(a)+" was read from flash 1"  );
      if ( (uint8_t)  255  == (uint8_t)*(addr) || (uint8_t)  0  == (uint8_t)*(addr))
      {
         Serial.println("force lim not set");
      }
      eraseflash();
            uint8_t t = 1;
            uint8_t tt = 2;
            uint8_t ttt = 3;
      writetoflash(addr);
       */
  // ForceSensorLimitCharacteristic.setValue((uint8_t)*(addr));
  //  Erase page:
  // flash_page_erase(addr);
  // i = 0;
  // for (int i = 0; i < 5; i++)
  //{
  //  uint32_t    a=(uint32_t)*(fs_config.p_start_addr+i);
  // Serial.println( String(a)+" was read from flash"  );
  //}
  /*
  uint8_t l = 2;
  flash_word_write(addr,(uint32_t)l);
   for (int i = 0; i < 9; i++)
  {
     uint8_t    a=(uint8_t)*(addr + i);
     Serial.println( String(a)+" was read from flash"  );
  }
  */
  /*
  do
  {
      printf("Enter char to write to flash\n\r");
      patwr = patold+1;
      if (patold != patwr)
      {
          patold = patwr;
          flash_word_write(addr, (uint32_t)patwr);
          ++addr;
          i += 4;
          Serial.println(String(patwr)+" was write to flash\n\r");
      }
      // Read from flash the last written data and send it back:
      patrd = (uint8_t)*(addr - 1);
      Serial.println(String(patrd) +" was read from flash\n\r\n\r"  );
  }
  while (i < pg_size);
  */
}
void loop()
{
  uint32_t err_code;
  if (true)
  {
    uint32_t evt_id;
    // Pull event from SOC.
    err_code = sd_evt_get(&evt_id);
    if (err_code == NRF_ERROR_NOT_FOUND)
    {
      // no_more_soc_evts = true;
    }
    else if (err_code != NRF_SUCCESS)
    {
      // APP_ERROR_HANDLER(err_code);
    }
    else
    {
      // Call application's SOC event handler.
      sys_evt_dispatch(evt_id);
    }
  }
  unsigned long currentMillis = millis();
  // if (someoneconnected == true)
  //{
  if (currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;
    if (strcmp(WrittenPass, PASSWORD) != 0)
    {
      // blePeripheral.central().disconnect();
      // someoneconnected=false;
    }
  }
  mid_reading = digitalRead(MID_SENSOR) ^ 1;
  if (mid_reading != lastMidState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (mid_reading != mid) {
      mid = mid_reading;
      ForceSensorCharacteristic.setValueLE(mid);
      
    }
  }
   lastMidState = mid_reading;
  //Serial.println("mid is: " + String(mid));







  // press and stop
  if (currentMillis - previousMillis2 > ForceSensingInterval && startsensing != 0)
  {
    Serial.println("mid is: " + String(mid));
   

    
    
      //float sensorvoltage = getForceSensorVoltage();
      if (timeaccum >= LOCKING_TIME && startsensing == 1)
      {
        Serial.println("timeaccum is: " + String(timeaccum));
        Serial.println("Locking Time is : " + String(LOCKING_TIME));
        
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(MOTOR_1, LOW);
         if (calibrationinprogress==0)
        {
           switchCharacteristic.setValue('32');
        }
        //print mid 
        Serial.println("mid is: " + String(mid)+ " and before is: " + String(midBefore));
        if(midBefore == mid){
          //erorr moved to little didnt even change mid sensor
          //go back only using time 
          Serial.println("ERROR moved to little mid didnt change");

           state = 0;
        startsensing = 0;
        goback = 6;
        gobacktime = timeaccum;
        timeaccum = 0;
        //midBeforeBack = mid;
        }
        else{
          //use mid sensor when going back
           state = 1; // LED ON LOCKED
        startsensing = 0;
        goback = 2;
        gobacktime = MaxgObacktime; //timeaccum
        timeaccum = 0;
        midBeforeBack = mid;
          

        }

       
      }
      if (timeaccum >= UNLOCKING_TIME && startsensing == 2)
      {

        Serial.println("timeaccum is: " + String(timeaccum));
        Serial.println("Unlocking Time is : " + String(UNLOCKING_TIME));
        
        digitalWrite(LED_PIN, LOW);
        digitalWrite(MOTOR_2, LOW);
        if (calibrationinprogress==0)
        {
           switchCharacteristic.setValue('33');
        }
        Serial.println("mid is: " + String(mid)+ " and before is: " + String(midBefore));
        if(midBefore == mid){
          //erorr moved to little didnt even change mid sensor
          //go back only using time  
          Serial.println("ERROR moved to little mid didnt change");
           state = 0;
        startsensing = 0;
        goback = 7;
        gobacktime = timeaccum;
        timeaccum = 0;
        //midBeforeBack = mid;

        }else{
           state = 0;
        startsensing = 0;
        goback = 1;
        gobacktime = MaxgObacktime;
        timeaccum = 0;
        midBeforeBack = mid;

        }
        
       
      }
      //ForceSensorCharacteristic.setValueLE(sensorvoltage);
      timeaccum += ForceSensingInterval;
      
      // if (timeaccum >= sensingTime && startsensing != 0)
      // {
      //   digitalWrite(MOTOR_1, LOW);
      //   digitalWrite(MOTOR_2, LOW);
      //    if (calibrationinprogress==0)
      //   {
      //     switchCharacteristic.setValue('34');
      //   }else{
      //     CalibrateCharacteristic.setValue('34');
      //   }
        
      //   if (startsensing == 1)
      //   {
      //     goback = 2;
      //   }
      //   if (startsensing == 2)
      //   {
      //     goback = 1;
      //   }
      //   startsensing = 0;
      //   gobacktime = MaxgObacktime;
      //   timeaccum = 0;
      // }
    
    previousMillis2 = currentMillis;
  }
  //}
  // go back and stop doesent need someone connected to work
  if (currentMillis - previousMillis3 > ForceSensingInterval && goback != 0)
  {
    Serial.println("going back " + String(gobacktime));
    //float sensorvoltage = getForceSensorVoltage();
    if (goback == 1)
    {
      delay(10);
      digitalWrite(MOTOR_1, HIGH);
      goback = 3;
    }
    if (goback == 7)
    {
      //go back when mid sensor is not changed cuz we moved to little
      delay(10);
      digitalWrite(MOTOR_1, HIGH);
      goback = 9;
    }
    if (goback == 2)
    {
      delay(10);
      digitalWrite(MOTOR_2, HIGH);
      goback = 4;
    }
    if (goback == 6)
    {
      delay(10);
      digitalWrite(MOTOR_2, HIGH);
      goback = 8;
    }
    if(goback==5 && (mid==1)){
      //popravek da je senzor sredine vedno odklopljen
      digitalWrite(MOTOR_1, LOW);
      goback=0;
      gobacktime=0;

    }
    if (goback == 3 && (mid!=midBeforeBack))
    {
      CalibrateCharacteristic.setValue('33');
      Serial.println("SUCESS 3 went back changed mid sensor from %d to %d: " + String(midBeforeBack) + " to " + String(mid));
      digitalWrite(MOTOR_1, LOW);
     
      
      gobacktime = 0;
      goback = 0;
    }
    if (goback == 9 && gobacktime<=0)
    {
      CalibrateCharacteristic.setValue('33');
      Serial.println("SUCESS 9 went back ONLY using time: \n");
      digitalWrite(MOTOR_1, LOW);
      gobacktime = 0;
      goback = 0;
    }
    if (goback == 4 && (mid!=midBeforeBack))
    {
     
      CalibrateCharacteristic.setValue('32');
        
      Serial.println("SUCESS 2 went back changed mid sensor from %d to %d: " + String(midBeforeBack) + " to " + String(mid));
      digitalWrite(MOTOR_2, LOW);
       delay(10);
       //popravek da je senzor sredine vedno odklopljen
       digitalWrite(MOTOR_1, HIGH);
      goback = 5;
      

    }
    if (goback == 8 && gobacktime<=0)
    {
     
      CalibrateCharacteristic.setValue('32');
      Serial.println("SUCESS 8 went back ONLY using time: \n");
      digitalWrite(MOTOR_2, LOW);
      goback = 0;
      gobacktime = 0;
      

    }
    
    //ForceSensorCharacteristic.setValueLE(sensorvoltage);
    // sensorvoltage>=sensitivity_voltage_limit &&  &&
    if (goback != 0 && gobacktime <= 0) // TODO FIX THIS
    {
      Serial.println("Error went back didnt hit mid sensor: " + String(gobacktime));
      digitalWrite(MOTOR_1, LOW);
      digitalWrite(MOTOR_2, LOW);
       if (calibrationinprogress==0)
        {
           switchCharacteristic.setValue('34');
        }else{
          CalibrateCharacteristic.setValue('34');
          calibrationinprogress=0;
        }
      goback = 0;
      gobacktime = 0;
    }
    gobacktime -= ForceSensingInterval;
    previousMillis3 = currentMillis;
  }

  if (!someoneconnected && startsensing == 0 && goback == 0)
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
  calibrationinprogress = 0;

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

      if(calibrationinprogress==1){
        calibrationinprogress=0;
        CalibrateCharacteristic.setValue('2');
        //fix if user exits calibration prematurely
      }
      Serial.println(F("Going Forward"));
      
      midBefore=mid;
      Serial.println("mid before: " + String(midBefore));

      check_mid = 1;
      startsensing = 1;
      digitalWrite(MOTOR_1, HIGH);
      // digitalWrite(LED_PIN, HIGH);
      // state=1;
      // delay(3000);
      // digitalWrite(LED_PIN, LOW);
      // switchCharacteristic.setValue(0);
    }
    if (switchCharacteristic.value() == '0')
    {
      Serial.println(F("Going Backward"));
      if(calibrationinprogress==1){
        calibrationinprogress=0;
        CalibrateCharacteristic.setValue('2');
        //fix if user exits calibration prematurely
      }
      //print mid 
      midBefore=mid;
      Serial.println("mid before: " + String(midBefore));
      check_mid = 1;
      startsensing = 2;
      digitalWrite(MOTOR_2, HIGH);
      // digitalWrite(LED_PIN, LOW);
      // state=0;
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
void ManualWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  unsigned char a;
  memcpy(&a, characteristic.value(), characteristic.valueLength());
  if (strcmp(WrittenPass, PASSWORD) != 0)
  {
    central.disconnect();
    return;
  }
  if ((int)a == 1)
  {
    Serial.println(F("Manual Forward"));
    digitalWrite(MOTOR_1, HIGH);
    delay(200);
    digitalWrite(MOTOR_1, LOW);
  }
  if ((int)a == 0)
  {
    Serial.println(F("Manual Backward"));
    digitalWrite(MOTOR_2, HIGH);
    delay(200);
    digitalWrite(MOTOR_2, LOW);
  }
}
void CalibrateWritten(BLECentral &central, BLECharacteristic &characteristic)
{
   Serial.print(F("Characteristic Calibrate event, writen: "));
   
  if (strcmp(WrittenPass, PASSWORD) == 0)
  {
    Serial.println((char)CalibrateCharacteristic.value());
    if (CalibrateCharacteristic.value() == '1')
    {
      Serial.println(F("Calibrate Going Forward"));

      if(!calibrationinprogress){
        LOCKING_TIME = MIN_LOCKING_TIME;
        calibrationinprogress = 1;
        check_mid = 1;
         midBefore=mid;
        startsensing = 1;
        digitalWrite(MOTOR_1, HIGH);

      }
      else if(LOCKING_TIME<MAX_LOCKING_TIME){
        Serial.println(F("Increasing LOCKING_TIME TRYING AGAIN"));
        LOCKING_TIME+=200;
         midBefore=mid;
        check_mid = 1;
        startsensing = 1;
        digitalWrite(MOTOR_1, HIGH);

      }else{
        CalibrateCharacteristic.setValue('35');
        calibrationinprogress = 0;
        return;
      }

    }
    
    if (CalibrateCharacteristic.value() == '0')
    {
      Serial.println(F("Calibrate Going Backward"));

       if(!calibrationinprogress){
        UNLOCKING_TIME = MIN_UNLOCKING_TIME;
        calibrationinprogress = 1;
        check_mid = 1;
         midBefore=mid;
      startsensing = 2;
      digitalWrite(MOTOR_2, HIGH);

      }else if(UNLOCKING_TIME<MAX_UNLOCKING_TIME){
        Serial.println(F("Increasing UNLOCKING_TIME TRYING AGAIN"));
        UNLOCKING_TIME+=200;
        check_mid = 1;
         midBefore=mid;
      startsensing = 2;
      digitalWrite(MOTOR_2, HIGH);

      }else{
        CalibrateCharacteristic.setValue('35');
        calibrationinprogress = 0;
        return;
      }

      
      
    }

    if (CalibrateCharacteristic.value() == '2')
    {

      //OK we are done calibrating
      calibrationinprogress = 0;
      Serial.println(F("Calibrate Done"));
      Serial.println(startsensing);
      //save to flash 
      
     
      
    }
    
  }
  else
  {
    central.disconnect();
  }
}
void ForceSettingWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  unsigned char a;
  memcpy(&a, characteristic.value(), characteristic.valueLength());
  if (0 < (int)a && (int)a <= 10)
  {
    sensitivity_voltage_limit = mapfloat((float)a, 0.0f, 10.0f, 0.0f, 3.0f);
  }
  Serial.println("Sensitivity set to: " + String(sensitivity_voltage_limit));
  uint32_t forcelim = (uint32_t)a;
  Serial.println("Erasing a flash page");
  fs_callback_flag = 1;
  fs_ret_t ret = fs_erase(&fs_config, address_of_page(1), 1, NULL);
  if (ret != FS_SUCCESS)
  {
    Serial.println("error saving char" + String(a));
  }
  w1(forcelim);
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
void ForceSettingWritten2(BLECentral &central, BLECharacteristic &characteristic)
{
  unsigned char a;
  memcpy(&a, characteristic.value(), characteristic.valueLength());
  if (0 < (int)a && (int)a <= 10)
  {
    sensitivity_voltage_limit2 = mapfloat((float)a, 0.0f, 10.0f, 0.0f, 3.0f);
  }
  Serial.println("Sensitivity set to: " + String(sensitivity_voltage_limit2));
  uint32_t forcelim = (uint32_t)a;
  Serial.println("Erasing a flash page");
  fs_callback_flag = 1;
  fs_ret_t ret = fs_erase(&fs_config, address_of_page(2), 1, NULL);
  if (ret != FS_SUCCESS)
  {
    Serial.println("error saving char" + String(a));
  }
  w2(forcelim);
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

void SubscribedToCalibrate(BLECentral &central, BLECharacteristic &characteristic)
{
  Serial.print(F("Notifications enabled for Calibrate "));
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
    /*
     digitalWrite(LED_0, HIGH);
    delay(delaytime);
    digitalWrite(LED_0, LOW);
    delay(delaytime);
    digitalWrite(LED_0, state);
     digitalWrite(LED_1, HIGH);
    delay(delaytime);
    digitalWrite(LED_1, LOW);
    delay(delaytime);
    digitalWrite(LED_1, state);
     digitalWrite(LED_2, HIGH);
    delay(delaytime);
    digitalWrite(LED_2, LOW);
    delay(delaytime);
    digitalWrite(LED_2, state);
    */
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
    // blinkLed(10,800);
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
    // blinkLed(4, 1000);
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
