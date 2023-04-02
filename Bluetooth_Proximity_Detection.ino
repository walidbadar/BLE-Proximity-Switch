
/*
 * 
 * This detects advertising messages of BLE devices and compares it with stored MAC addresses. 
 * If one matches, it sends an MQTT message to swithc something
 */


#include <stdio.h>
#include "BLEDevice.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_err.h"

static BLEAddress *pServerAddress;

#define DEEP_SLEEP_TIME 1
gpio_num_t ignition = GPIO_NUM_27;
gpio_num_t battery = GPIO_NUM_26;

BLEScan* pBLEScan;
BLEClient*  pClient;
bool deviceFound = false;

String knownAddresses[] = {"ff:ff:00:0a:b3:c3"};

unsigned long entry;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());

      bool known = false;
      for (int i = 0; i < (sizeof(knownAddresses) / sizeof(knownAddresses[0])); i++) {
        if (strcmp(pServerAddress->toString().c_str(), knownAddresses[i].c_str()) == 0) known = true;
      }
      if (known) {
        Serial.print("Device found: ");
        Serial.println(advertisedDevice.getRSSI());
        if (advertisedDevice.getRSSI() > -100) deviceFound = true;
        else deviceFound = false;
        Serial.println(pServerAddress->toString().c_str());
        advertisedDevice.getScan()->stop();
      }
    }
}; // MyAdvertisedDeviceCallbacks

void setup() {
  adc_power_off();
  //esp_wifi_stop();
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  //rtc_gpio_init(ignition);
  //rtc_gpio_set_direction(ignition, RTC_GPIO_MODE_OUTPUT_ONLY);
  pinMode(ignition, OUTPUT);
  pinMode(battery, OUTPUT);
  digitalWrite(ignition, LOW);
  digitalWrite(battery, LOW);
  
  BLEDevice::init("");

  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void loop() {  
  Serial.println();
  Serial.println("BLE Scan restarted.....");
  
  deviceFound = false;
  BLEScanResults scanResults = pBLEScan->start(20);
  
  rtc_gpio_hold_dis(ignition);
  rtc_gpio_hold_dis(battery);
  
  if (deviceFound) {
    Serial.println("on");
    digitalWrite(ignition, HIGH);
    digitalWrite(battery, HIGH);
  }
  else {
    Serial.println("off");
    digitalWrite(ignition, LOW);
    digitalWrite(battery, LOW);
  }
  Serial.flush();
  goToDeepSleep();
} // End of loop

void goToDeepSleep()
{
  Serial.println("Going to deep sleep...zZz");
  //WiFi.disconnect(true);
  //WiFi.mode(WIFI_OFF);
  //btStop();

  //esp_wifi_stop();
  //esp_bt_controller_disable();
  rtc_gpio_hold_en(ignition);
  rtc_gpio_hold_en(battery);
  //gpio_deep_sleep_hold_en();
  
  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME * 60L * 1000000L);

  // Go to sleep! Zzzz
  esp_deep_sleep_start();
}
