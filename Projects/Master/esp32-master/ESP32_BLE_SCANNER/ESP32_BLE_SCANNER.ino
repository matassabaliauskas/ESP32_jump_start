#include <ArduinoJson.h>
#include <NimBLEDevice.h>

#define RX 3 //Box 3
#define TX 1 //Box 1

unsigned long beacon_millis = 0;

int scanTime = 3; //Default 3 seconds
BLEScan* pBLEScan;

DynamicJsonDocument doc(1024);

char str1[6] = "3LOGY";

typedef struct beacon_data {
    uint16_t b_id;
    int16_t b_rssi;
    float b_temperature;
    uint8_t b_flags;
    uint8_t b_battery;
} beacon_data;

float process_temp(const char *p){
    int16_t temperature = p[3] << 8 | p[2];
    return (float)temperature / 100;
}

uint8_t process_battery_percent(const char *p){
  return (uint8_t)(p[5] / 1.06);
}

uint16_t beacon_id_check(const char *p){
    uint16_t beacon_id = ((uint8_t)~p[0]) << 8 | (uint8_t)(~p[1]);
    return beacon_id;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice* advertisedDevice) {
//      Serial.printf("Advertised Device: %s \n", advertisedDevice.getName().c_str());
        char device_name[10];
        memcpy(device_name,advertisedDevice->getName().c_str(),10);
        if (!strncmp(str1,device_name,5)){
            const char* p_data = advertisedDevice->getServiceData().c_str();
            beacon_data discovered_beacon;
            discovered_beacon.b_id = 100 * (*(device_name+7)-48) + 10 * (*(device_name+8)-48) + (*(device_name+9)-48);
            if (discovered_beacon.b_id != beacon_id_check(p_data)){
                return;
            }
            discovered_beacon.b_rssi = (int16_t)advertisedDevice->getRSSI();
            discovered_beacon.b_temperature = process_temp(p_data);
            discovered_beacon.b_flags = (uint8_t)p_data[4];
            discovered_beacon.b_battery = process_battery_percent(p_data);
//            Serial.printf("ID: %u, RSSI: %d, Temp: ", discovered_beacon.b_id, discovered_beacon.b_rssi);
//            Serial.print(process_temp(p_data_copy));
//            Serial.print(process_temp(p_data_copy));
//            Serial.printf(" SOS_flag: %d Fall: %d Battery Level: ", (p_data[2]&16)>>4, p_data[2]&1);

            JsonObject new_beacon = doc.createNestedObject(String("B") + String(discovered_beacon.b_id)); // can remove the "B", it's not necessary
            new_beacon["RSSI"] = discovered_beacon.b_rssi;
            new_beacon["Temp"] = String(discovered_beacon.b_temperature, 2);
            new_beacon["SOS"] = (discovered_beacon.b_flags & 16) >> 4;
            new_beacon["Fall"] = discovered_beacon.b_flags & 1;
            new_beacon["Battery"] = discovered_beacon.b_battery;
        }
    }
};

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX, TX);

  Serial.println("HELLO WORLD");

  BLEDevice::setScanFilterMode(CONFIG_BTDM_SCAN_DUPL_TYPE_DATA_DEVICE);
  BLEDevice::setScanDuplicateCacheSize(500);
  BLEDevice::init("");

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9); 
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN ,ESP_PWR_LVL_P9);
  
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
  pBLEScan->setDuplicateFilter(1);
}

void loop() {
    //start bluetooth scanning
    doc.clear();
//    Serial.println("Scan started!");
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
//    Serial.println("Scan done!");
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory

    if (!doc.isNull()){
        Serial2.write(2);
        serializeJson(doc,Serial2); // max length for 1 beacon is 68 {"B1":{"RSSI":-110,"Temp":"28.00","SOS":0,"Fall":0,"Battery":100}}
        Serial2.write(3);
        beacon_millis = millis();
    }
    else if (millis()-beacon_millis >= 3000){
        Serial2.write(2);
        Serial2.write("{\"B0\":{\"RSSI\":0,\"Temp\":0,\"SOS\":0,\"Fall\":0,\"Battery\":0}}");
        Serial.println("{\"B0\":{\"RSSI\":0,\"Temp\":0,\"SOS\":0,\"Fall\":0,\"Battery\":0}}");
        Serial2.write(3);
        beacon_millis = millis();
    }
    serializeJson(doc,Serial);
    Serial.println();
//    serializeJsonPretty(doc,Serial);
    
}
