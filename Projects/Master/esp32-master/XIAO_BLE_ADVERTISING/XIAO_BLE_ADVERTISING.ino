#include <Arduino.h>
#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"

#define BEACON_ID 3
#define ADV_SEC 2 //Default 2 seconds

//#define NO_BUTTONS // comment this out if you are using buttons

char beacon_name[11] = "3LOGY-B"; // must be left outside the setup function because it must be
// saved inside PROGMEM instead of RAM, will be destroyed if left in RAM and leaves scope of setup function

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

float acx, acy, acz, gyx, gyy, gyz;
bool fall = false;
bool trigger1 = false;
bool trigger2 = false;
bool trigger3 = false;
uint8_t free_fall_count = 0;
uint8_t no_move_count = 0;
uint8_t trigger1_count = 0;
uint8_t trigger2_count = 0;
uint8_t trigger3_count = 0;

float amplitude_buffer[50];
float *amplitude_buffer_start = amplitude_buffer;
float *amplitude_buffer_end = amplitude_buffer;

bool SOS_flag = 0;
bool battery_low_flag = 0;
bool fall_flag = 0;

unsigned long prev_millis = 0;

unsigned long SOS_button_prev_millis = 0;
uint8_t SOS_button_count = 0;

unsigned long power_button_prev_millis = 0;
uint8_t power_button_count = 0;

unsigned long imu_prev_millis = 0;

#define SOS_button 9
#define power_button 10

inline void cycle_pointers(){
    if (amplitude_buffer_end == amplitude_buffer + 50){
        amplitude_buffer_end = amplitude_buffer;
    }
    if (amplitude_buffer_end == amplitude_buffer_start){
        amplitude_buffer_start++;
    }
    if (amplitude_buffer_start == amplitude_buffer + 50){
        amplitude_buffer_start = amplitude_buffer;
    }
}

float* increment_amplitude_buffer_pointer(float *p){
    if (++p == amplitude_buffer + 50){
        return amplitude_buffer;
    }
    else {
        return p;
    }
}

void read_accel(){
    acx = myIMU.readFloatAccelX();
    acy = myIMU.readFloatAccelY();
    acz = myIMU.readFloatAccelZ();
}

void setup()
{
    // setting the registers for the bluetooth transmit power
    uint32_t *TXPOWER = (uint32_t *)0x4000150C;
    *TXPOWER &= 0x0;
    *TXPOWER |= 0x08;

    uint32_t *MODE = (uint32_t *)0x40001510;
    *MODE &= 0x0;
    *MODE |= 0x05;
    
    pinMode(SOS_button, INPUT);
    pinMode(power_button, INPUT);
    pinMode(P0_14, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(P0_14, LOW);
    
    Serial.begin(115200);

    if (!BLE.begin())
    {
        Serial.println("starting BLE failed!");
        while (1)
            ;
    }

    Serial.println("Starting BLE ok!");

    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }
    
    if (BEACON_ID / 100) {
        beacon_name[7] = BEACON_ID / 100 + 48;
        beacon_name[8] = BEACON_ID / 10 % 10 + 48;
        beacon_name[9] = BEACON_ID % 10 + 48;
    }
    else if (BEACON_ID / 10) {
        beacon_name[7] = '0';
        beacon_name[8] = BEACON_ID / 10 + 48;
        beacon_name[9] = BEACON_ID % 10 + 48;
    }
    else {
        beacon_name[7] = '0';
        beacon_name[8] = '0';
        beacon_name[9] = BEACON_ID + 48;
    }
    beacon_name[10] = '\0';
    BLE.setLocalName(beacon_name);
//    BLE.setLocalName("3LOGY-B001");
}

void loop()
{
#ifndef NO_BUTTONS
    while(true){
        if (millis() - power_button_prev_millis > 50){
            bool reading = digitalRead(power_button);
//            Serial.print("Power Button Reading OFF: ");
//            Serial.println(reading);
            if (reading && power_button_count <= 61){
                power_button_count++;
            }
            
            if (power_button_count >= 2 && reading == 0){
                if (power_button_count < 60) {
                    break;
                }
                power_button_count = 0;
            }
            power_button_prev_millis = millis();
        }
    }
#endif
    // add some startup sequence here
    digitalWrite(LED_BUILTIN, 0);

    while(true){
        if (millis() - imu_prev_millis >= 100){
            read_accel();
            *amplitude_buffer_end++ = pow(acx*acx+acy*acy+acz*acz,0.5);
            cycle_pointers();
        //    Serial.print("Amplitude buffer: [");
            
        //    float *prt = amplitude_buffer_start;
        ////    print the buffer to serial monitor
        //    while (prt != amplitude_buffer_end){
        //        Serial.print(*prt++);
        //        Serial.print(", ");
        //        if (prt == amplitude_buffer + 50){
        //            prt = amplitude_buffer;
        //        }
        //    }
        //    Serial.println("]");
            float *p_read = amplitude_buffer_start;
            if (*p_read <= 0.15){
                free_fall_count++;
                while (p_read != amplitude_buffer_end){
                    p_read = increment_amplitude_buffer_pointer(p_read);
                    if (*p_read <= 0.15){
                        free_fall_count++;
                    }
                    else if (*p_read <= 1.2 && *p_read >= 0.8){
                        no_move_count++;
                    }
                }
                if (free_fall_count > 0 && no_move_count > 25){
                    Serial.println("FALL DETECTED!");
                    fall_flag = 1;
                }
                free_fall_count = 0;
                no_move_count = 0;
            }
            imu_prev_millis = millis();
        }
        
        if (millis()-prev_millis >= ADV_SEC * 1000) {
            BLE.stopAdvertise();
            // Battery Level Reading
            unsigned int adcCount = analogRead(PIN_VBAT);
            double adcVoltage = (adcCount * 3.3) / 1024;
            double vBat = adcVoltage*151/51; // Voltage divider from Vbat to ADC
            uint8_t battery_send = vBat * 100 - 300;
            
            Serial.print("adcCount = ");
            Serial.print(adcCount);
            Serial.print(" adcVoltage = ");
            Serial.print(adcVoltage,3);
            Serial.print(" vBat = ");
            Serial.print(vBat,3);
            Serial.print(" battery_send = ");
            Serial.print(battery_send);
            Serial.println("");
            
            BLEAdvertisingData advData;
            advData.setFlags(BLEFlagsBREDRNotSupported | BLEFlagsGeneralDiscoverable);
            Serial.println(myIMU.readTempC(), 4);
            int16_t temperature_16 = myIMU.readTempC() * 100;
            
            Serial.println(SOS_flag);
        
            unsigned char send_data[6] = {
                (unsigned char) ~(BEACON_ID >> 8),
                (unsigned char) ~(BEACON_ID & 0xFF),
                (unsigned char)(temperature_16 & 0xFF),
                (unsigned char)(temperature_16 >> 8),
                (unsigned char) 0 | SOS_flag << 4 | fall_flag,
                (unsigned char) battery_send
            };
            
            advData.setAdvertisedServiceData(0x2ACA, send_data, sizeof(send_data));
            BLE.setAdvertisingData(advData);
            BLE.advertise();
        
            prev_millis = millis();
        }
    
//        if (millis() - SOS_button_prev_millis > 50){
//            bool reading = !digitalRead(SOS_button);
//            Serial.print("SOS Button Reading: ");
//            Serial.println(reading);
//            if (reading){
//                SOS_button_count++;
//            }
//            else {
//                SOS_button_count = 0;
//            }
//            if (SOS_button_count == 100){
//                SOS_flag = 1;
//            }
//            SOS_button_prev_millis = millis();
//        }
    #ifndef NO_BUTTONS
        if (millis() - SOS_button_prev_millis > 50){
            int reading = digitalRead(SOS_button);
    
            if (reading && SOS_button_count <= 101){
                SOS_button_count++;
            }
            
            if (SOS_button_count >= 3 && reading == 0){
                if (SOS_button_count < 60) {
                    fall_flag = 0;
                }
                SOS_button_count = 0;
            }
            if (SOS_button_count == 100){
                SOS_flag = 1;
            }
            SOS_button_prev_millis = millis();
        }

        if (millis() - power_button_prev_millis > 50){
            int reading = digitalRead(power_button);
//            Serial.print("Power Button Reading ON: ");
//            Serial.println(reading);
            
            if (reading && power_button_count <= 61){
                power_button_count++;
            }
            
            if (power_button_count >= 3 && reading == 0){
                power_button_count = 0;
            }
            if (power_button_count == 60){
                break;
            }
            power_button_prev_millis = millis();
        }
    #endif
    }
    
    // insert shutdown sequence
    BLE.stopAdvertise();
    SOS_flag = 0;
    fall_flag = 0;
    digitalWrite(LED_BUILTIN, 1);

    // alternate SOS_button code
//    bool SOS_button_pressed = digitalRead(SOS_button);
//    if (SOS_button_pressed){
//        if (millis() - SOS_button_prev_millis > 50){
//            SOS_button_count++;
//            if (SOS_button_count == 100){
//                SOS_flag = 1;
//            }
//            SOS_button_prev_millis = millis();
//        }
//    }
//    else {
//        SOS_button_count = 0;
//    }



    
//    if (millis() - power_button_prev_millis > 50){
//        int reading = digitalRead(power_button);
//
//        if (reading && power_button_count <= 61){
//            power_button_count++;
//        }
//        
//        if (power_button_count >= 2 && reading == 0){
//            if (power_button_count < 60) {
//                // set some flag for short press
//            }
//            power_button_count = 0;
//        }
//        if (power_button_count == 60){
//            power_button_count = 0; // if break right after this, need to reset the count
//            // set some flag for long press
//        }
//    }
    
}
