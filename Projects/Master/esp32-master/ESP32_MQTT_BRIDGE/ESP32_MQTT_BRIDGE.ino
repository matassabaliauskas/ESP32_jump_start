#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "esp_wifi.h"

#define   RX  3  //Box 3
#define   TX  1  //Box 1

#define   UART_BUFFER_SIZE 4096

#define   STATION_SSID      "TP-Link_3logytech"
#define   STATION_PASSWORD  "3logytech1928"

#define   HOSTNAME        "MQTT_Bridge"

// Prototypes
void sendMessage(); 
void mqttCallback(char* topic, byte* payload, unsigned int length);
void connectToWifi();
IPAddress getlocalIP();

char uart_buffer[UART_BUFFER_SIZE]; // UART buffer to receive data from connected ESP32 scanning BLE
char *uart_buffer_end = uart_buffer;
uint16_t uart_buffer_length = 0;

unsigned long last_millis = 0;

bool msg_write = 0;
bool ready_to_send = 0;

inline void uart_buffer_write(char c){
  if (uart_buffer_end != uart_buffer + UART_BUFFER_SIZE){
    *uart_buffer_end++ = c;
    uart_buffer_length++;
//    Serial.print('.');
  }
//  uart_buffer_length++;
}

inline void uart_buffer_clear(){
  memset(uart_buffer, 0, UART_BUFFER_SIZE);
  uart_buffer_end = uart_buffer;
  uart_buffer_length = 0;
}

void uart_buffer_print(){
    Serial.println(String(uart_buffer));
}

//Scheduler userScheduler; // to control your personal task

IPAddress myIP(0,0,0,0);
IPAddress mqttBroker(192,168,1,121);

WiFiClient wifiClient;
PubSubClient mqttClient(mqttBroker, 1883, mqttCallback, wifiClient);

void send_mqtt(){
  char topic[12];
//  char *p = uart_buffer+1;
//  uint8_t count = 0;
//  while (*p != 4){ // if using node_id
//    topic[count++] = *p++;
//    if (count > 11){
//        return;
//    }
//  }
  int idLen=0;
  while(uart_buffer[idLen] != '{') {
    idLen++;
  }
//  Serial.print("length: "); Serial.println(idLen);

//{"B1":{"RSSI":-74,"Temp":"32.91","SOS":0,"Fall":0,"Battery":61}}

  memcpy(topic, uart_buffer, idLen); // if using node_id , old code, some node_id can be 10 digits long, could have some even longer...
  Serial.print("TOPIC");Serial.println(topic);
  
  char msg[uart_buffer_length];
  memcpy(msg, uart_buffer + idLen , uart_buffer_length);
  Serial.print("MESSAGE");Serial.println(msg);
  if(!mqttClient.publish(topic,msg)){ 
    Serial.println("Failed");
//    while(1);
  }
  else Serial.println("Success");
//  mqttClient.publish(topic,msg);
}

//Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval

void connectToWifi(){
  Serial.print("Connecting to: ");
  WiFi.begin(STATION_SSID,STATION_PASSWORD);
  Serial.println(STATION_SSID);
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected.");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000, SERIAL_8N1, RX, TX);
//  Serial2.setRxBufferSize(32768);

  connectToWifi();
  if (ESP_OK == esp_wifi_set_max_tx_power(80)) Serial.println("POWER SET");

//  pinMode(LED, OUTPUT);

//  userScheduler.addTask( taskSendMessage );
//  taskSendMessage.enable();
}

void loop() {
  mqttClient.loop();

  if(myIP != getlocalIP()){
    myIP = getlocalIP();
    Serial.println("My IP is " + myIP.toString());
    
    if (mqttClient.connect("MQTT BRIDGE NODE")){
        mqttClient.setBufferSize(4096);
        Serial.println("mqtt connected");
//      mqttClient.publish("painlessMesh/from/gateway", "Ready!");
//      mqttClient.subscribe("painlessMesh/to/#");
    }
  }
  while (Serial2.available()){
//        Serial.print(char(Serial2.read()));
    char c = (char) Serial2.read();
    uint8_t msg_write_prev = msg_write;
    if (c == 2){
        if (msg_write == 0){
            msg_write = 1;
            uart_buffer_clear();
            continue;
        }
        else {
            uart_buffer_clear(); // if the uart_buffer data is not complete and a new set of data comes in, starting with 2 again, then the incomplete data will be deleted.
        }
    }
    else if (c == 3){
        msg_write = 0;
    }
    if (msg_write == 1){
        uart_buffer_write(c);
    }
    else if (msg_write_prev == 1){
      ready_to_send = 1;
      break;
    }
  }

  if (ready_to_send){
    uart_buffer_print();
    if (!mqttClient.state()) mqttClient.connect("MQTT BRIDGE NODE");
    send_mqtt();
    ready_to_send = 0;
    uart_buffer_clear();
  }

  if (millis() - last_millis > 3000){
//    uart_buffer_print();
    last_millis = millis();
  }
}

//void sendMessage() {
//  String msg = "Hello from node ";
//  msg += mesh.getNodeId();
//  msg += " myFreeMemory: " + String(ESP.getFreeHeap());
//  mesh.sendBroadcast(msg);
//
//  if (calc_delay) {
//    SimpleList<uint32_t>::iterator node = nodes.begin();
//    while (node != nodes.end()) {
//      mesh.startDelayMeas(*node);
//      node++;
//    }
//    calc_delay = false;
//  }
//
//  Serial.printf("Sending message: %s\n", msg.c_str());
//  
//  taskSendMessage.setInterval( random(TASK_SECOND * 1, TASK_SECOND * 5));  // between 1 and 5 seconds
//}


void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
    Serial.println("mqttCallback!");
//  char* cleanPayload = (char*)malloc(length+1);
//  memcpy(cleanPayload, payload, length);
//  cleanPayload[length] = '\0';
//  String msg = String(cleanPayload);
//  free(cleanPayload);
//
//  String targetStr = String(topic).substring(16);
//
//  if(targetStr == "gateway")
//  {
//    if(msg == "getNodes")
//    {
//      auto nodes = mesh.getNodeList(true);
//      String str;
//      for (auto &&id : nodes)
//        str += String(id) + String(" ");
//      mqttClient.publish("painlessMesh/from/gateway", str.c_str());
//    }
//  }
//  else if(targetStr == "broadcast") 
//  {
//    mesh.sendBroadcast(msg);
//  }
//  else
//  {
//    uint32_t target = strtoul(targetStr.c_str(), NULL, 10);
//    if(mesh.isConnected(target))
//    {
//      mesh.sendSingle(target, msg);
//    }
//    else
//    {
//      mqttClient.publish("painlessMesh/from/gateway", "Client not connected!");
//    }
//  }
}

IPAddress getlocalIP() {
  return IPAddress(WiFi.localIP());
}
