#include <painlessMesh.h>
#include "esp_wifi.h"

#define   RX  3  //Box 3
#define   TX  1  //Box 1

// some gpio pin that is connected to an LED...
// on my rig, this is 5, change to the right number of your LED.
#define   LED 2

#define   BLINK_PERIOD    3000 // milliseconds until cycle repeat
#define   BLINK_DURATION  100  // milliseconds LED is on for

#define   MESH_SSID       "trinet"
#define   MESH_PASSWORD   "something"
#define   MESH_PORT       5555
#define   MESH_CHANNEL    7

// Prototypes
void sendMessage(); 
void checkLeaf();
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback(); 
void nodeTimeAdjustedCallback(int32_t offset); 
void delayReceivedCallback(uint32_t from, int32_t delay);

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;

bool calc_delay = false;
SimpleList<uint32_t> nodes;

// Task to blink the number of nodes
Task blinkNoNodes;
bool onFlag = false;

int ranNum, prevNum;

void checkLeaf(){
  prevNum = ranNum;
  while (prevNum == ranNum) ranNum = random(10);
//  ranNum = 1;
  String msg = "ROOT CHECK";
  msg += (String)ranNum;
  mesh.sendBroadcast(msg);
}

Task checkLeafConnected( TASK_SECOND * 5, TASK_FOREVER, &checkLeaf);

//Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000, SERIAL_8N1, RX, TX);

  pinMode(LED, OUTPUT);

  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION | DEBUG);  // set before init() so that you can see error messages

  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, MESH_CHANNEL, 0);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);

  if (ESP_OK == esp_wifi_set_max_tx_power(80)) Serial.println("POWER SET");

  blinkNoNodes.set(BLINK_PERIOD, (mesh.getNodeList().size() + 1) * 2, []() {
      // If on, switch off, else switch on
      if (onFlag)
        onFlag = false;
      else
        onFlag = true;
      blinkNoNodes.delay(BLINK_DURATION);

      if (blinkNoNodes.isLastIteration()) {
        // Finished blinking. Reset task for next run 
        // blink number of nodes (including this node) times
        blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
        // Calculate delay based on current mesh time and BLINK_PERIOD
        // This results in blinks between nodes being synced
        blinkNoNodes.enableDelayed(BLINK_PERIOD - 
            (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
      }
  });
  userScheduler.addTask(blinkNoNodes);
  blinkNoNodes.enable();

  userScheduler.addTask(checkLeafConnected);
  checkLeafConnected.enable();

//  userScheduler.addTask( taskSendMessage );
//  taskSendMessage.enable();
}

void loop() {
  mesh.update();

  digitalWrite(LED, !onFlag);

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


void receivedCallback(uint32_t from, String & msg) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
//  String topic = "painlessMesh/from/" + String(from);
//  mqttClient.publish(topic.c_str(), msg.c_str());
  Serial2.write(2);
  Serial2.printf("%u", from);  //302302861
  Serial2.print(msg.c_str());  //{"B001":{"RSSI":-110,"Temp":"28.00","SOS":0,"Fall":0,"Battery":100}}
  Serial2.write(3);
  // FULL msg = 302302861{"B001":{"RSSI":-110,"Temp":"28.00","SOS":0,"Fall":0,"Battery":100}}
}

void newConnectionCallback(uint32_t nodeId) {
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  String msg = "ROOT NODE ID: ";
  msg += mesh.getNodeId();
  mesh.sendSingle(nodeId, msg);
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
  Serial.printf("--> startHere: New Connection, %s\n", mesh.subConnectionJson(true).c_str());
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);

  String msg = "ROOT NODE ID: ";
  msg += mesh.getNodeId();
  mesh.sendBroadcast(msg);
  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("--> startHere: Changed Connection, %s\n", mesh.subConnectionJson(true).c_str());

  if (nodes.size()==0) ESP.restart(); //Restart if no mesh nodes connected
//  Serial.printf("Connection list:");
//
//  SimpleList<uint32_t>::iterator node = nodes.begin();
//  while (node != nodes.end()) {
//    Serial.printf(" %u", *node);
//    node++;
//  }
//  Serial.println();
//  calc_delay = true;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}

//void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
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
//}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}
