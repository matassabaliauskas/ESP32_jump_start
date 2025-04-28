//************************************************************
// this is a simple example that uses the easyMesh library
//
// 1. blinks led once for every node on the mesh
// 2. blink cycle repeats every BLINK_PERIOD
// 3. sends a silly message to every node on the mesh at a random time between 1 and 5 seconds
// 4. prints anything it receives to Serial.print
//
//
//************************************************************
#include <painlessMesh.h>
#include "esp_wifi.h"

#define RX 3 //Box 3
#define TX 1 //Box 1

#define UART_BUFFER_SIZE 1024

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
void checkRoot();
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback(); 
void nodeTimeAdjustedCallback(int32_t offset); 
void delayReceivedCallback(uint32_t from, int32_t delay);

char uart_buffer[UART_BUFFER_SIZE]; // UART buffer to receive data from connected ESP32 scanning BLE
char *uart_buffer_end = uart_buffer;

bool msg_write = 0;
bool ready_to_send = 0;

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;
char rootidChar[12];
uint64_t rootidInt = 0;

bool calc_delay = false;
SimpleList<uint32_t> nodes;

//void sendMessage() ; // Prototype
//Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval

// Task to blink the number of nodes
Task blinkNoNodes;
bool onFlag = false;

inline void uart_buffer_write(char c){
  if (uart_buffer_end != uart_buffer + UART_BUFFER_SIZE){
    *uart_buffer_end++ = c;
  }
}

inline void uart_buffer_clear(){
  memset(uart_buffer, 0, UART_BUFFER_SIZE);
  uart_buffer_end = uart_buffer;
}

inline void send_gateway_data(){
//  mesh.sendSingle(rootidInt, String(uart_buffer));
    String uart_buffer_string = String(uart_buffer);
    mesh.sendBroadcast(uart_buffer_string.substring(1,uart_buffer_string.length()));
    Serial.println(uart_buffer_string.substring(1,uart_buffer_string.length()));
}

uint64_t atoll(char* str)
{
    // Initialize result
    int res = 0;
    for (int i = 0; str[i] != '\0'; ++i)
        res = res * 10 + str[i] - '0';
    return res;
}
  
char newNumChar[1];
uint8_t prevNum, newNumInt=10;

void check_connected_cb(){
    Serial.print("prevNum: "); Serial.println(prevNum);
    Serial.print("newNum: "); Serial.println(newNumInt);
    if (rootidInt && (!mesh.isConnected(rootidInt) || (prevNum==newNumInt))){
        Serial.println("RESTARTING!");
        Serial.println(rootidInt);
        Serial.println(mesh.isConnected(rootidInt));
        ESP.restart();
    }
    else prevNum = newNumInt;
}

Task task_check_connected(TASK_SECOND * 5, TASK_FOREVER, &check_connected_cb);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX, TX);
  Serial.println("HELLO");

  pinMode(LED, OUTPUT);

  mesh.setDebugMsgTypes(ERROR | DEBUG);  // set before init() so that you can see error messages

  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, MESH_CHANNEL, 0);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);

  if (ESP_OK == esp_wifi_set_max_tx_power(80)) Serial.println("POWER SET");

//  mesh.setContainsRoot(true);

//  userScheduler.addTask( taskSendMessage );
//  taskSendMessage.enable();

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
  userScheduler.addTask(task_check_connected);
  blinkNoNodes.enable();
  task_check_connected.enable();

//  randomSeed(analogRead(A0));
}

void loop() {
  mesh.update();
  digitalWrite(LED, !onFlag);
  
  while (Serial2.available()){
//        Serial.print(char(Serial2.read()));
    char c = (char) Serial2.read();
    uint8_t msg_write_prev = msg_write;
    if (c == 2){
        if (msg_write == 0){
            msg_write = 1;
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
    send_gateway_data();
    ready_to_send = 0;
    uart_buffer_clear();
  }
}

//void sendMessage() {
//  String msg = "Hello from node ";
//  msg += mesh.getNodeId();
//  msg += " myFreeMemory: " + String(ESP.getFreeHeap());
//  mesh.sendSingle(rootidInt, msg);
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
  Serial.printf("startHere: Received from %u msg= %s\n", from, msg.c_str());
  if (!strncmp(msg.c_str(),"ROOT NODE ID",12)){
      strncpy(rootidChar, &msg.c_str()[14],msg.length()-14);
      rootidInt = atoll(rootidChar);
  }

  // check random int here from root node, if same as prev, restart esp, if not carry on
  if (!strncmp(msg.c_str(), "ROOT CHECK", 10)){
    strncpy(newNumChar, &msg.c_str()[10], msg.length()-10);
    newNumInt = atoll(newNumChar);
  }

}

void newConnectionCallback(uint32_t nodeId) {
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
  Serial.printf("--> startHere: New Connection, %s\n", mesh.subConnectionJson(true).c_str());
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");
  Serial.printf("%s\n", mesh.subConnectionJson(true).c_str());

//  SimpleList<uint32_t>::iterator node = nodes.begin();
//  while (node != nodes.end()) {
//    Serial.printf(" %u", *node);
//    node++;
//  }
//  Serial.println();
  calc_delay = true;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}

void droppedConnectionCallback(uint32_t nodeId){
  
}
