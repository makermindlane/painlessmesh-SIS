// //************************************************************
// // this is a simple example that uses the painlessMesh library to
// // connect to a node on another network. Please see the WIKI on gitlab
// // for more details
// //
// https://gitlab.com/painlessMesh/painlessMesh/wikis/bridge-between-mesh-and-another-network
// //************************************************************
#include <Arduino.h>
#include <painlessMesh.h>
#include <ESPDash.h>

#define MESH_PREFIX "MeshSSID"
#define MESH_PASSWORD "MeshPass"
#define MESH_PORT 5555

#define STATION_SSID "Muon"
#define STATION_PASSWORD "convolution"

#define HOSTNAME "HTTP_BRIDGE"

//#define _LCL_DEBUG_ // Enable local debugging
//#define _LCL_DEBUG_MESH_ // Enable local debugging of mesh network only

/****************** Variables ******************/
IPAddress myIP(0, 0, 0, 0);

painlessMesh mesh;

AsyncWebServer server(80);

uint16_t onConnectBlinkInterval = 2000;
uint16_t onDisconnectBlinkInterval = 500;

// Attach ESP-DASH to AsyncWebServer
ESPDash dashboard(&server);

// struct to hold card and status of card whether its on or off.
typedef struct {
  Card* card;
  bool status;
} CardDetail;
// Map that associates nodeId to CardDetail*
std::map<uint32_t, CardDetail*> cardDetailMap;

const uint8_t THRESHOLD = 50;
const uint8_t MAX_NO_NODES = 3;

uint8_t thresholdReachedCardCounter = 0;

/****************** Function prototypes ******************/
IPAddress getlocalIP();
long mapReading(long x, long in_min, long in_max, long out_min, long out_max);

void onNewConnection(uint32_t nodeId);
void onDroppedConnection(uint32_t nodeId);
void onReceive(uint32_t id, String& msg);
void onChangedConnections();

void updateDash(const std::list<uint32_t>& nodeList, std::map<uint32_t, CardDetail*>& cardDetailMap);
void addNode(const std::list<uint32_t>& nodeList, std::map<uint32_t, CardDetail*>& cardDetailMap);
void removeNode(const std::list<uint32_t>& nodeList, std::map<uint32_t, CardDetail*>& cardDetailMap);
void addCard(uint32_t nodeId);
void removeCard(uint32_t nodeId);
void updateCard(uint32_t nodeId, String& msg);
void turnOnStarter();
void turnOffStarter();

// Task callback function prototypes
void tcb_blinkLED();
void tcb_printLocalIP();
void tcb_updateDashboard();
void tcb_updateDash();

// Scheduler
Scheduler ts;

// Tasks
Task tBlinkLED(500, TASK_FOREVER, &tcb_blinkLED, &ts);
Task tPrintLocalIP(TASK_MINUTE, TASK_FOREVER, &tcb_printLocalIP, &ts);
Task tUpdateDashboard(TASK_SECOND * 1, TASK_FOREVER, &tcb_updateDashboard, &ts);
Task tupdateDash(TASK_SECOND * 5 , TASK_FOREVER, &tcb_updateDash, &ts);

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // set before init() so that ykou can see startup messages
  mesh.setDebugMsgTypes(ERROR | STARTUP);

  // Channel set to 6. Make sure to use the same channel for your mesh and for
  // you other network (STATION_SSID)
  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 6);

  // Station ssid and password
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);

  mesh.setRoot(true);
  mesh.setContainsRoot(true);

  // Event callbacks
  mesh.onNewConnection(&onNewConnection); 
  mesh.onReceive(&onReceive);
  mesh.onDroppedConnection(&onDroppedConnection);
  mesh.onChangedConnections(&onChangedConnections);

  server.begin();

  tBlinkLED.enable();
  tPrintLocalIP.enable();
  tUpdateDashboard.enable();
  tupdateDash.enable();
}

void loop() {
  mesh.update();
  ts.execute();
}

/**
 * Function to be called on new connection
 */
void onNewConnection(uint32_t nodeId) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\nonNewConnection(): ENTER");
#endif

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.printf("New connection: %u\n", nodeId);
#endif
  
  addCard(nodeId);

  if (tBlinkLED.getInterval() != onConnectBlinkInterval) {
    tBlinkLED.setInterval(onConnectBlinkInterval);
  }

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("onNewConnection(): EXIT\n");
#endif 
}

/**
 * Function to be called when a node disconnects.
 */
void onDroppedConnection(uint32_t nodeId) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\nonDroppedConnection(): ENTER");
#endif 

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.printf("Connection dropped: %u\n", nodeId);
#endif

  removeCard(nodeId);
  
  if ((tBlinkLED.getInterval() != onDisconnectBlinkInterval) && (cardDetailMap.size() == 0)) {
    tBlinkLED.setInterval(onDisconnectBlinkInterval);
  }
  
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("onDroppedConnection(): EXIT\n");
#endif 
}

/**
 * Function to be called when receiving from other node
 */
void onReceive(uint32_t id, String& msg) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\nonReceive(): ENTER");
#endif 

  updateCard(id, msg);

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("onReceive(): EXIT\n");
#endif 
}

/**
 * Function to be called when connection layout is changed
 */
void onChangedConnections() {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\nonChangedConnections(): ENTER");
#endif

  updateDash(mesh.getNodeList(), cardDetailMap);

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("onChangedConnections(): EXIT\n");
#endif
}

/****************** Task Callback functions ******************/

// Printing the local IP address every minute.
void tcb_printLocalIP() {
  #ifdef _LCL_DEBUG_
    Serial.println("\nprintLocalIP(): ENTER");
  #endif

  if (myIP != IPAddress(mesh.getStationIP())) {
    myIP = getlocalIP();
    Serial.printf("My IP is: %s\n", myIP.toString().c_str());
  }

  #ifdef _LCL_DEBUG_
    Serial.println("tcb_printLocalIP(): EXIT\n");
  #endif
}

// Blinking LED.
void tcb_blinkLED() { 
  #ifdef _LCL_DEBUG_
    Serial.println("\nblinkLED(): ENTER");
  #endif

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 

  #ifdef _LCL_DEBUG_
    Serial.println("tcb_blinkLED(): EXIT\n");
  #endif
}

// Send updates to dashboard every 5 seconds i.e., update the 
// UI every 5 second.
void tcb_updateDashboard() {
  dashboard.sendUpdates();
}

// Update no. of nodes every 10 second. (Basically polling, checking to 
// see if any node has disconnected or connected)
void tcb_updateDash() {
  updateDash(mesh.getNodeList(), cardDetailMap);
}

/****************** Local/helper functions ******************/
void updateDash(const std::list<uint32_t>& nodeList, std::map<uint32_t, CardDetail*>& cardDetailMap) {

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\nupdateDash(): ENTER");
#endif

  auto nodeListSize = nodeList.size();
  auto mapSize = cardDetailMap.size();

  // Everything is upto date.
  if (nodeListSize == mapSize) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("nodeList = cardDetailMap");
#endif
    return;
  }
  // Add card.
  else if (nodeListSize > mapSize) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("nodeList > cardDetailMap");
#endif

    addNode(nodeList, cardDetailMap);
  }
  // Remove card.
  else if (nodeListSize < mapSize) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("nodeList < cardDetailMap");
#endif

    removeNode(nodeList, cardDetailMap);
  }

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("updateDash(): EXIT\n");
#endif
}

// Add node to cardDetailMap.
void addNode(const std::list<uint32_t>& nodeList, std::map<uint32_t, CardDetail*>& cardDetailMap) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\naddCard(): ENTER");
#endif

  for (const auto& node : nodeList) {
    if (cardDetailMap.find(node) == cardDetailMap.end()) {
      addCard(node);
    }
  }

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("addNode(): EXIT\n");
#endif
}

// Remove node from cardDetailMap.
void removeNode(const std::list<uint32_t>& nodeList, std::map<uint32_t, CardDetail*>& cardDetailMap) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\nremoveCard(): ENTER");
#endif

  std::list<uint32_t>::const_iterator listFindItr;
  for (auto& m : cardDetailMap) {
    listFindItr = std::find(nodeList.begin(), nodeList.end(), m.first);
    if (listFindItr == nodeList.end()) {
      removeCard(m.first);
    }
  }

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("removeNode(): EXIT\n");
#endif
}

// Add Card UI to the dashboard.
void addCard(uint32_t nodeId) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\naddCard(): ENTER");
#endif
  // If the Card with node id equals to nodeId does not 
  // exists in the cardDetailMap, then add Card.
  if (cardDetailMap.find(nodeId) == cardDetailMap.end()) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.print("addCard(): node (");
  Serial.print(nodeId);
  Serial.println(") does not exist in cardDetailMap, now adding...");
#endif
    cardDetailMap[nodeId] = new CardDetail;
    cardDetailMap[nodeId]->card = new Card(&dashboard, HUMIDITY_CARD, String(nodeId).c_str(), "%", 0, 100);
    cardDetailMap[nodeId]->status = true;
  }

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("addCard(): EXIT\n");
#endif
}

// Remove Card UI from dashboard.
void removeCard(uint32_t nodeId) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\nremoveCard(): ENTER");
#endif
  // If the Card with node id equals to nodeId exists
  // in the cardDetailMap, then remove it.
  if (cardDetailMap.find(nodeId) != cardDetailMap.end()) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.print("removeCard(): node ("); 
  Serial.print(nodeId);
  Serial.println(") exists in the cardDetailMap, now removing...");
#endif
    dashboard.remove(cardDetailMap[nodeId]->card);
    cardDetailMap[nodeId]->card = nullptr;
    delete cardDetailMap[nodeId];
    cardDetailMap.erase(nodeId);
  }

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("removeCard(): EXIT\n");
#endif
}

// Update a particular Card UI associated with node = nodeId
// with value msg.
void updateCard(uint32_t nodeId, String& msg) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("\nupdateCard(): ENTER");
#endif

  if (cardDetailMap.find(nodeId) != cardDetailMap.end()) {
#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.print("updateCard(): node (");
  Serial.print(nodeId);
  Serial.println(") found");
#endif

    uint16_t sensorIn = msg.toInt();
    sensorIn = mapReading(sensorIn, 0, 1023, 0, 100);
    sensorIn = constrain(sensorIn, 0, 100);
    cardDetailMap[nodeId]->card->update(sensorIn);
    bool status = cardDetailMap[nodeId]->status;

    if ((sensorIn > THRESHOLD) && (status == true)) {
      Serial.printf("\nThreshold reached: %ui\n", nodeId);
      cardDetailMap[nodeId]->status = false;
      thresholdReachedCardCounter++;

      Serial.printf("thresholdReachedCardCounter: %ui", thresholdReachedCardCounter);

      if (thresholdReachedCardCounter >= MAX_NO_NODES) {
        turnOffStarter();
      }
    }
    else if ((sensorIn < THRESHOLD) && (status == false)) {
      Serial.printf("\nResuming operation: %ui\n", nodeId);
      cardDetailMap[nodeId]->status = true;
      thresholdReachedCardCounter--;

      Serial.printf("thresholdReachedCardCounter: %ui", thresholdReachedCardCounter);

      if (thresholdReachedCardCounter == 0) {
        turnOnStarter();
      }
    }
  }

#if defined(_LCL_DEBUG_) || defined(_LCL_DEBUG_MESH_)
  Serial.println("updateCard(): EXIT\n");
#endif
}

void turnOnStarter() {
  if (thresholdReachedCardCounter == 0) {
    // code to turn on the relay that controls the starter power on.
    Serial.println("Starter is turned on");
  }
}

void turnOffStarter() {
  if (thresholdReachedCardCounter >= MAX_NO_NODES) {
    // code to turn of the relay which controls the starter power on.
    Serial.println("Starter is turned off");
  }
}

IPAddress getlocalIP() { return IPAddress(mesh.getStationIP()); }

long mapReading(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
