#include <Arduino.h>
#include "LoraMesher.h"
#include "display.h"
#include <SPI.h>
#include "camera.h" 
#include <Wire.h>
#include "ws2812.h"     

#define OLED_SDA 3
#define OLED_SCL 2


#define LORA_SCK   45  
#define LORA_MISO  47  
#define LORA_MOSI  21  
#define LORA_NSS   14  
#define LORA_RST    1  
#define LORA_DIO1  41  
#define LORA_BUSY  42  

#define MOTION_SENSOR_PIN 35

#ifndef NODE_IS_GATEWAY
#define NODE_IS_GATEWAY 1  
#endif

bool motionDetected = false;

LoraMesher& radio = LoraMesher::getInstance();

uint32_t dataCounter = 0;
struct dataPacket {
    uint32_t counter;           // 4 bytes
    uint8_t data[196];          // 196 bytes to make total 200 bytes
};

dataPacket* helloPacket = new dataPacket;

static void pixelFlash(uint16_t flashes, uint16_t delayMs, int color) {
    for (uint16_t i = 0; i < flashes; i++) {
        ws2812SetColor(color);
        vTaskDelay(delayMs / portTICK_PERIOD_MS);
        ws2812SetColor(0);
        vTaskDelay(delayMs / portTICK_PERIOD_MS);
    }
}

static inline void pixelOK()      { ws2812SetColor(2); } // green
static inline void pixelBusy()    { ws2812SetColor(3); } // blue
static inline void pixelError()   { ws2812SetColor(1); } // red
static inline void pixelOff()     { ws2812SetColor(0); } // off


void printRoutingTableToDisplay() {
    LM_LinkedList<RouteNode>* routingTableList = radio.routingTableListCopy();
    routingTableList->setInUse();

    const int rtSize = radio.routingTableSize();
    Screen.changeSizeRouting(rtSize);

    // render all routes
    char buf[24];
    for (int i = 0; i < rtSize; i++) {
        RouteNode* r = (*routingTableList)[i];
        NetworkNode n = r->networkNode;
        snprintf(buf, sizeof(buf), "|%X(%d)->%X", n.address, n.metric, r->via);
        Screen.changeRoutingText(buf, i);
    }

    // find GW and also remember the "best" route as a fallback
    RouteNode* gw = radio.getBestNodeWithRole(ROLE_GATEWAY);
    RouteNode* best = nullptr;
    int bestMetric = 0x7FFFFFFF;
    for (int i = 0; i < rtSize; i++) {
        RouteNode* r = (*routingTableList)[i];
        if (r->networkNode.metric < bestMetric) {
            bestMetric = r->networkNode.metric;
            best = r;
        }
    }

    routingTableList->releaseInUse();
    delete routingTableList;

    // place the summary line in slot 0 or 1 depending on table size
    int sumSlot = (rtSize > 0) ? 1 : 0;

    if (gw) {
        bool direct = (gw->via == gw->networkNode.address);
        if (direct) {
            Screen.changeRoutingText("GW direct", sumSlot);
        } else {
            char gwBuf[24];
            snprintf(gwBuf, sizeof(gwBuf), "GW via %X", gw->via);
            Screen.changeRoutingText(gwBuf, sumSlot);
        }
    } else if (best) {
        // fallback so the NODE shows something meaningful until GW role propagates
        char fb[24];
        snprintf(fb, sizeof(fb), "Best %X via %X", best->networkNode.address, best->via);
        Screen.changeRoutingText(fb, sumSlot);
    } else {
        Screen.changeRoutingText("No routes", sumSlot);
    }

    Screen.drawDisplay();
}

void routeUIUpdateTask(void*){
  for(;;){
    printRoutingTableToDisplay();
    vTaskDelay(pdMS_TO_TICKS(1000));  // update every 1s
  }
}

void processReceivedPackets(void* parameter) {
    for (;;) {
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        if (radio.getReceivedQueueSize() > 0) {
            pixelFlash(1, 100, 3);  
            Serial.println("\n*** PACKETS IN QUEUE! ***");
            Serial.printf("Queue size: %d\n", radio.getReceivedQueueSize());

            while (radio.getReceivedQueueSize() > 0) {
                Serial.println("Processing packet...");
                AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();

                if (packet == NULL) {
                    Serial.println("ERROR: Received NULL packet!");
                    pixelFlash(2, 80, 1); 
                    continue;
                }

                uint32_t recvNum = packet->payload->counter;
                Serial.printf("Packet received from %X\n", packet->src);
                Serial.printf("  Counter: %d\n", recvNum);
                Serial.printf("  Payload size: %d bytes\n", packet->payloadSize);

                bool dataOk = true;
                for (int i = 0; i < 10; i++) {
                    if (packet->payload->data[i] != ((recvNum + i) & 0xFF)) {
                        dataOk = false;
                        break;
                    }
                }
                Serial.printf("  Data pattern: %s\n", dataOk ? "OK" : "MISMATCH");
                if (!dataOk) pixelFlash(2, 60, 1); else pixelFlash(1, 60, 2);

                Screen.changeLineTwo("Recv " + String(recvNum));
                Screen.changeLineThree(String(packet->src, HEX) + "->" + String(recvNum));
                printRoutingTableToDisplay();

                pixelFlash(3, 100, 3);

                radio.deletePacket(packet);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        static uint32_t lastPrint = 0;
        if (++lastPrint >= 10) {
            lastPrint = 0;
            Serial.printf("Queue check - Size: %d\n", radio.getReceivedQueueSize());
        }
    }
}

TaskHandle_t receiveLoRaMessage_Handle = NULL;

void createReceiveMessages() {
    int res = xTaskCreate(
        processReceivedPackets,
        "Receive App Task",
        4096,
        (void*) 1,
        2,
        &receiveLoRaMessage_Handle);
    if (res != pdPASS) {
        Serial.printf("Error: Receive App Task creation gave error: %d\n", res);
        pixelFlash(4, 100, 1); 
    }
}

void printAddressDisplay() {
    char addrStr[15];
    snprintf(addrStr, 15, "Id: %X\r\n", radio.getLocalAddress());
    Screen.changeLineOne(String(addrStr));
}

void sendLoRaMessage(void*) {
    int dataTablePosition = 0;

    for (;;) {
        Serial.println("\n=== Send Task Running ===");
        Serial.printf("Routing table size: %d\n", radio.routingTableSize());

        if (radio.routingTableSize() == 0) {
            Serial.println("No routes available, waiting...");
            pixelBusy(); 
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        RouteNode* gwNode = radio.getBestNodeWithRole(ROLE_GATEWAY);
        if (!gwNode) {
            Serial.println("No gateway in routing table yet");

            LM_LinkedList<RouteNode>* routingTableList = radio.routingTableListCopy();
            routingTableList->setInUse();
            for (int i = 0; i < radio.routingTableSize(); i++) {
                RouteNode* rNode = (*routingTableList)[i];
                Serial.printf("Route %d: Address=%X, Role=%d\n",
                    i, rNode->networkNode.address, rNode->networkNode.role);
            }
            routingTableList->releaseInUse();
            delete routingTableList;

            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        uint16_t gwAddr = gwNode->networkNode.address;
        Serial.printf("Sending to gateway at address: %X\n", gwAddr);

        helloPacket->counter = dataCounter;
        for (int i = 0; i < (int)sizeof(helloPacket->data); i++) {
            helloPacket->data[i] = (dataCounter + i) & 0xFF;
            if (i % 50 == 0) vTaskDelay(1);
        }

        pixelBusy(); 
        radio.sendReliable(gwAddr, helloPacket, sizeof(dataPacket));
        pixelOK();   

        printRoutingTableToDisplay();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void createSendMessages() {
    TaskHandle_t sendLoRaMessage_Handle = NULL;
    BaseType_t res = xTaskCreate(
        sendLoRaMessage,
        "Send LoRa Message routine",
        4098,
        (void*) 1,
        1,
        &sendLoRaMessage_Handle);
    if (res != pdPASS) {
        Serial.printf("Error: Send LoRa Message task creation gave error: %d\n", res);
        vTaskDelete(sendLoRaMessage_Handle);
        pixelFlash(4, 100, 1);
    }
}

void setupLoraMesher() {
    LoraMesher::LoraMesherConfig config = LoraMesher::LoraMesherConfig();

    config.loraCs        = LORA_NSS;
    config.loraRst       = LORA_RST;
    config.loraIrq       = LORA_BUSY; 
    config.loraIo1       = LORA_DIO1;
    config.module        = LoraMesher::LoraModules::SX1280_MOD;
    config.freq          = 2400.0;
    config.bw            = 203.125;
    config.sf            = 10;
    config.cr            = 5;
    config.power         = 13;
    config.preambleLength = 32;
    config.syncWord      = 0x12;

    radio.begin(config);
    createReceiveMessages();
    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
    if (NODE_IS_GATEWAY) {
      radio.addGatewayRole();
      Serial.println("Role: GATEWAY");
    } else {
      Serial.println("Role: CAMERA");
    }
    radio.start();
    
    Serial.println("Lora initialized");
}

const uint32_t CAPTURE_LOCKOUT_MS = 1500;
volatile bool pirRising = false;
uint32_t lastCaptureMs = 0;

void IRAM_ATTR pirISR() {
  pirRising = true; 
}

void setup() {
  delay(300);
  Serial.begin(115200);
  delay(2000);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  Serial.println("Setup started...");
  Serial.println("\nBOOT: hello from ESP32-S3");

  ws2812Init();
  ws2812SetColor(3);                  // blue = booting

  pinMode(MOTION_SENSOR_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), pirISR, RISING);

  setupLoraMesher();

  Wire.begin(OLED_SDA, OLED_SCL, 400000);
  Screen.initDisplay();
  Screen.drawDisplay();

  setUpCamera();
  if (!NODE_IS_GATEWAY) {
    createSendMessages();   // enable periodic send on camera only
  }

  printAddressDisplay();
  xTaskCreate(routeUIUpdateTask, "RouteUI", 2048, nullptr, 1, nullptr);
  Screen.drawDisplay(); 

  ws2812SetColor(2);                  // green = ready
  Serial.println("Setup complete.");
}

void loop() {
  // PIR → capture throttle
  if (pirRising) {
    pirRising = false;
    uint32_t now = millis();
    if (now - lastCaptureMs > CAPTURE_LOCKOUT_MS) {
      Serial.println("PIR rising edge → capture");
      ws2812SetColor(3);              // blue while capturing
      captureImage();                 // keep this non-blocking as much as possible
      ws2812SetColor(2);              // green when done
      lastCaptureMs = now;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(5));
}