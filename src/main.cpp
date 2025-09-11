#include <Arduino.h>
#include "LoraMesher.h"
#include "display.h"
#include <SPI.h>
#include "camera.h" 
#include <Wire.h>
#include "ws2812.h"    
#include "crc32.h" 
#include "ft/ft_protocol.h"
#include "ft/ft_common.h"
#include "ft/ft_dispatch.h"
#include "ft/ft_gateway.h"
#include "ft/ft_camera.h"

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

#define ENABLE_LEGACY_SEND 0

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

    char buf[24];
    for (int i = 0; i < rtSize; i++) {
        RouteNode* r = (*routingTableList)[i];
        NetworkNode n = r->networkNode;
        snprintf(buf, sizeof(buf), "|%X(%d)->%X", n.address, n.metric, r->via);
        Screen.changeRoutingText(buf, i);
    }
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
    vTaskDelay(pdMS_TO_TICKS(1000)); 
  }
}

void processReceivedPackets(void* parameter) {
  uint32_t lastLog = 0;
  uint32_t packetsReceived = 0;
  
  for (;;) {
    if (millis() - lastLog > 10000) {
      lastLog = millis();
      #if NODE_IS_GATEWAY
      Serial.printf("[GW-RX] Receive task alive, total packets: %lu, queue size: %d\n", 
                    packetsReceived, radio.getReceivedQueueSize());
      #else  
      Serial.printf("[CAM-RX] Receive task alive, total packets: %lu, queue size: %d\n",
                    packetsReceived, radio.getReceivedQueueSize());
      #endif
    }
    while (radio.getReceivedQueueSize() > 0) {
      #if NODE_IS_GATEWAY
      const char* prefix = "[GW-RX]";
      #else
      const char* prefix = "[CAM-RX]";
      #endif
      
      Serial.printf("%s Processing queue, size: %d\n", 
                    prefix, radio.getReceivedQueueSize());
      
      AppPacket<uint8_t>* pkt = radio.getNextAppPacket<uint8_t>();
      if (!pkt) {
        Serial.printf("%s getNextAppPacket returned NULL\n", prefix);
        break; 
      }
      
      packetsReceived++;
      
      const uint8_t* p = (const uint8_t*)pkt->payload;
      size_t n = pkt->payloadSize;
      
      Serial.printf("%s Received packet #%lu: src=%04X, len=%d, type=%u\n", 
                    prefix, packetsReceived, pkt->src, (int)n, n > 0 ? p[0] : 255);
      
      if (n > 0) {
        Serial.printf("%s First bytes:", prefix);
        for (int i = 0; i < min((int)n, 8); i++) {
          Serial.printf(" %02X", p[i]);
        }
        Serial.println();
      }
      
      if (ft_handle_rx(pkt->src, p, n)) {
        Serial.printf("%s Packet handled by FT protocol\n", prefix);
      } else {
        Serial.printf("%s Unknown/legacy msg from %04X\n", prefix, pkt->src);
      }
      
      radio.deletePacket(pkt);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
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

#if ENABLE_LEGACY_SEND
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
#endif

void setupLoraMesher() {
    LoraMesher::LoraMesherConfig config = LoraMesher::LoraMesherConfig();

    config.loraCs        = LORA_NSS;
    config.loraRst       = LORA_RST;
    config.loraIrq       = LORA_BUSY; 
    config.loraIo1       = LORA_DIO1;
    config.module        = LoraMesher::LoraModules::SX1280_MOD;
    config.freq          = 2400.0;
    config.bw            = 203.125;
    config.sf            = 7;
    config.cr            = 5;
    config.power         = 13;
    config.preambleLength = 32;
    config.syncWord      = 0x12;

    radio.begin(config);
    createReceiveMessages();
    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
    #if NODE_IS_GATEWAY
        radio.addGatewayRole();
        Serial.println("Role: GATEWAY");
    #else
        Serial.println("Role: CAMERA");
    #endif
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
  ws2812SetColor(3);                 

  pinMode(MOTION_SENSOR_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), pirISR, RISING);

  setupLoraMesher();

  Wire.begin(OLED_SDA, OLED_SCL, 400000);
  Screen.initDisplay();
  Screen.drawDisplay();

  setUpCamera();
  #if NODE_IS_GATEWAY
    ft_gw_setup();
  #endif
  #if !NODE_IS_GATEWAY && ENABLE_LEGACY_SEND
    createSendMessages();
  #endif

  printAddressDisplay();
  xTaskCreate(routeUIUpdateTask, "RouteUI", 2048, nullptr, 1, nullptr);
  Screen.drawDisplay(); 

  ws2812SetColor(2);            
  Serial.println("Setup complete.");
}

void loop() {
  if (pirRising) {
    pirRising = false;
    uint32_t now = millis();
    if (now - lastCaptureMs > CAPTURE_LOCKOUT_MS) {
      Serial.println("PIR rising edge → capture");
      ws2812SetColor(3);              
      captureImage();                
      ws2812SetColor(2);            
      lastCaptureMs = now;
    }
  }
#if !NODE_IS_GATEWAY
  static bool meta_kickoff_done = false;
  static uint8_t gw_seen_stable = 0;

  if (!meta_kickoff_done) {
    RouteNode* gw = radio.getBestNodeWithRole(ROLE_GATEWAY);
    if (gw) {
      if (++gw_seen_stable >= 3) {
        uint32_t fileId = 0, start = 0;
        if (ft_cam_handshakeMeta("image_0.jpg", fileId, start)) {
          Serial.printf("[CAM] start offset %u for id=%08X\n",
                        (unsigned)start, (unsigned)fileId);
          ft_cam_sendFile("image_0.jpg", fileId, start); 
          meta_kickoff_done = true; 
        } else {
          Serial.println("[CAM] META postponed/failed; will retry next loop");
        }
      }
    } else {
      gw_seen_stable = 0;
      static uint32_t t0 = 0;
      if (millis() - t0 > 3000) { t0 = millis(); Serial.println("[CAM] waiting for GW route..."); }
    }
  }
#endif

  vTaskDelay(pdMS_TO_TICKS(5));
}