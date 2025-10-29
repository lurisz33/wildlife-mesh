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
#include <SD_MMC.h>
#include "sd_read_write.h"
enum : uint8_t { MSG_PING = 0xA1, MSG_PONG = 0xA2, MSG_BIG = 0xA3 };

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

void printDataPacket(AppPacket<dataPacket>* packet) {
    if (!packet) return;
    Serial.printf("[DATA] Packet from %04X: counter=%u, size=%u\n",
                  packet->src, 
                  packet->payload[0].counter,
                  packet->getPayloadLength());
    // Print first few bytes of data
    Serial.print("[DATA] First 16 bytes: ");
    for (int i = 0; i < 16 && i < sizeof(packet->payload[0].data); i++) {
        Serial.printf("%02X ", packet->payload[0].data[i]);
    }
    Serial.println();
}

void processReceivedPackets(void*) {
    for (;;) {
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        pixelFlash(1, 100, 3);  // Blue flash on receive
        
        while (radio.getReceivedQueueSize() > 0) {
            auto* rawPacket = radio.getNextAppPacket<uint8_t>();
            if (!rawPacket) continue;
            
            uint16_t src = rawPacket->src;
            size_t dataLen = rawPacket->getPayloadLength();
            uint8_t* data = rawPacket->payload;
            
            Serial.printf("[RX] Packet from %04X, len=%u, type=%02X\n", 
                          src, (unsigned)dataLen, data[0]);
            
            // First try to handle with ft_handle_rx
            bool handled = ft_handle_rx(src, data, dataLen);
            
            if (!handled) {
                // Handle non-FT packets (like dataPacket)
                if (dataLen == sizeof(dataPacket)) {
                    // Convert to dataPacket
                    radio.deletePacket(rawPacket);
                    AppPacket<dataPacket>* dataPacketPtr = radio.getNextAppPacket<dataPacket>();
                    if (dataPacketPtr) {
                        uint32_t recvNum = dataPacketPtr->payload[0].counter;
                        Serial.printf("[DATA] Received packet #%u from %04X\n", 
                                      recvNum, dataPacketPtr->src);
                        Screen.changeLineTwo("Data: " + String(recvNum));
                        radio.deletePacket(dataPacketPtr);
                    }
                    continue;
                } else {
                    Serial.printf("[RX] Unhandled packet type %02X from %04X\n", 
                                  data[0], src);
                }
            }
            
            printRoutingTableToDisplay();
            radio.deletePacket(rawPacket);
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
    Serial.printf("Local addr=%04X\n", radio.getLocalAddress());
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
  //sdmmcInit();

  printAddressDisplay();
  xTaskCreate(routeUIUpdateTask, "RouteUI", 2048, nullptr, 1, nullptr);
  Screen.drawDisplay(); 

  Serial.printf("sizeof(FtMeta)=%u sizeof(FtAck)=%u\n",
              (unsigned)sizeof(FtMeta), (unsigned)sizeof(FtAck));

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
  {
    static bool fileSent = false;
    static uint32_t lastSendAttempt = 0;
    const uint32_t SEND_RETRY_INTERVAL = 10000; // 10 seconds between attempts
    
    // Get gateway node
    RouteNode* gw = radio.getBestNodeWithRole(ROLE_GATEWAY);
    if (gw && !fileSent) {
      uint32_t now = millis();
      if (now - lastSendAttempt > SEND_RETRY_INTERVAL) {
        uint16_t gwAddr = gw->networkNode.address;
        const char* fname = "image_0.jpg";
        uint32_t fileId = 0x3CA1C8D9;  // You might want to generate this dynamically
        uint32_t startOffset = 0;
        
        Serial.printf("[CAM] Sending file %s to gateway %04X\n", fname, gwAddr);
        
        // Send the file directly
        bool success = ft_cam_sendFile(fname, fileId, startOffset, gwAddr);
        
        if (success) {
          Serial.println("[CAM] File sent successfully");
          fileSent = true;  // Mark as sent to avoid resending
        } else {
          Serial.println("[CAM] File send failed, will retry in 10 seconds");
        }
        
        lastSendAttempt = now;
      }
    }
    
    if (!gw) {
      // No gateway available yet
      if (millis() % 10000 < 10) {  // Print every 10 seconds
        Serial.println("[CAM] Waiting for gateway in routing table...");
      }
    }
  }
#endif


  vTaskDelay(pdMS_TO_TICKS(5));
}