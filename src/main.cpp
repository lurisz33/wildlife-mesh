#include <Arduino.h>
#include "LoraMesher.h"
#include "display.h"
#include <SPI.h>
#include "camera.h"

#define BOARD_LED   2
#define LED_ON      LOW
#define LED_OFF     HIGH

#define LORA_SCK   33
#define LORA_MISO  34
#define LORA_MOSI  43
#define LORA_NSS   36
#define LORA_RST   37
#define LORA_DIO1  41
#define LORA_BUSY  42

#define MOTION_SENSOR_PIN 35
bool motionDetected = false;


LoraMesher& radio = LoraMesher::getInstance();

uint32_t dataCounter = 0;
struct dataPacket {
    uint32_t counter;           // 4 bytes
    uint8_t data[196];         // 196 bytes to make total 200 bytes
};

dataPacket* helloPacket = new dataPacket;

void led_Flash(uint16_t flashes, uint16_t delaymS) {
    uint16_t index;
    for (index = 1; index <= flashes; index++) {
        digitalWrite(BOARD_LED, LED_ON);
        vTaskDelay(delaymS / portTICK_PERIOD_MS);
        digitalWrite(BOARD_LED, LED_OFF);
        vTaskDelay(delaymS / portTICK_PERIOD_MS);
    }
}

void printRoutingTableToDisplay() {
    // Make a copy of the routing table (must call releaseInUse and delete later)
    LM_LinkedList<RouteNode>* routingTableList = radio.routingTableListCopy();
    routingTableList->setInUse();

    // Resize display to fit current table size (max 2 lines assumed)
    Screen.changeSizeRouting(radio.routingTableSize());

    // Print each route entry
    char buf[16];
    for (int i = 0; i < radio.routingTableSize(); i++) {
        RouteNode* rNode = (*routingTableList)[i];
        NetworkNode node = rNode->networkNode;
        // Format: |<addr>(<metric>)-><via>
        snprintf(buf, sizeof(buf), "|%X(%d)->%X", node.address, node.metric, rNode->via);
        Screen.changeRoutingText(buf, i);
    }

    // Release and delete the copy
    routingTableList->releaseInUse();
    delete routingTableList;

    // Now display the gateway path on the next line slot
    RouteNode* gw = radio.getBestNodeWithRole(ROLE_GATEWAY);
    int gwSlot = radio.routingTableSize();
    // Clamp to available slots (0 or 1)
    if (gwSlot > 1) gwSlot = 1;

    if (gw) {
        bool direct = (gw->via == gw->networkNode.address);
        // Direct vs via
        if (direct) {
            Screen.changeRoutingText("GW direct", gwSlot);
        } else {
            char gwBuf[16];
            snprintf(gwBuf, sizeof(gwBuf), "GW via %X", gw->via);
            Screen.changeRoutingText(gwBuf, gwSlot);
        }
    } else {
        Screen.changeRoutingText("No GW route", gwSlot);
    }
}



void processReceivedPackets(void* parameter) {
    
    for (;;) {
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        // Check queue directly instead of waiting for notification
        if (radio.getReceivedQueueSize() > 0) {
            led_Flash(1, 100);
            Serial.println("\n*** PACKETS IN QUEUE! ***");
            Serial.printf("Queue size: %d\n", radio.getReceivedQueueSize());
            
            while (radio.getReceivedQueueSize() > 0) {
                Serial.println("Processing packet...");
                AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();
                
                if (packet == NULL) {
                    Serial.println("ERROR: Received NULL packet!");
                    continue;
                }
                
                uint32_t recvNum = packet->payload->counter;
                //Serial.printf("Packet received from %X with counter: %d\n", packet->src, recvNum);
                Serial.printf("Packet received from %X\n", packet->src);
                Serial.printf("  Counter: %d\n", recvNum);
                Serial.printf("  Payload size: %d bytes\n", packet->payloadSize);
                
                // Verify data pattern (first 10 bytes)
                bool dataOk = true;
                for (int i = 0; i < 10; i++) {
                    if (packet->payload->data[i] != ((recvNum + i) & 0xFF)) {
                        dataOk = false;
                        break;
                    }
                }
                Serial.printf("  Data pattern: %s\n", dataOk ? "OK" : "MISMATCH");
                
                Screen.changeLineTwo("Recv " + String(recvNum));
                Screen.changeLineThree(String(packet->src, HEX) + "->" + String(recvNum));
                printRoutingTableToDisplay();
                
                // Flash LED 3 times for received packet
                led_Flash(3, 100);
                
                radio.deletePacket(packet);
            }
        } else {
            // No packets, just wait a bit
            vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
        }
        
        // Print status every 10 seconds
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

        // Wait until there's at least 1 route (do not send blindly)
        if (radio.routingTableSize() == 0) {
            Serial.println("No routes available, waiting...");
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

        // Fill message payload
        helloPacket->counter = dataCounter;
        for (int i = 0; i < sizeof(helloPacket->data); i++) {
            helloPacket->data[i] = (dataCounter + i) & 0xFF;

            if (i % 50 == 0) {
                vTaskDelay(1);
            }
        }

        radio.sendReliable(gwAddr, helloPacket, sizeof(dataPacket));
    
        printRoutingTableToDisplay();

        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Wait longer for reliability
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
    radio.start();
    //radio.addGatewayRole();
    Serial.println("Lora initialized");
}

void setup() {
    Serial.begin(115200);
    pinMode(BOARD_LED, OUTPUT); 
    pinMode(MOTION_SENSOR_PIN, INPUT);
    gpio_install_isr_service(0);
    //Screen.initDisplay();
    Serial.println("Board Init");     
    //setupLoraMesher();
    setUpCamera();
    //printAddressDisplay();
    //createSendMessages();
}

void loop() {
  int motionSensorState = digitalRead(MOTION_SENSOR_PIN);
  if (motionSensorState == HIGH && !motionDetected) {
    motionDetected = true;
    Serial.println("Motion detected!");
    captureImage();
  } else if (motionSensorState == LOW) {
    motionDetected = false;
  }
}

