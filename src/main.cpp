#include <Arduino.h>
#include "LoraMesher.h"
#include "display.h"
#include <SPI.h>
#include <SD_MMC.h> 
#include <esp_camera.h>

// Define device role
#define IS_GATEWAY false  // true for gateway/receiver, false for camera/sender

// LED Pin (Note: GPIO 4 is used for both LED and camera SIOD)
#define BOARD_LED   2     // Using GPIO 2 for LED since GPIO 4 is camera
#define LED_ON      LOW
#define LED_OFF     HIGH

// Photo trigger pins - using safe pins
#define PHOTO_PIN1    21   // Safe GPIO for input
#define PHOTO_PIN2    22   // Safe GPIO for input

// Camera Pin definitions
#define CAM_PIN_PWDN 38
#define CAM_PIN_RESET -1   //software reset will be performed
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_D0 11
#define CAM_PIN_D1 9
#define CAM_PIN_D2 8
#define CAM_PIN_D3 10
#define CAM_PIN_D4 12
#define CAM_PIN_D5 18
#define CAM_PIN_D6 17
#define CAM_PIN_D7 16

// LoRa Pin definitions (no conflicts with camera)
#define LORA_SCK   14    // SPI Clock
#define LORA_MISO  27    // SPI MISO
#define LORA_MOSI  26    // SPI MOSI
#define LORA_NSS   25    // Chip Select (CS)
#define LORA_RST   33    // Reset
#define LORA_DIO1  32    // DIO1/IRQ
#define LORA_BUSY  35    // Busy pin (input only)

LoraMesher& radio = LoraMesher::getInstance();

uint32_t dataCounter = 0;
uint32_t photoCounter = 0;

struct dataPacket {
    uint32_t counter;           // 4 bytes
    uint8_t data[196];         // 196 bytes to make total 200 bytes
};

struct imagePacket {
    uint32_t packetNumber;
    uint32_t totalPackets;
    uint32_t imageId;
    uint16_t dataSize;
    uint8_t imageData[190];
};

dataPacket* helloPacket = new dataPacket;

/**
 * @brief Flash the LED
 */
void led_Flash(uint16_t flashes, uint16_t delaymS) {
    for (uint16_t i = 1; i <= flashes; i++) {
        digitalWrite(BOARD_LED, LED_ON);
        vTaskDelay(delaymS / portTICK_PERIOD_MS);
        digitalWrite(BOARD_LED, LED_OFF);
        vTaskDelay(delaymS / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Initialize camera
 */
void initCamera() {
    camera_config_t config;
    memset(&config, 0, sizeof(config));
    
    // Pin configuration
    config.pin_pwdn = CAM_PIN_PWDN;
    config.pin_reset = CAM_PIN_RESET;
    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_sscb_sda = CAM_PIN_SIOD;
    config.pin_sscb_scl = CAM_PIN_SIOC;
    
    config.pin_d7 = CAM_PIN_D7;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;
    config.pin_pclk = CAM_PIN_PCLK;
    
    // Camera configuration
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;     // 320x240 - smaller size to save memory
    config.jpeg_quality = 10;               // 0-63, lower means higher quality
    config.fb_count = 1;                    // Single frame buffer to save memory
    
    // ESP32-S3 specific settings
    config.fb_location = CAMERA_FB_IN_DRAM; // Use DRAM instead of PSRAM
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    // Initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return;
    }

    Serial.println("Camera initialized successfully");
    
    // Drop down frame size for higher initial frame rate
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_framesize(s, FRAMESIZE_QVGA);
    }
}

/**
 * @brief Initialize SD card
 */
void initSD() {
    // For ESP32-S3, we need to configure SD_MMC pins
    // Standard 1-bit SD mode pins for ESP32-S3
    SD_MMC.setPins(
        39,  // CMD
        40,  // CLK  
        41   // D0
    );
    
    // Initialize SD card in 1-bit mode
    if (!SD_MMC.begin("/sdcard", true)) {  // true = 1-bit mode
        Serial.println("SD Card Mount Failed - continuing without SD");
        return;
    }
    
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }
    
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));
}

/**
 * @brief Take photo and save to SD card
 */
bool takeAndSavePhoto(String& filename) {
    Serial.println(">> Taking photo!");
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return false;
    }
    
    filename = "/photo_" + String(photoCounter++) + ".jpg";
    File f = SD_MMC.open(filename.c_str(), FILE_WRITE);
    if (!f) {
        Serial.println("Failed to open file for writing");
        esp_camera_fb_return(fb);
        return false;
    }
    
    f.write(fb->buf, fb->len);
    f.close();
    Serial.printf("Photo saved: %s (%u bytes)\n", filename.c_str(), fb->len);
    
    esp_camera_fb_return(fb);
    return true;
}

/**
 * @brief Send image over LoRa
 */
void sendImageOverLoRa(const String& filename) {
    File file = SD_MMC.open(filename.c_str(), FILE_READ);
    if (!file) {
        Serial.println("Failed to open image file for sending");
        return;
    }
    
    size_t imageSize = file.size();
    const uint16_t MAX_DATA_PER_PACKET = 190;
    uint32_t totalPackets = (imageSize + MAX_DATA_PER_PACKET - 1) / MAX_DATA_PER_PACKET;
    uint32_t imageId = millis();
    
    Serial.printf("Sending image: %s, %d bytes in %d packets\n", 
                  filename.c_str(), imageSize, totalPackets);
    
    RouteNode* gwNode = radio.getBestNodeWithRole(ROLE_GATEWAY);
    if (!gwNode) {
        Serial.println("No gateway found!");
        file.close();
        return;
    }
    
    uint8_t buffer[MAX_DATA_PER_PACKET];
    for (uint32_t packetNum = 0; packetNum < totalPackets; packetNum++) {
        imagePacket* packet = new imagePacket;
        packet->packetNumber = packetNum;
        packet->totalPackets = totalPackets;
        packet->imageId = imageId;
        
        size_t bytesRead = file.read(buffer, MAX_DATA_PER_PACKET);
        packet->dataSize = bytesRead;
        memcpy(packet->imageData, buffer, bytesRead);
        
        radio.createPacketAndSend(gwNode->networkNode.address, packet, sizeof(imagePacket));
        Serial.printf("Sent packet %d/%d\n", packetNum + 1, totalPackets);
        
        delete packet;
        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay between packets
    }
    
    file.close();
    Serial.println("Image transmission complete!");
}

/**
 * @brief Print routing table to display
 */
void printRoutingTableToDisplay() {
    LM_LinkedList<RouteNode>* routingTableList = radio.routingTableListCopy();
    routingTableList->setInUse();

    Screen.changeSizeRouting(radio.routingTableSize());

    char buf[16];
    for (int i = 0; i < radio.routingTableSize(); i++) {
        RouteNode* rNode = (*routingTableList)[i];
        NetworkNode node = rNode->networkNode;
        snprintf(buf, sizeof(buf), "|%X(%d)->%X", node.address, node.metric, rNode->via);
        Screen.changeRoutingText(buf, i);
    }

    routingTableList->releaseInUse();
    delete routingTableList;
}

/**
 * @brief Process received packets
 */
void processReceivedPackets(void* parameter) {
    Serial.println("*** Receive Task Started ***");
    
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Fixed: pdTRUE not pdPASS
        
        while (radio.getReceivedQueueSize() > 0) {
            led_Flash(1, 100);
            
            // Try to get image packet first
            AppPacket<imagePacket>* imgPacket = radio.getNextAppPacket<imagePacket>();
            if (imgPacket != NULL) {
                Serial.printf("Received image packet %d/%d\n", 
                             imgPacket->payload->packetNumber + 1,
                             imgPacket->payload->totalPackets);
                // Handle image reconstruction here if needed
                radio.deletePacket(imgPacket);
            } else {
                // Try data packet
                AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();
                if (packet != NULL) {
                    Serial.printf("Received data packet from %X, counter: %d\n", 
                                 packet->src, packet->payload->counter);
                    Screen.changeLineTwo("Recv " + String(packet->payload->counter));
                    led_Flash(3, 100);
                    radio.deletePacket(packet);
                }
            }
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
    char addrStr[20];
    snprintf(addrStr, 20, "%s: %X", IS_GATEWAY ? "GW" : "Node", radio.getLocalAddress());
    Screen.changeLineOne(String(addrStr));
}

/**
 * @brief Send periodic messages
 */
void sendLoRaMessage(void*) {
    for (;;) {
        Serial.println("\n=== Send Task Running ===");
        
        if (radio.routingTableSize() == 0) {
            Serial.println("No routes available, waiting...");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        RouteNode* gwNode = radio.getBestNodeWithRole(ROLE_GATEWAY);
        if (!gwNode) {
            Serial.println("No gateway in routing table yet");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        // Send data packet
        helloPacket->counter = dataCounter++;
        for (int i = 0; i < sizeof(helloPacket->data); i++) {
            helloPacket->data[i] = (helloPacket->counter + i) & 0xFF;
        }

        radio.createPacketAndSend(gwNode->networkNode.address, helloPacket, sizeof(dataPacket));
        Serial.printf("Sent data packet %d to %X\n", helloPacket->counter, gwNode->networkNode.address);
        
        printRoutingTableToDisplay();
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void createSendMessages() {
    TaskHandle_t sendLoRaMessage_Handle = NULL;
    BaseType_t res = xTaskCreate(
        sendLoRaMessage,
        "Send LoRa Message",
        4096,
        (void*) 1,
        1,
        &sendLoRaMessage_Handle);
    if (res != pdPASS) {
        Serial.printf("Error: Send task creation gave error: %d\n", res);
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
    
    if (IS_GATEWAY) {
        radio.addGatewayRole();
        Serial.println("LoRa initialized as GATEWAY");
    } else {
        Serial.println("LoRa initialized as NODE");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== ESP32-S3 Camera + LoRa System ===");
    Serial.printf("Role: %s\n", IS_GATEWAY ? "GATEWAY/RECEIVER" : "NODE/CAMERA");
    
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_OFF);
    
    // Initialize display first if available
    Screen.initDisplay();
    delay(100);
    
    // Initialize GPIO ISR service after display
    gpio_install_isr_service(0);
    
    if (!IS_GATEWAY) {
        // Initialize photo trigger pins
        pinMode(PHOTO_PIN1, INPUT_PULLUP);
        pinMode(PHOTO_PIN2, INPUT_PULLUP);
        
        // Initialize camera
        Serial.println("Initializing camera...");
        initCamera();
        
        // Initialize SD card (optional)
        Serial.println("Initializing SD card...");
        initSD();
    }
    
    Serial.println("Board Init Complete");
    led_Flash(2, 125);
    
    setupLoraMesher();
    printAddressDisplay();
    
    if (!IS_GATEWAY) {
        createSendMessages();
    }
}

void loop() {
    // Handle photo trigger for camera nodes
    if (!IS_GATEWAY) {
        static bool lastState = false;
        bool photoTrigger = (digitalRead(PHOTO_PIN1) == LOW && digitalRead(PHOTO_PIN2) == LOW);
        
        if (photoTrigger && !lastState) {
            String filename;
            if (takeAndSavePhoto(filename)) {
                // Wait for gateway to be available
                RouteNode* gw = radio.getBestNodeWithRole(ROLE_GATEWAY);
                if (gw) {
                    sendImageOverLoRa(filename);
                } else {
                    Serial.println("Photo saved, but no gateway available for sending");
                }
            }
        }
        lastState = photoTrigger;
    }

    Screen.drawDisplay();
    
    // Status print
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 5000) {
        lastPrint = millis();
        Serial.printf("\nStatus - Address: %X, Routes: %d, Queue: %d\n", 
                     radio.getLocalAddress(), 
                     radio.routingTableSize(), 
                     radio.getReceivedQueueSize());
    }
    
    delay(50);
}