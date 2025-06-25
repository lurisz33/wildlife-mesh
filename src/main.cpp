#include <Arduino.h>
#include <LoraMesher.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_NSS   5
#define PIN_RST   14
#define PIN_DIO1  26 
#define PIN_BUSY  27 

LoraMesher& radio = LoraMesher::getInstance();

uint32_t dataCounter = 0;
struct dataPacket {
    uint32_t counter = 0;
};

dataPacket* helloPacket = new dataPacket;

void printPacket(dataPacket data) {
    Serial.printf("Hello Counter received nº %d\n", data.counter);
}

void printDataPacket(AppPacket<dataPacket>* packet) {
    Serial.printf("Packet arrived from %X with size %d\n", packet->src, packet->payloadSize);

    //Get the payload to iterate through it
    dataPacket* dPacket = packet->payload;
    size_t payloadLength = packet->getPayloadLength();

    for (size_t i = 0; i < payloadLength; i++) {
        //Print the packet
        printPacket(dPacket[i]);
    }
}


void processReceivedPackets(void*) {
    for (;;) {
        /* Wait for the notification of processReceivedPackets and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);

        //Iterate through all the packets inside the Received User Packets Queue
        while (radio.getReceivedQueueSize() > 0) {
            Serial.println("ReceivedUserData_TaskHandle notify received");
            Serial.printf("Queue receiveUserData size: %d\n", radio.getReceivedQueueSize());

            //Get the first element inside the Received User Packets Queue
            AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();

            //Print the data packet
            printDataPacket(packet);

            //Delete the packet when used. It is very important to call this function to release the memory of the packet.
            radio.deletePacket(packet);
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

void setupLoraMesher() {
    Serial.println("Creating LoRaMesher config...");

    LoraMesher::LoraMesherConfig cfg = LoraMesher::LoraMesherConfig();
    cfg.loraCs = PIN_NSS;
    cfg.loraRst = PIN_RST;
    cfg.loraIrq = PIN_DIO1;
    cfg.loraIo1 = PIN_BUSY;
    cfg.freq = 2400.0;
    cfg.bw = 812.5;
    cfg.sf = 9;
    cfg.cr = 7;
    cfg.syncWord = 0x12;
    cfg.power = 10;
    cfg.preambleLength = 12;
    cfg.module = LoraMesher::LoraModules::SX1280_MOD;
    sp_err_t r = gpio_install_isr_service(0);
    if (r == ESP_OK) {
        Serial.println("GPIO ISR service installed");
    } else if (r == ESP_ERR_INVALID_STATE) {
        Serial.println("GPIO ISR already installed");
    } else {
        Serial.printf("GPIO ISR install failed: %d\n", r);
    }

    radio.begin(cfg);
    createReceiveMessages();

    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
    radio.start();

    Serial.println("Lora initialized");
}

void setup() {
    Serial.begin(115200);

    Serial.println("initBoard");    
    setupLoraMesher();
}

uint32_t loopCounter = 0;

void loop() {
    for (;;) {
        Serial.printf("Send packet %d\n", dataCounter);

        helloPacket->counter = dataCounter++;

        radio.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);

        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}