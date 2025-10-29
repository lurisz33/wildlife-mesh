#pragma once
#include "LoraMesher.h"
#include <FS.h>

#ifndef NODE_IS_GATEWAY
#define NODE_IS_GATEWAY 0   
#endif

#define GATEWAY_ADDR 0xE04C 
#define CAMERA_ADDR  0x545C 

struct ImageTransferState {
    bool active = false;
    uint16_t senderAddr = 0;
    char filename[32] = {0};
    uint32_t expectedSize = 0;
    uint32_t receivedSize = 0;
    uint32_t expectedCrc32 = 0;
    uint32_t runningCrc32 = 0xFFFFFFFFu;
    unsigned long lastActivity = 0;
    File file;
    char filePath[128] = {0};
};

#pragma pack(push,1)
struct ImageHeader {
  uint8_t  type;         
  char     filename[16];  
  uint32_t fileSize;
  uint32_t crc32;
  uint32_t timestamp;
};
#pragma pack(pop)
static_assert(sizeof(ImageHeader) <= 34, "ImageHeader too large");

extern LoraMesher& radio;

enum : uint8_t {
  IMG_HDR  = 63,  
  IMG_DATA = 64,  
  IMG_HDR_ACK = 65, 
};
constexpr uint8_t IMG_CHUNK_MAX = 32; 