#pragma once
#include <stdint.h>

enum FtMsgType : uint8_t {
  FT_META     = 1,   // camera -> gateway
  FT_CHUNK    = 2,   // camera -> gateway
  FT_ACK      = 3,   // gateway -> camera (progress + first gap)
  FT_END      = 4,   // camera -> gateway (all sent)
  FT_END_ACK  = 5    // gateway -> camera (final OK/FAIL)
};

#define FT_FILE_NAME_MAX 40
#define FT_CHUNK_DATA    180   

#pragma pack(push,1)

struct FtMeta {
  uint8_t  type;                 
  uint32_t fileSize;             
  uint32_t crc32;                
  char     fileName[FT_FILE_NAME_MAX];
};

struct FtChunk {
  uint8_t  type;                 
  uint32_t offset;               
  uint16_t dataLen;              
  uint32_t chunkCrc32;           
  uint8_t  data[FT_CHUNK_DATA];
};

struct FtAck {
  uint8_t  type;                 
  uint32_t nextOffset;           
  uint32_t nackOffset;           
};

struct FtEnd  { uint8_t type; }; 

struct FtEndAck {
  uint8_t type;                  
  uint8_t status;                // 1=OK, 0=FAIL
};
#pragma pack(pop)

static inline bool ft_is_msg(uint8_t t) {
  return t >= FT_META && t <= FT_END_ACK;
}
