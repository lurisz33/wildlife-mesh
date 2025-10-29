#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>
#include <stddef.h>
#include "ft_common.h"
#include "ft_protocol.h"
#include "crc32.h"

#if !NODE_IS_GATEWAY
uint32_t const WAIT_FOR_ACK = 30000;


static bool getFileInfo(const char* relName, uint32_t& outSize, uint32_t& outCrc){
  char path[96];
  snprintf(path,sizeof(path),"/camera/%s",relName);
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;
  outSize = f.size();
  uint8_t buf[512]; uint32_t crc=0xFFFFFFFFu;
  while (true){ int n=f.read(buf,sizeof(buf)); if(n<=0) break;
    for(int i=0;i<n;++i) crc=crc32_update(crc,buf[i]); }
  f.close(); outCrc = crc ^ 0xFFFFFFFFu; return true;
}


bool ft_cam_sendFile(const char* relName,
                     uint32_t /*fileId*/,  
                     uint32_t startOffset,
                     uint16_t gwAddr)
{
  char path[64];
  snprintf(path, sizeof(path), "/camera/%s", relName);
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) { Serial.printf("[CAM] send: can't open %s\n", path); return false; }

  const uint32_t fileSize = f.size();
  uint8_t* fileData = (uint8_t*)malloc(fileSize);
  if (!fileData) { Serial.println("[CAM] Not enough memory for file"); f.close(); return false; }

  size_t bytesRead = f.read(fileData, fileSize);
  f.close();
  if (bytesRead != fileSize) {
    Serial.println("[CAM] File read error");
    free(fileData);
    return false;
  }


  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < fileSize; i++) crc = crc32_update(crc, fileData[i]);
  crc ^= 0xFFFFFFFFu;

  ImageHeader header{};
  header.type = IMG_HDR;
  strncpy(header.filename, relName, sizeof(header.filename) - 1);
  header.fileSize  = fileSize;
  header.crc32     = crc;
  header.timestamp = millis();

  Serial.println("[CAM] Sending image header...");
  radio.createPacketAndSend(gwAddr, &header, sizeof(header));

  if (startOffset > fileSize) startOffset = fileSize;
  uint32_t off = startOffset;

  Serial.printf("[CAM] send: %u bytes -> %04X (start=%u)\n",
                (unsigned)fileSize, gwAddr, (unsigned)off);

  while (off < fileSize) {
    uint8_t n = (uint8_t)min<uint32_t>(IMG_CHUNK_MAX, fileSize - off);

    uint8_t frame[1 + 1 + IMG_CHUNK_MAX]; 
    frame[0] = IMG_DATA;
    frame[1] = n;
    memcpy(&frame[2], &fileData[off], n);

    radio.createPacketAndSend(gwAddr, frame, (size_t)(2 + n));
    off += n;
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }

  free(fileData);
  return true;
}


void ft_cam_onAck(const FtAck* a) {
    Serial.printf("[CAM] ACK received: fileId=%08X, nextOffset=%u\n", 
                  a->fileId, a->nextOffset);
}
#endif
