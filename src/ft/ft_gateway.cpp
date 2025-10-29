#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>
#include <stddef.h>
#include "ft_common.h"
#include "ft_protocol.h"
#include "ft_gateway.h"
#include "crc32.h"

const char FT_RX_BASE[] = "/rx";

struct FileRecvState {
  bool     inUse=false;
  uint16_t src=0;
  uint32_t fileId=0, fileSize=0, receivedSize=0;
  uint32_t crcExpected=0, crcRunning=0xFFFFFFFFu;
  char     fileName[FT_FILE_NAME_MAX]={0};
  File     file;
};
static constexpr int FT_MAX_SESS = 4;
static FileRecvState g_sess[FT_MAX_SESS];

static FileRecvState* findSess(uint16_t src, uint32_t fileId){
  for (auto& s: g_sess) if (s.inUse && s.src==src && s.fileId==fileId) return &s;
  return nullptr;
}
static FileRecvState* allocSess(uint16_t src, uint32_t fileId){
  if (auto* s=findSess(src,fileId)) return s;
  for (auto& s: g_sess) if(!s.inUse){ s = FileRecvState{}; s.inUse=true; s.src=src; s.fileId=fileId; return &s; }
  return nullptr;
}

static uint32_t scanExisting(const char* path, uint32_t& runningCrc){
  runningCrc=0xFFFFFFFFu; File f=SD_MMC.open(path, FILE_READ); if(!f) return 0;
  uint8_t buf[512]; uint32_t total=0;
  while(true){ int n=f.read(buf,sizeof(buf)); if(n<=0) break;
    total+=n; for(int i=0;i<n;++i) runningCrc=crc32_update(runningCrc,buf[i]); }
  f.close(); return total;
}

static inline bool chunkFrameSizeOK(size_t n, const FtChunk* c){
  const size_t header = offsetof(FtChunk, data);
  if (n < header) return false;
  if (c->dataLen > FT_CHUNK_DATA) return false;
  if (n < header + c->dataLen) return false;
  return true;
}

static void sendAck(uint16_t dest, uint32_t fileId, uint32_t nextOffset,
                    uint32_t nackOffset=0xFFFFFFFFu){
  FtAck a{}; a.type=FT_ACK; a.fileId=fileId; a.nextOffset=nextOffset; a.nackOffset=nackOffset;
  radio.createPacketAndSend(dest, &a, sizeof(a)); 
}

void ft_gw_setup(){
  if (SD_MMC.cardType()==CARD_NONE) return;
  if (!SD_MMC.exists(FT_RX_BASE)) SD_MMC.mkdir(FT_RX_BASE);
}

void ft_gw_onMeta(uint16_t src, const FtMeta* m, size_t n){
  Serial.printf("[GW] META n=%u (need %u)\n", (unsigned)n, (unsigned)sizeof(FtMeta)); 
  Serial.printf("[GW] META name='%s' id=%08X size=%u crc=%08X\n",
                m->fileName, (unsigned)m->fileId, (unsigned)m->fileSize, (unsigned)m->crc32);

  if (n < sizeof(FtMeta)){ Serial.printf("[GW] META short n=%u\n",(unsigned)n); return; }

  char safeName[FT_FILE_NAME_MAX]; memcpy(safeName,m->fileName,FT_FILE_NAME_MAX);
  safeName[FT_FILE_NAME_MAX-1]='\0';
  for(int i=0; i<FT_FILE_NAME_MAX && safeName[i]; ++i)
    if (safeName[i]=='/' || safeName[i]=='\\'){ Serial.println("[GW] bad filename"); return; }

  char nodeDir[32]; snprintf(nodeDir,sizeof(nodeDir),"%s/%04X",FT_RX_BASE,(unsigned)src);
  char path[128];   snprintf(path,sizeof(path),"%s/%08X_%s",nodeDir,(unsigned)m->fileId,safeName);

  if (!SD_MMC.exists(FT_RX_BASE)) SD_MMC.mkdir(FT_RX_BASE);
  if (!SD_MMC.exists(nodeDir))    SD_MMC.mkdir(nodeDir);

  uint32_t have=0, crcHave=0xFFFFFFFFu;
  if (SD_MMC.exists(path)) have = scanExisting(path, crcHave);
  sendAck(src, m->fileId, have);

  FileRecvState* s = allocSess(src, m->fileId);
  if (!s){ Serial.println("[GW] no free session"); return; }

  File f = SD_MMC.open(path, have ? FILE_APPEND : FILE_WRITE);
  if (!f){ Serial.printf("[GW] open fail %s\n", path); return; }

  *s = FileRecvState{};
  s->inUse=true; s->src=src; s->fileId=m->fileId; s->fileSize=m->fileSize;
  s->receivedSize=have; s->crcExpected=m->crc32; s->crcRunning = have ? crcHave : 0xFFFFFFFFu;
  strncpy(s->fileName, safeName, FT_FILE_NAME_MAX-1);
  s->file=f;

  Serial.printf("[GW] META ok %s have=%u\n", s->fileName, (unsigned)have);
}

void ft_gw_onChunk(uint16_t src, const FtChunk* c, size_t n){
  if (!chunkFrameSizeOK(n,c)){
    Serial.printf("[GW] CHUNK invalid: n=%u dataLen=%u (max=%u)\n",
                  (unsigned)n,(unsigned)c->dataLen,(unsigned)FT_CHUNK_DATA);
    sendAck(src, c->fileId, 0, 0);
    return;
  }
  FileRecvState* s = findSess(src, c->fileId);
  if (!s || !s->inUse){ sendAck(src, c->fileId, 0, 0); return; }

  if (c->offset != s->receivedSize){
    sendAck(src, c->fileId, s->receivedSize, s->receivedSize);
    return;
  }

  uint32_t crc=0xFFFFFFFFu; for(uint16_t i=0;i<c->dataLen;++i) crc=crc32_update(crc,c->data[i]);
  crc ^= 0xFFFFFFFFu;
  if (crc != c->chunkCrc32){
    sendAck(src, c->fileId, s->receivedSize, s->receivedSize);
    return;
  }

  size_t w = s->file.write(c->data, c->dataLen);
  if (w != c->dataLen){
    sendAck(src, c->fileId, s->receivedSize, s->receivedSize);
    return;
  }
  for(uint16_t i=0;i<c->dataLen;++i) s->crcRunning=crc32_update(s->crcRunning,c->data[i]);
  s->receivedSize += c->dataLen;

  sendAck(src, c->fileId, s->receivedSize);
}

void ft_gw_onEnd(uint16_t src, const FtEnd* e, size_t n){
  if (n < sizeof(FtEnd)) return;
  FileRecvState* s = findSess(src, e->fileId);
  if (!s || !s->inUse) return;

  s->file.flush(); s->file.close();
  uint32_t finalCrc = s->crcRunning ^ 0xFFFFFFFFu;
  bool sizeOk = (s->receivedSize == s->fileSize);
  bool crcOk  = (finalCrc == s->crcExpected);
  Serial.printf("[GW] END: size %s (%u/%u), crc %s (%08X/%08X) -> %s\n",
    sizeOk?"OK":"BAD",(unsigned)s->receivedSize,(unsigned)s->fileSize,
    crcOk?"OK":"BAD",(unsigned)finalCrc,(unsigned)s->crcExpected,
    (sizeOk && crcOk)?"SUCCESS":"FAIL");

  *s = FileRecvState{};
}


static ImageTransferState g_imageTransfer;

void ft_gw_onImageHeader(uint16_t src, const ImageHeader* header) {
    Serial.printf("[IMG] ========================================\n");
    Serial.printf("[IMG] Image header received from %04X\n", src);
    Serial.printf("[IMG] Filename: %s\n", header->filename);
    Serial.printf("[IMG] Size: %u bytes\n", header->fileSize);
    Serial.printf("[IMG] CRC32: %08X\n", header->crc32);
    Serial.printf("[IMG] Timestamp: %u\n", header->timestamp);
    
    if (g_imageTransfer.active && g_imageTransfer.file) {
        Serial.printf("[IMG] Closing previous transfer from %04X\n", g_imageTransfer.senderAddr);
        g_imageTransfer.file.close();
    }
    
    g_imageTransfer = ImageTransferState();
    g_imageTransfer.active = true;
    g_imageTransfer.senderAddr = src;
    strncpy(g_imageTransfer.filename, header->filename, sizeof(g_imageTransfer.filename) - 1);
    g_imageTransfer.expectedSize = header->fileSize;
    g_imageTransfer.expectedCrc32 = header->crc32;
    g_imageTransfer.receivedSize = 0;
    g_imageTransfer.runningCrc32 = 0xFFFFFFFFu;
    g_imageTransfer.lastActivity = millis();
    
    if (!SD_MMC.exists("/rx")) {
        SD_MMC.mkdir("/rx");
        Serial.println("[IMG] Created /rx directory");
    }
    
    char nodeDir[32];
    snprintf(nodeDir, sizeof(nodeDir), "/rx/%04X", src);
    if (!SD_MMC.exists(nodeDir)) {
        SD_MMC.mkdir(nodeDir);
        Serial.printf("[IMG] Created directory: %s\n", nodeDir);
    }
    
    snprintf(g_imageTransfer.filePath, sizeof(g_imageTransfer.filePath), 
             "/rx/%04X/%08X_%s", src, header->timestamp, header->filename);
    
    g_imageTransfer.file = SD_MMC.open(g_imageTransfer.filePath, FILE_WRITE);
    if (!g_imageTransfer.file) {
        Serial.printf("[IMG] ERROR: Failed to create file: %s\n", g_imageTransfer.filePath);
        g_imageTransfer.active = false;
        return;
    }
    
    Serial.printf("[IMG] File created: %s\n", g_imageTransfer.filePath);
    Serial.printf("[IMG] Waiting for data chunks...\n");
    Serial.printf("[IMG] ========================================\n");
}

void ft_gw_onImageData(uint16_t src, const uint8_t* data, size_t len) {
    if (!g_imageTransfer.active) {
        Serial.printf("[IMG] WARNING: Received data but no active transfer from %04X\n", src);
        return;
    }
    
    if (g_imageTransfer.senderAddr != src) {
        Serial.printf("[IMG] WARNING: Data from %04X but expecting from %04X\n", 
                      src, g_imageTransfer.senderAddr);
        return;
    }
    
    g_imageTransfer.lastActivity = millis();
    
    if (g_imageTransfer.receivedSize + len > g_imageTransfer.expectedSize) {
        len = g_imageTransfer.expectedSize - g_imageTransfer.receivedSize;
        Serial.printf("[IMG] Trimming chunk to fit expected size\n");
    }
    
    if (len == 0) {
        Serial.printf("[IMG] WARNING: Zero-length chunk\n");
        return;
    }
    
    size_t written = g_imageTransfer.file.write(data, len);
    if (written != len) {
        Serial.printf("[IMG] ERROR: Write failed - wrote %u of %u bytes\n", written, len);
        g_imageTransfer.file.close();
        g_imageTransfer.active = false;
        return;
    }
    
    for (size_t i = 0; i < len; i++) {
        g_imageTransfer.runningCrc32 = crc32_update(g_imageTransfer.runningCrc32, data[i]);
    }
    
    g_imageTransfer.receivedSize += len;
    
    static uint32_t lastProgressPrint = 0;
    float progress = (float)g_imageTransfer.receivedSize / g_imageTransfer.expectedSize * 100.0f;
    if (progress - lastProgressPrint >= 10.0f || g_imageTransfer.receivedSize >= g_imageTransfer.expectedSize) {
        Serial.printf("[IMG] Progress: %.1f%% (%u/%u bytes)\n", 
                      progress, g_imageTransfer.receivedSize, g_imageTransfer.expectedSize);
        lastProgressPrint = (uint32_t)progress;
    }
    
    if (g_imageTransfer.receivedSize >= g_imageTransfer.expectedSize) {
        g_imageTransfer.file.close();
        
        uint32_t finalCrc = g_imageTransfer.runningCrc32 ^ 0xFFFFFFFFu;
        bool crcOk = (finalCrc == g_imageTransfer.expectedCrc32);
        
        Serial.printf("[IMG] ========================================\n");
        Serial.printf("[IMG] Transfer COMPLETE: %s\n", g_imageTransfer.filename);
        Serial.printf("[IMG] Size: %u bytes\n", g_imageTransfer.receivedSize);
        Serial.printf("[IMG] CRC32: %08X (expected %08X) - %s\n", 
                      finalCrc, g_imageTransfer.expectedCrc32, crcOk ? "MATCH" : "MISMATCH");
        Serial.printf("[IMG] File saved: %s\n", g_imageTransfer.filePath);
        Serial.printf("[IMG] ========================================\n");
        
        g_imageTransfer.active = false;
        lastProgressPrint = 0;
    }
}

bool ft_gw_hasActiveImageTransfer(uint16_t src) {
    return g_imageTransfer.active && g_imageTransfer.senderAddr == src;
}
