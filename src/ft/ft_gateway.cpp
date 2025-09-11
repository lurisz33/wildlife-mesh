#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>

#include "ft_common.h"
#include "ft_protocol.h"
#include "ft_gateway.h"
#include "crc32.h"
#include <stddef.h> 

static inline bool chunkFrameSizeOK(size_t n, const FtChunk* c) {
  const size_t header = offsetof(FtChunk, data);
  if (n < header) return false;
  if (c->dataLen > FT_CHUNK_DATA) return false;
  if (n < header + c->dataLen) return false;
  return true;
}

const char FT_RX_BASE[] = "/rx";

struct FileRecvState {
  bool     inUse = false;
  uint16_t src = 0;
  uint32_t fileId = 0;
  uint32_t fileSize = 0;
  uint32_t receivedSize = 0;
  uint32_t crcExpected = 0;
  uint32_t crcRunning = 0xFFFFFFFFu;
  char     fileName[FT_FILE_NAME_MAX] = {0};
  File     file;
};

static constexpr int FT_MAX_SESS = 4;
static FileRecvState g_sess[FT_MAX_SESS];

static FileRecvState* findSess(uint16_t src, uint32_t fileId) {
  for (auto& s : g_sess) if (s.inUse && s.src == src && s.fileId == fileId) return &s;
  return nullptr;
}
static FileRecvState* allocSess(uint16_t src, uint32_t fileId) {
  if (auto* s = findSess(src, fileId)) return s;
  for (auto& s : g_sess) if (!s.inUse) {
    s = FileRecvState{};
    s.inUse = true;
    s.src = src;
    s.fileId = fileId;
    return &s;
  }
  return nullptr;
}

static void ensureDirForNode(uint16_t src) {
  SD_MMC.mkdir(FT_RX_BASE);
  char dir[32];
  snprintf(dir, sizeof(dir), "%s/%04X", FT_RX_BASE, (unsigned)src);
  SD_MMC.mkdir(dir);
}

static uint32_t scanExisting(const char* path, uint32_t& runningCrc) {
  runningCrc = 0xFFFFFFFFu;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return 0;
  uint8_t buf[512]; uint32_t total = 0;
  while (true) {
    int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    total += n;
    for (int i = 0; i < n; ++i) runningCrc = crc32_update(runningCrc, buf[i]);
  }
  f.close();
  return total;
}

static void sendAck(uint16_t dest, uint32_t fileId, uint32_t nextOffset,
                    uint32_t nackOffset = 0xFFFFFFFFu) {
  FtAck a{};
  a.type       = FT_ACK;
  a.fileId     = fileId;
  a.nextOffset = nextOffset;
  a.nackOffset = nackOffset;
  Serial.printf("[GW] Sending ACK to %X", dest);
  radio.createPacketAndSend(dest, &a, sizeof(a));
}

void ft_gw_setup() {
  Serial.println("[GW] Setup: Checking SD card...");

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("[GW] SD not mounted yet, skipping /rx mkdir");
    return;
  }

  Serial.printf("[GW] SD card mounted. Ensuring %s exists...\n", FT_RX_BASE);
  if (!SD_MMC.exists(FT_RX_BASE)) {
    if (SD_MMC.mkdir(FT_RX_BASE)) {
      Serial.println("[GW] /rx created successfully.");
    } else {
      Serial.println("[GW] Failed to create /rx!");
    }
  } else {
    Serial.println("[GW] /rx already exists.");
  }
}


void ft_gw_onMeta(uint16_t src, const FtMeta* m, size_t n) {
  if (n < sizeof(FtMeta)) {
    Serial.printf("[GW] META too short: n=%u (need %u)\n", (unsigned)n, (unsigned)sizeof(FtMeta));
    return;
  }

  // make sure fileName is NUL-terminated inside our fixed array
  char safeName[FT_FILE_NAME_MAX];
  memcpy(safeName, m->fileName, FT_FILE_NAME_MAX);          // <-- FIXED
  safeName[FT_FILE_NAME_MAX - 1] = '\0';

  // reject sneaky paths
  for (int i = 0; i < FT_FILE_NAME_MAX && safeName[i]; ++i) {
    if (safeName[i] == '/' || safeName[i] == '\\') { Serial.println("[GW] bad filename"); return; }
  }

  Serial.printf("[GW] Received META from %04X for file ID %08X (%s)\n",
                src, m->fileId, safeName);

  // Build paths
  char nodeDir[32];
  snprintf(nodeDir, sizeof(nodeDir), "%s/%04X", FT_RX_BASE, (unsigned)src);

  char path[128];
  snprintf(path, sizeof(path), "%s/%08X_%s", nodeDir, (unsigned)m->fileId, safeName);
  Serial.printf("[GW] nodeDir=%s\n", nodeDir);
  Serial.printf("[GW] fullPath=%s\n", path);

  // Ensure dirs exist
  if (!SD_MMC.exists(FT_RX_BASE)) {
    Serial.printf("[GW] %s missing, creating...\n", FT_RX_BASE);
    SD_MMC.mkdir(FT_RX_BASE);
  }
  if (!SD_MMC.exists(nodeDir)) {
    Serial.printf("[GW] %s missing, creating...\n", nodeDir);
    SD_MMC.mkdir(nodeDir);
  }

  // Resume support: see what we already have and the running CRC of those bytes
  uint32_t have = 0;
  uint32_t crcHave = 0xFFFFFFFFu;
  if (SD_MMC.exists(path)) {
    have = scanExisting(path, crcHave);                     // computes running CRC state
    Serial.printf("[GW] Existing file found, size=%u\n", have);
  } else {
    Serial.println("[GW] File does not exist yet, will create new.");
  }

  // ACK now (let camera know where to start/resume)
  Serial.printf("[GW] Sending ACK: fileId=%08X nextOffset=%u\n", m->fileId, have);
  sendAck(src, m->fileId, have);

  // Allocate/refresh session
  FileRecvState* s = allocSess(src, m->fileId);
  if (!s) { Serial.println("[GW] No free session slot; will rely on next META."); return; }
  Serial.println("[GW] Session allocated/refreshed.");

  // Open file in appropriate mode
  Serial.printf("[GW] Opening file in %s mode...\n", have ? "APPEND" : "WRITE");
  File f = SD_MMC.open(path, have ? FILE_APPEND : FILE_WRITE);
  if (!f) { Serial.printf("[GW] ERROR: Failed to open file: %s\n", path); return; }

  *s = FileRecvState{};   // reset slot
  s->inUse        = true;
  s->src          = src;
  s->fileId       = m->fileId;
  s->fileSize     = m->fileSize;
  s->receivedSize = have;
  s->crcExpected  = m->crc32;
  s->crcRunning   = (have ? crcHave : 0xFFFFFFFFu);         // <-- seed running CRC to match existing bytes
  strncpy(s->fileName, safeName, FT_FILE_NAME_MAX - 1);
  s->file = f;

  Serial.printf("[GW] Session state initialized for %s\n", s->fileName);
}


void ft_gw_onChunk(uint16_t src, const FtChunk* c, size_t n) {
  if (!chunkFrameSizeOK(n, c)) {
    Serial.printf("[GW] CHUNK invalid: n=%u dataLen=%u (max=%u)\n",
                  (unsigned)n, (unsigned)c->dataLen, (unsigned)FT_CHUNK_DATA);
    // Suggest restart from what we have (0 if no session yet)
    sendAck(src, c->fileId, 0, 0);
    return;
  }

  FileRecvState* s = findSess(src, c->fileId);
  if (!s || !s->inUse) {
    Serial.printf("[GW] CHUNK for unknown sess id=%08X from %04X, ask for META\n", c->fileId, src);
    sendAck(src, c->fileId, 0, 0); // suggest restart
    return;
  }

  // linear protocol: exact offset expected
  if (c->offset != s->receivedSize) {
    Serial.printf("[GW] CHUNK offset %u != expected %u -> NACK\n",
                  (unsigned)c->offset, (unsigned)s->receivedSize);
    sendAck(src, c->fileId, s->receivedSize, s->receivedSize);
    return;
  }

  // CRC of payload
  uint32_t crc = 0xFFFFFFFFu;
  for (uint16_t i = 0; i < c->dataLen; ++i) crc = crc32_update(crc, c->data[i]);
  crc ^= 0xFFFFFFFFu;
  if (crc != c->chunkCrc32) {
    Serial.printf("[GW] CHUNK CRC mismatch (%08X != %08X)\n",
                  (unsigned)crc, (unsigned)c->chunkCrc32);
    sendAck(src, c->fileId, s->receivedSize, s->receivedSize);
    return;
  }

  // append
  size_t wrote = s->file.write(c->data, c->dataLen);
  if (wrote != c->dataLen) {
    Serial.printf("[GW] write failed (%u/%u)\n", (unsigned)wrote, (unsigned)c->dataLen);
    sendAck(src, c->fileId, s->receivedSize, s->receivedSize);
    return;
  }

  for (uint16_t i = 0; i < c->dataLen; ++i) s->crcRunning = crc32_update(s->crcRunning, c->data[i]);
  s->receivedSize += c->dataLen;

  // progress ACK (every chunk)
  sendAck(src, c->fileId, s->receivedSize);
  Serial.printf("[GW] CHUNK ok -> next=%u\n", (unsigned)s->receivedSize);
}


void ft_gw_onEnd(uint16_t src, const FtEnd* e, size_t n) {
  if (n < sizeof(FtEnd)) { Serial.println("[GW] END short"); return; }

  FileRecvState* s = findSess(src, e->fileId);
  if (!s || !s->inUse) { Serial.printf("[GW] END for unknown sess id=%08X\n", e->fileId); return; }

  s->file.flush();
  s->file.close();

  uint32_t finalCrc = s->crcRunning ^ 0xFFFFFFFFu;
  bool sizeOk = (s->receivedSize == s->fileSize);
  bool crcOk  = (finalCrc == s->crcExpected);

  Serial.printf("[GW] END: size %s (%u/%u), crc %s (%08X/%08X) -> %s\n",
    sizeOk ? "OK" : "BAD", (unsigned)s->receivedSize, (unsigned)s->fileSize,
    crcOk ? "OK" : "BAD", (unsigned)finalCrc, (unsigned)s->crcExpected,
    (sizeOk && crcOk) ? "SUCCESS" : "FAIL");

  *s = FileRecvState{}; // free slot
}


