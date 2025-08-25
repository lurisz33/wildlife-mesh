#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>

#include "ft_common.h"
#include "ft_protocol.h"
#include "ft_gateway.h"
#include "crc32.h"

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
  radio.sendReliable(dest, &a, sizeof(a));
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


void ft_gw_onMeta(uint16_t src, const FtMeta* m) {
  Serial.printf("[GW] Received META from %04X for file ID %08X (%s)\n",
                src, m->fileId, m->fileName);

  // --- Build path(s)
  char nodeDir[32];
  snprintf(nodeDir, sizeof(nodeDir), "%s/%04X", FT_RX_BASE, (unsigned)src);

  char path[128];
  snprintf(path, sizeof(path), "%s/%08X_%s",
           nodeDir, (unsigned)m->fileId, m->fileName);
  Serial.printf("[GW] nodeDir=%s\n", nodeDir);
  Serial.printf("[GW] fullPath=%s\n", path);

  // --- Ensure dirs exist
  if (!SD_MMC.exists(FT_RX_BASE)) {
    Serial.printf("[GW] %s missing, creating...\n", FT_RX_BASE);
    SD_MMC.mkdir(FT_RX_BASE);
  }

  if (!SD_MMC.exists(nodeDir)) {
    Serial.printf("[GW] %s missing, creating...\n", nodeDir);
    SD_MMC.mkdir(nodeDir);
  }

  // --- Find current file size
  uint32_t have = 0;
  if (SD_MMC.exists(path)) {
    File t = SD_MMC.open(path, FILE_READ);
    if (t) {
      have = t.size();
      Serial.printf("[GW] Existing file found, size=%u\n", have);
      t.close();
    } else {
      Serial.println("[GW] File exists but failed to open for size check!");
    }
  } else {
    Serial.println("[GW] File does not exist yet, will create new.");
  }

  // --- ACK now
  Serial.printf("[GW] Sending ACK: fileId=%08X nextOffset=%u\n", m->fileId, have);
  sendAck(src, m->fileId, have);

  // --- Allocate or refresh session
  FileRecvState* s = allocSess(src, m->fileId);
  if (!s) {
    Serial.println("[GW] No free session slot; will rely on next META.");
    return;
  }
  Serial.println("[GW] Session allocated/refreshed.");

  // --- Open file
  Serial.printf("[GW] Opening file in %s mode...\n", have ? "APPEND" : "WRITE");
  File f = SD_MMC.open(path, have ? FILE_APPEND : FILE_WRITE);
  if (!f) {
    Serial.printf("[GW] ERROR: Failed to open file: %s\n", path);
    return;
  }

  *s = FileRecvState{};
  s->inUse        = true;
  s->src          = src;
  s->fileId       = m->fileId;
  s->fileSize     = m->fileSize;
  s->receivedSize = have;
  s->crcExpected  = m->crc32;
  s->crcRunning   = 0xFFFFFFFFu;
  strncpy(s->fileName, m->fileName, FT_FILE_NAME_MAX - 1);
  s->file = f;

  Serial.printf("[GW] Session state initialized for %s\n", s->fileName);
}


