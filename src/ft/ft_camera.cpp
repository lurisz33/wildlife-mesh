#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>
#include "ft_common.h"
#include "ft_protocol.h"
#include "crc32.h"
#include <stddef.h>

constexpr uint16_t TX_DATA_BUDGET = 30;

#if !NODE_IS_GATEWAY
static volatile bool s_ack_ready = false;
static FtAck         s_ack_last;

static bool getFileInfo(const char* relName, uint32_t& outSize, uint32_t& outCrc) {
  char path[96];
  snprintf(path, sizeof(path), "/camera/%s", relName);
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;

  outSize = f.size();
  uint8_t buf[512];
  uint32_t crc = 0xFFFFFFFFu;
  while (true) {
    int n = f.read(buf, sizeof(buf)); if (n <= 0) break;
    for (int i = 0; i < n; ++i) crc = crc32_update(crc, buf[i]);
  }
  f.close();
  outCrc = crc ^ 0xFFFFFFFFu;
  return true;
}

void ft_cam_onAck(const FtAck* a) {
  s_ack_last = *a; s_ack_ready = true;
  Serial.printf("[CAM] RX ACK: id=%08X next=%u nack=%u\n",
                (unsigned)a->fileId, (unsigned)a->nextOffset, (unsigned)a->nackOffset);
}

bool ft_cam_handshakeMeta(const char* relName, uint32_t& fileId, uint32_t& startOffset) {
  static uint32_t lastFileId = 0;
  static char lastFileName[FT_FILE_NAME_MAX] = {0};
  
  RouteNode* gw = radio.getBestNodeWithRole(ROLE_GATEWAY);
  if (!gw) { Serial.println("[CAM] no GW route"); return false; }

  uint32_t fsize=0, fcrc=0;
  if (!getFileInfo(relName, fsize, fcrc)) {
    Serial.println("[CAM] file not found on /camera"); return false;
  }
  uint32_t fid;
  if (strcmp(lastFileName, relName) == 0 && lastFileId != 0) {
    fid = lastFileId; 
  } else {
    fid = esp_random();
    lastFileId = fid;
    strncpy(lastFileName, relName, FT_FILE_NAME_MAX - 1);
  }
  
  FtMeta m{};
  m.type     = FT_META;
  m.fileId   = fid;
  m.fileSize = fsize;
  m.crc32    = fcrc;
  strncpy(m.fileName, relName, FT_FILE_NAME_MAX - 1);

  s_ack_ready = false;
  Serial.printf("[CAM] Sending META to GW %X for %s size=%u crc=%08X\n",
                gw->networkNode.address, relName, (unsigned)fsize, (unsigned)fcrc);
  radio.sendReliable  (gw->networkNode.address, &m, sizeof(m));
  Serial.printf("[CAM] META sent: id=%08X %s size=%u crc=%08X\n",
                (unsigned)fid, relName, (unsigned)fsize, (unsigned)fcrc);

  uint32_t t0 = millis();
  while (millis() - t0 < 20000) {
    if (s_ack_ready) {
      s_ack_ready = false;
      if (s_ack_last.fileId == fid) {
        startOffset = s_ack_last.nextOffset;
        fileId = fid;
        Serial.printf("[CAM] META ACK: start=%u\n", (unsigned)startOffset);
        lastFileId = 0;  
        return true;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  Serial.println("[CAM] META ACK timeout");
  return false;
}


bool ft_cam_sendFile(const char* relName, uint32_t fileId, uint32_t startOffset) {
  RouteNode* gw = radio.getBestNodeWithRole(ROLE_GATEWAY);
  if (!gw) { Serial.println("[CAM] send: no GW route"); return false; }

  // Open file on camera SD
  char path[96];
  snprintf(path, sizeof(path), "/camera/%s", relName);
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) { Serial.printf("[CAM] send: can't open %s\n", path); return false; }

  const uint32_t fileSize = f.size();
  uint32_t off = startOffset;
  if (off > fileSize) off = fileSize;
  if (off > 0) f.seek(off);

  Serial.printf("[CAM] FT_CHUNK_DATA=%u, budget=%u, header=%u, fileSize=%u, start=%u\n",
                (unsigned)FT_CHUNK_DATA, (unsigned)TX_DATA_BUDGET,
                (unsigned)offsetof(FtChunk, data),
                (unsigned)fileSize, (unsigned)off);

  // Buffer sized to the *actual* per-frame budget
  uint8_t buf[TX_DATA_BUDGET];
  Serial.printf("[CAM] send: start at %u of %u bytes\n", (unsigned)off, (unsigned)fileSize);

  while (off < fileSize) {
    // Read only what fits in one mesh frame
    uint32_t remaining = fileSize - off;
    int toRead = (int)min((uint32_t)TX_DATA_BUDGET, remaining);
    if (toRead <= 0) break;

    int n = f.read(buf, toRead);
    if (n <= 0) { Serial.println("[CAM] read returned 0"); break; }

    // Build CHUNK
    FtChunk c{};
    c.type    = FT_CHUNK;
    c.fileId  = fileId;
    c.offset  = off;
    c.dataLen = (uint16_t)n;

    uint32_t crc = 0xFFFFFFFFu;
    for (int i = 0; i < n; ++i) crc = crc32_update(crc, buf[i]);
    c.chunkCrc32 = crc ^ 0xFFFFFFFFu;

    memcpy(c.data, buf, n);
    const size_t payloadLen = offsetof(FtChunk, data) + (size_t)n;

    // Stop-and-wait for an ACK/NACK for this chunk
    const uint32_t ACK_WAIT_MS = 800;   // LoRa-friendly window
    int tries = 0;
    bool progressed = false;

    while (!progressed && tries < 4) {
      // (Re)send this chunk
      RouteNode* gwNow = radio.getBestNodeWithRole(ROLE_GATEWAY);
      if (!gwNow) { Serial.println("[CAM] GW route vanished"); f.close(); return false; }
      radio.createPacketAndSend(gwNow->networkNode.address, &c, payloadLen);
      // Serial.printf("[CAM] CHUNK off=%u len=%u\n", (unsigned)off, (unsigned)n);

      // Wait for ACK/NACK
      uint32_t t0 = millis();
      while (millis() - t0 < ACK_WAIT_MS) {
        if (s_ack_ready) {
          s_ack_ready = false;
          if (s_ack_last.fileId == fileId) {
            uint32_t next = s_ack_last.nextOffset;
            uint32_t nack = s_ack_last.nackOffset;

            if (nack != 0xFFFFFFFFu) {
              // GW requests re-send from 'nack'
              if (nack > fileSize) nack = fileSize;
              if (nack != off) {
                Serial.printf("[CAM] NACK -> set off=%u (was %u)\n",
                              (unsigned)nack, (unsigned)off);
                off = nack;
                f.seek(off);
              }
              // We'll loop and re-send from new 'off'
              progressed = true;   // exit tries loop; outer loop will continue at new 'off'
              break;
            } else if (next > off) {
              // Normal forward progress
              off = next;
              progressed = true;
              break;
            } else {
              // ACK did not advance; keep waiting (will re-send if we time out)
            }
          }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      if (!progressed) {
        ++tries;
        Serial.printf("[CAM] ACK timeout -> resend try %d at off=%u len=%u\n",
                      tries, (unsigned)off, (unsigned)n);
        f.seek(off);  // ensure we re-read same bytes if GW didn’t advance
      }
    }

    if (!progressed) {
      Serial.printf("[CAM] give up at off=%u (file %u) -> abort\n",
                    (unsigned)off, (unsigned)fileSize);
      f.close();
      return false;  // do not send END
    }

    vTaskDelay(pdMS_TO_TICKS(3));  // small pacing
  }

  f.close();

  // Send END only if we actually finished the file
  if (off == fileSize) {
    FtEnd e{}; e.type = FT_END; e.fileId = fileId;
    if (RouteNode* gwEnd = radio.getBestNodeWithRole(ROLE_GATEWAY)) {
      radio.sendReliable(gwEnd->networkNode.address, &e, sizeof(FtEnd));
    }
    Serial.println("[CAM] send: END sent");
    return true;
  } else {
    Serial.printf("[CAM] send: ABORT at %u/%u, not sending END\n",
                  (unsigned)off, (unsigned)fileSize);
    return false;
  }
}




#endif
