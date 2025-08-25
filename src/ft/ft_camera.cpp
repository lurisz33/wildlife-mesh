#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>
#include "ft_common.h"
#include "ft_protocol.h"
#include "crc32.h"

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
  s_ack_last = *a;
  s_ack_ready = true;
}

bool ft_cam_handshakeMeta(const char* relName, uint32_t& fileId, uint32_t& startOffset) {
  RouteNode* gw = radio.getBestNodeWithRole(ROLE_GATEWAY);
  if (!gw) { Serial.println("[CAM] no GW route"); return false; }

  uint32_t fsize=0, fcrc=0;
  if (!getFileInfo(relName, fsize, fcrc)) {
    Serial.println("[CAM] file not found on /camera"); return false;
  }

  uint32_t fid = esp_random();
  FtMeta m{};
  m.type     = FT_META;
  m.fileId   = fid;
  m.fileSize = fsize;
  m.crc32    = fcrc;
  strncpy(m.fileName, relName, FT_FILE_NAME_MAX - 1);

  s_ack_ready = false;
  radio.sendReliable(gw->networkNode.address, &m, sizeof(m));
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
        return true;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  Serial.println("[CAM] META ACK timeout");
  return false;
}
#endif
