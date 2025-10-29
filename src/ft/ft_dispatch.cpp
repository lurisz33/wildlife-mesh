#include <Arduino.h>
#include "ft_protocol.h"
#include "ft_gateway.h"
#include "ft_camera.h"

#include <Arduino.h>
#include "ft_common.h"     
#include "ft_protocol.h"
#include "ft_gateway.h"
#include "ft_camera.h"

bool ft_handle_rx(uint16_t src, const uint8_t* p, size_t n) {
  if (n < 1) return false;

  if (p[0] == IMG_HDR) {
#if NODE_IS_GATEWAY
    if (n >= sizeof(ImageHeader)) {
      Serial.printf("[FT] Image header from %04X\n", src);
      ft_gw_onImageHeader(src, reinterpret_cast<const ImageHeader*>(p));
    } else {
      Serial.printf("[FT] Image header too short from %04X n=%u (need %u)\n",
                    src, (unsigned)n, (unsigned)sizeof(ImageHeader));
    }
#else
    Serial.println("[CAM] unexpected IMG_HDR");
#endif
    return true;
  }

  if (p[0] == IMG_DATA) {
#if NODE_IS_GATEWAY
    if (n >= 2) { 
      uint8_t len = p[1];
      if (n >= (size_t)(2 + len)) {
        ft_gw_onImageData(src, p + 2, len);
      } else {
        Serial.printf("[FT] Image data short from %04X n=%u len=%u\n",
                      src, (unsigned)n, (unsigned)len);
      }
    }
#else
    Serial.println("[CAM] unexpected IMG_DATA");
#endif
    return true;
  }

  if (!ft_is_msg(p[0])) return false;

  switch (p[0]) {
    case FT_META:
#if NODE_IS_GATEWAY
      if (n >= sizeof(FtMeta)) ft_gw_onMeta(src, (const FtMeta*)p, n);
      else Serial.printf("[FT] META short from %04X n=%u\n", src, (unsigned)n);
#else
      Serial.println("[CAM] unexpected META");
#endif
      return true;

    case FT_CHUNK:
#if NODE_IS_GATEWAY
      ft_gw_onChunk(src, (const FtChunk*)p, n);
#else
      Serial.println("[CAM] unexpected CHUNK");
#endif
      return true;

    case FT_END:
#if NODE_IS_GATEWAY
      ft_gw_onEnd(src, (const FtEnd*)p, n);
#else
      Serial.println("[CAM] unexpected END");
#endif
      return true;

    case FT_ACK:
#if !NODE_IS_GATEWAY
      if (n >= sizeof(FtAck)) ft_cam_onAck((const FtAck*)p);
#endif
      return true;

    default:
      Serial.printf("[FT] unhandled type %u\n", p[0]);
      return true;
  }
}
