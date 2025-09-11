#include <Arduino.h>
#include "ft_protocol.h"
#include "ft_gateway.h"
#include "ft_camera.h"
#include "ft_common.h" 

bool ft_handle_rx(uint16_t src, const uint8_t* p, size_t n) {
  if (n < 1 || !ft_is_msg(p[0])) return false;
  Serial.printf("[FT] Handling msg type=%u from %04X (len=%u)\n", p[0], src, n);

  switch (p[0]) {
    case FT_META: {
      if (n >= sizeof(FtMeta)) {
#if NODE_IS_GATEWAY
        ft_gw_onMeta(src, (const FtMeta*)p, n);
#else
        Serial.println("[CAM] unexpected META");
#endif
      } else {
        Serial.printf("[FT] META short from %X (n=%u)\n", src, (unsigned)n);
      }
      return true;
    }

    case FT_CHUNK: {
#if NODE_IS_GATEWAY
      ft_gw_onChunk(src, (const FtChunk*)p, n);
#else
      Serial.println("[CAM] unexpected CHUNK");
#endif
      return true;
    }

    case FT_END: {
#if NODE_IS_GATEWAY
      ft_gw_onEnd(src, (const FtEnd*)p, n);
#else
      Serial.println("[CAM] unexpected END");
#endif
      return true;
    }

    case FT_ACK: {
      if (n < sizeof(FtAck)) return false;
#if !NODE_IS_GATEWAY
      ft_cam_onAck((const FtAck*)p);
#endif
      return true;
    }

    default:
      Serial.printf("[FT] type %u not handled in this step\n", p[0]);
      return true;
  }
}


