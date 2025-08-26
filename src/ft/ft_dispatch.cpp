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
        const FtMeta* m = (const FtMeta*)p;
        ft_gw_onMeta(src, m);
#else
        Serial.println("[CAM] unexpected META");
#endif
      } else {
        Serial.printf("[FT] META short from %X\n", src);
      }
      return true;
    }

    case FT_ACK: {
    if (n < sizeof(FtAck)) return false;
    const FtAck* a = (const FtAck*)p;
    
    Serial.printf("[FT] Got ACK from %04X: id=%08X next=%u\n",
                  src, a->fileId, a->nextOffset);
    
    #if !NODE_IS_GATEWAY
        ft_cam_onAck(a); 
    #endif
    
    return true;
    } 

    default:
      Serial.printf("[FT] type %u not handled in this step\n", p[0]);
      return true;
  }
}

