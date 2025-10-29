#pragma once
#include <stdint.h>
#include "ft_protocol.h"

void ft_cam_onAck(const FtAck* a);
bool ft_cam_handshakeMeta(const char* relName,
                          uint32_t& fileId,
                          uint32_t& startOffset,
                          uint16_t gwAddr);

bool ft_cam_sendFile(const char* relName,
                     uint32_t fileId,
                     uint32_t startOffset,
                     uint16_t gwAddr);