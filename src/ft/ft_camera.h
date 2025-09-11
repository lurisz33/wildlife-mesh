#pragma once
#include <stdint.h>
#include "ft_protocol.h"

bool ft_cam_handshakeMeta(const char* relName, uint32_t& fileId, uint32_t& startOffset);
void ft_cam_onAck(const FtAck* a);
bool ft_cam_sendFile(const char* relName, uint32_t fileId, uint32_t startOffset);