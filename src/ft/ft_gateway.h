#pragma once
#include <Arduino.h>
#include <FS.h>
#include "ft_protocol.h"
#include "ft_common.h"

extern const char FT_RX_BASE[];   

void ft_gw_setup();
void ft_gw_onMeta(uint16_t src, const FtMeta* m);
void ft_gw_onChunk(uint16_t src, const FtChunk* c);
void ft_gw_onEnd(uint16_t src, const FtEnd* e);
