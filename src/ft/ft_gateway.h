#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ft_protocol.h"

void ft_gw_setup();

void ft_gw_onMeta (uint16_t src, const FtMeta*  m, size_t n);
void ft_gw_onChunk(uint16_t src, const FtChunk* c, size_t n);
void ft_gw_onEnd  (uint16_t src, const FtEnd*   e, size_t n);