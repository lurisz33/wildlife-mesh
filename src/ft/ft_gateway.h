#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ft_protocol.h"
#include "ft_common.h"

void ft_gw_setup();

void ft_gw_onMeta (uint16_t src, const FtMeta*  m, size_t n);
void ft_gw_onChunk(uint16_t src, const FtChunk* c, size_t n);
void ft_gw_onEnd  (uint16_t src, const FtEnd*   e, size_t n);

void ft_gw_onImageHeader(uint16_t src, const ImageHeader* header);
void ft_gw_onImageData(uint16_t src, const uint8_t* data, size_t len);
bool ft_gw_hasActiveImageTransfer(uint16_t src);