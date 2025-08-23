#pragma once
#include <stdint.h>
#include <stddef.h>

static inline uint32_t crc32_update(uint32_t crc, uint8_t byte) {
  crc ^= byte;
  for (int i = 0; i < 8; i++) {
    crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
  }
  return crc;
}

static inline uint32_t crc32_buf(const uint8_t* data, size_t len, uint32_t crc = 0xFFFFFFFFu) {
  for (size_t i = 0; i < len; i++) {
    crc = crc32_update(crc, data[i]);
  }
  return crc ^ 0xFFFFFFFFu;
}

template<typename T>
static inline uint32_t crc32_pod(uint32_t crc, const T& v) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
  for (size_t i = 0; i < sizeof(T); ++i) crc = crc32_update(crc, p[i]);
  return crc;
}
