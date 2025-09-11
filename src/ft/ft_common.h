#pragma once
#include "LoraMesher.h"

#ifndef NODE_IS_GATEWAY
#define NODE_IS_GATEWAY 0   
#endif

#define GATEWAY_ADDR 0xE04C 
#define CAMERA_ADDR  0x545C 

extern LoraMesher& radio;