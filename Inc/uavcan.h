#ifndef UAVCAN_H_INCLUDED
#define UAVCAN_H_INCLUDED

#include <stdint.h>
#include <stddef.h>

void UAVCAN_setup(void);
void UAVCAN_send(void);
void UAVCAN_request(void);
uint8_t parse_canard_frame( uint32_t id, size_t size, void* payload);
#endif