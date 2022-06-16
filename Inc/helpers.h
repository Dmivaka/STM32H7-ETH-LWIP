#ifndef HELPERS_H
#define HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint8_t decode_bus_num( uint32_t encoded_bus_num );
uint32_t decode_can_id( uint32_t encoded_can_id );

uint32_t encode_bus_id( uint8_t bus_num, uint32_t can_id );

uint8_t LengthDecoder( uint32_t length );
uint32_t LengthCoder( uint8_t length );

#ifdef __cplusplus
}
#endif

#endif  // this file will only ever be copied in once to another file