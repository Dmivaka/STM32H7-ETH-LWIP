#include "helpers.h"

uint8_t decode_bus_num( uint32_t encoded_bus_num )
{
  return encoded_bus_num >> 30;
}

uint32_t decode_can_id( uint32_t encoded_can_id )
{
  return (encoded_can_id & 0x1FFFFFFF );
}

uint32_t encode_bus_id( uint8_t bus_num, uint32_t can_id )
{
  uint32_t bus_id = can_id;
  bus_id = ((bus_id & 0x1FFFFFFF) | (bus_num << 30));

  return bus_id;
}
