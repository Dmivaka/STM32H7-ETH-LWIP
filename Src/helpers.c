#include "helpers.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

uint8_t decode_bus_num( uint32_t encoded_bus_num )
{
  return encoded_bus_num >> 29;
}

uint32_t decode_can_id( uint32_t encoded_can_id )
{
  return (encoded_can_id & 0x1FFFFFFF );
}

uint32_t encode_bus_id( uint8_t bus_num, uint32_t can_id )
{
  uint32_t bus_id = can_id;
  bus_id = ((bus_id & 0x1FFFFFFF) | (bus_num << 29));

  return bus_id;
}

/*
  * @brief Decodes FDCAN_data_length_code into the decimal length of FDCAN message
  * @param[in]          length           FDCAN_data_length_code
  * @retval             uint8_t         Decimal message length (bytes)
*/
uint8_t LengthDecoder( uint32_t length )
{
  if( length < FDCAN_DLC_BYTES_12 )
  {
    return (uint8_t)(length >> 16);
  }
  else if( length < FDCAN_DLC_BYTES_32 )
  {
    return (uint8_t)((length >> 14) - 24);
  }

  switch( length )
  {
    case FDCAN_DLC_BYTES_32:    return 32;
    case FDCAN_DLC_BYTES_48:    return 48; 
    case FDCAN_DLC_BYTES_64:    return 64;
      
    default:
      while(1); //error
  }
}

/*
  * @brief Codes decimal length of FDCAN message into the FDCAN_data_length_code
  * @param[in]          length              Decimal message length (bytes)
  * @retval             FDCAN_data_length_code        Code of required message length
*/
uint32_t LengthCoder( uint8_t length )
{
  if( length < 12 )
  {
    return (uint32_t)length << 16;
  }
  if( length < 32 )
  {
    return ((uint32_t)length << 14) + 24;
  }

  switch( length )
  {
    case 32:    return FDCAN_DLC_BYTES_32;
    case 48:    return FDCAN_DLC_BYTES_48;
    case 64:    return FDCAN_DLC_BYTES_64;
      
    default:
      while(1); //error
  }
}