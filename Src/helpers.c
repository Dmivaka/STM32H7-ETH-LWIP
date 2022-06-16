#include "helpers.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

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

/*
  * @brief Decodes FDCAN_data_length_code into the decimal length of FDCAN message
  * @param[in]          length           FDCAN_data_length_code
  * @retval             uint8_t         Decimal message length (bytes)
*/
uint8_t LengthDecoder( uint32_t length )
{
  switch( length )
  {
    case FDCAN_DLC_BYTES_0:     return 0;
    case FDCAN_DLC_BYTES_1:     return 1;
    case FDCAN_DLC_BYTES_2:     return 2;
    case FDCAN_DLC_BYTES_3:     return 3;
    case FDCAN_DLC_BYTES_4:     return 4;
    case FDCAN_DLC_BYTES_5:     return 5;
    case FDCAN_DLC_BYTES_6:     return 6;
    case FDCAN_DLC_BYTES_7:     return 7;
    case FDCAN_DLC_BYTES_8:     return 8;
    case FDCAN_DLC_BYTES_12:    return 12;
    case FDCAN_DLC_BYTES_16:    return 16;
    case FDCAN_DLC_BYTES_20:    return 20;
    case FDCAN_DLC_BYTES_24:    return 24;
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
  switch( length )
  {
    case 0:     return FDCAN_DLC_BYTES_0;
    case 1:     return FDCAN_DLC_BYTES_1;
    case 2:     return FDCAN_DLC_BYTES_2;
    case 3:     return FDCAN_DLC_BYTES_3;
    case 4:     return FDCAN_DLC_BYTES_4;
    case 5:     return FDCAN_DLC_BYTES_5;
    case 6:     return FDCAN_DLC_BYTES_6;
    case 7:     return FDCAN_DLC_BYTES_7;
    case 8:     return FDCAN_DLC_BYTES_8;
    case 12:    return FDCAN_DLC_BYTES_12;
    case 16:    return FDCAN_DLC_BYTES_16;
    case 20:    return FDCAN_DLC_BYTES_20;
    case 24:    return FDCAN_DLC_BYTES_24;
    case 32:    return FDCAN_DLC_BYTES_32;
    case 48:    return FDCAN_DLC_BYTES_48;
    case 64:    return FDCAN_DLC_BYTES_64;
      
    default:
      while(1); //error
  }
}