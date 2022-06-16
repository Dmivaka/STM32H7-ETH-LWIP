#include "main.h"
#include "stm32h7xx_hal_fdcan.h"
#include "net.h"
#include "helpers.h"



#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "lwip.h"
#include "lwip/udp.h"
//-----------------------------------------------
struct udp_pcb *upcb;
char stringer[78];

#define LOCAL_PORT 1555
#define REMOTE_PORT 1556
uint8_t RMT_IP_ADDRESS[4] = {10,127,0,0};

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
//-----------------------------------------------
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
//-----------------------------------------------
uint8_t write_can_frame(buffer_instance * s, FDCAN_RxHeaderTypeDef * head, uint8_t *data)
{
  uint8_t can_message_length = LengthDecoder( head->DataLength ) + 5; // together with service bytes
  
  write_buffer(s, &can_message_length, 1); // write message length
  
  can_message_length -= 5; // 25 bytes with the service info
  
  uint8_t bus = 1;
  uint32_t id = head->Identifier;
  uint32_t bus_id = encode_bus_id( bus, id );

  write_buffer(s, (uint8_t*)&bus_id, 4); // write message length
  write_buffer(s, data, can_message_length); // write message length
  
  return 0;
}

uint8_t read_can_frame(buffer_instance * s, FDCAN_TxHeaderTypeDef * head, uint8_t *data)
{
  uint8_t can_message_length = 0;
  uint32_t bus_id = 0;
  uint8_t bus = 0;
  uint32_t id = 0;
  
  read_buffer(s, &can_message_length, 1);
  can_message_length -= 5;
  read_buffer(s, (uint8_t *)&bus_id, 4);
  read_buffer(s, data, can_message_length);

  head->DataLength = LengthCoder( can_message_length );
  head->Identifier = decode_can_id( bus_id );
  // process can frame
  
  return 0;
}
//-----------------------------------------------
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
//-----------------------------------------------
void udp_client_connect(void)
{
  ip_addr_t DestIPaddr;
  err_t err;
  upcb = udp_new();
  if (upcb!=NULL)
  {
    IP4_ADDR(&DestIPaddr, RMT_IP_ADDRESS[0], RMT_IP_ADDRESS[1], RMT_IP_ADDRESS[2], RMT_IP_ADDRESS[3]);
    upcb->local_port = LOCAL_PORT;
    err= udp_connect(upcb, &DestIPaddr, REMOTE_PORT);
    if (err == ERR_OK)
    {
      udp_recv(upcb, udp_receive_callback, NULL);
    }
  }
}
//-----------------------------------------------
extern buffer_instance gaga;

void udp_client_send(void)
{
  struct pbuf *p;
  //sprintf(str1,"%lu\r\n",HAL_GetTick());
  //p = pbuf_alloc(PBUF_TRANSPORT, strlen(str1), PBUF_RAM);
  
  p = pbuf_alloc(PBUF_TRANSPORT, gaga.bytes_written, PBUF_RAM);
  
  // 
  if( gaga.bytes_written < buf_size - gaga.head )
  {
    // memcpy in one step
    pbuf_take(p, (void *) &gaga.buffer_body[gaga.head], gaga.bytes_written);
  }
  else
  {
    // memcpy in two steps
    pbuf_take(p, (void *) &gaga.buffer_body[gaga.head], buf_size - gaga.head);
    pbuf_take_at(p, (void *) gaga.buffer_body, gaga.tail, buf_size - gaga.head);
  }
  
  if (p != NULL)
  {
    //pbuf_take(p, (void *) str1, strlen(str1));
    udp_send(upcb, p);
    pbuf_free(p);
    gaga.bytes_written -= gaga.bytes_written;
    gaga.head = gaga.tail;
  }
}
//-----------------------------------------------
uint8_t buffer[128] = {0};

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  uint8_t * data = p->payload; // pointer to the dynamically allocated UDP packet payload buffer. safe to use untill buffer is freed. 
  uint8_t packet_length = p->len;
  
  memcpy(&buffer, data, 128);
  
  uint16_t index = 0; // specifies number of bytes read from the udp packet. 
  
  while( index < packet_length ) // iterate over UDP packet
  {
    uint32_t bus_id = 0;
    uint8_t * frame_payload;

    uint8_t can_message_length = data[index] - 5;
    index += 1;
    memcpy(&bus_id, &data[index], 4);
    index += 4;
    frame_payload = &data[index];
    index += can_message_length;
    
    uint8_t bus_num = decode_bus_num( bus_id );

    FDCAN_HandleTypeDef *hfdcan_ptr;
    
    if( bus_num == 1 )
    {
      hfdcan_ptr = &hfdcan1;
    }
    else if( bus_num == 2 )
    {
      hfdcan_ptr = &hfdcan2;
    }
    else if( bus_num == 3 )
    {
      hfdcan_ptr = &hfdcan3;
    }
    else
    {
      Error_Handler();
    }
    
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan_ptr) != 0)
    {
      FDCAN_TxHeaderTypeDef TxHeader;
      // Add message to Tx FIFO 
      TxHeader.Identifier = decode_can_id( bus_id );
      TxHeader.IdType = FDCAN_STANDARD_ID;
      TxHeader.TxFrameType = FDCAN_DATA_FRAME;
      TxHeader.DataLength = LengthCoder( can_message_length );
      TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
      TxHeader.BitRateSwitch = FDCAN_BRS_ON;
      TxHeader.FDFormat = FDCAN_FD_CAN;
      TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
      TxHeader.MessageMarker = 0x00;
      if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_ptr, &TxHeader, frame_payload) != HAL_OK)
      {
        Error_Handler();
      }
    }
    else
    {
      Error_Handler();
    }
  }
  
  if( index != p->len )
  {
    Error_Handler();
  }
  
  pbuf_free(p);
}
//-----------------------------------------------
void TIM1_Callback(void)
{
  udp_client_send();
}
//--------------------------------------------------
