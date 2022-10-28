#include "main.h"
#include "stm32h7xx_hal.h"
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
uint8_t RMT_IP_ADDRESS[4] = {192,168,2,105};

extern FDCAN_HandleTypeDef * FDCAN_Handles_Map[3];
extern buffer_instance * TX_Buffers_Map[3];
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
uint8_t debug_buf[128] = {0};

void udp_client_send(void)
{
  // make a local copy of buffer variables to allow concurrent write access to the global buffer
  uint16_t bytes = gaga.bytes_written;
  uint16_t head0 = gaga.head;
  uint16_t tail0 = gaga.tail;

  if( bytes == 0 )
  {
    // no data to process
    return ; 
  }
  
  memset(debug_buf, 0, 128);

  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, bytes, PBUF_RAM); // allocate LWIP memory for outgoing UDP packet
  
  memcpy(debug_buf, p->payload, 128);
    
  if (p != NULL)
  {
    if( bytes < buf_size - head0 )
    {
      // memcpy in one step - written data gapless
      pbuf_take(p, (void *) &gaga.buffer_body[head0], bytes);
      memcpy(debug_buf, p->payload, 128);
    }
    else
    {
      // memcpy in two steps - data is divided in two parts - at the end and the beginning of circular buffer
      pbuf_take(p, (void *) &gaga.buffer_body[head0], buf_size - head0);
      pbuf_take_at(p, (void *) gaga.buffer_body, tail0, buf_size - head0);
      memcpy(debug_buf, p->payload, 128);
    }
    
    memcpy(debug_buf, p->payload, 128);

    udp_send(upcb, p);
    pbuf_free(p);
    gaga.bytes_written -= bytes;
    gaga.head = tail0;
  }
  else
  {
    // ran out of memory? 
    Error_Handler();
  }
}


uint8_t companion_TX_buf[buf_size] = {0};
buffer_instance companion_TX_ins = {0, NULL, 0, companion_TX_buf};

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  uint8_t * data = p->payload; // pointer to the dynamically allocated UDP packet payload buffer. safe to use untill buffer is freed. 
  uint16_t packet_length = p->len;

  uint16_t index = 0; // specifies number of bytes read from the udp packet. 
  
  while( index < packet_length ) // iterate over UDP packet
  {
    uint32_t bus_id = 0;

    // parse one can frame from the rx UDP packet
    uint8_t can_frame_length = data[index];
    memcpy(&bus_id, &data[index+1], 4);
    uint8_t bus_num = decode_bus_num( bus_id );
    
    if( bus_num < 3 )
    {
      FDCAN_HandleTypeDef * FDCAN_Handle = FDCAN_Handles_Map[bus_num];
      buffer_instance *TX_buffer = TX_Buffers_Map[bus_num];
      
      if( FDCAN_Handle != NULL )
      {    
        if( HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_Handle) > 0 )
        {
          push_can_frame( FDCAN_Handle, &data[index], can_frame_length);
        }
        else
        {
          write_buffer(TX_buffer, &data[index], can_frame_length); // write message length
        }
      }
      else
      {
        // this bus is disabled
      }
    }
    else
    {
      write_buffer(&companion_TX_ins, &data[index], can_frame_length); // write message length
    }

    index += can_frame_length;
  }
  
  if( index != packet_length )
  {
    Error_Handler();
  }
  
  pbuf_free(p);
}
//-----------------------------------------------
void TIM1_Callback(void)
{
  //udp_client_send();
}
//--------------------------------------------------
