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
uint8_t RMT_IP_ADDRESS[4] = {10,127,0,0};

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
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
  // make a local copy of buffer variables to allow concurrent write access to the global buffer
  uint16_t bytes = gaga.bytes_written;
  uint16_t head0 = gaga.head;
  uint16_t tail0 = gaga.tail;

  if( bytes == 0 )
  {
    // no data to process
    return ; 
  }

  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, bytes, PBUF_RAM); // allocate LWIP memory for outgoing UDP packet
  
  if (p != NULL)
  {
    if( bytes < buf_size - head0 )
    {
      // memcpy in one step - written data gapless
      pbuf_take(p, (void *) &gaga.buffer_body[head0], bytes);
    }
    else
    {
      // memcpy in two steps - data is divided in two parts - at the end and the beginning of circular buffer
      pbuf_take(p, (void *) &gaga.buffer_body[head0], buf_size - head0);
      pbuf_take_at(p, (void *) gaga.buffer_body, tail0, buf_size - head0);
    }

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
//-----------------------------------------------
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  uint8_t * data = p->payload; // pointer to the dynamically allocated UDP packet payload buffer. safe to use untill buffer is freed. 
  uint8_t packet_length = p->len;
  
  //memcpy(&buffer, data, 128);
  
  uint16_t index = 0; // specifies number of bytes read from the udp packet. 
  
  while( index < packet_length ) // iterate over UDP packet
  {
    uint32_t bus_id = 0;

    // parse one can frame from the rx UDP packet
    uint8_t can_message_length = data[index] - 5;
    index += 1;
    memcpy(&bus_id, &data[index], 4);
    index += 4;
    uint8_t * frame_payload = &data[index];
    index += can_message_length;
    
    uint8_t bus_num = decode_bus_num( bus_id );
    uint32_t frame_id = decode_can_id( bus_id );
    
    push_can_frame( bus_num, frame_id, frame_payload, can_message_length);
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
  //udp_client_send();
}
//--------------------------------------------------
