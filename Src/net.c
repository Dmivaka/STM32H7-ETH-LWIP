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
#include "lwip/igmp.h"

#include "stm32h7xx_ll_spi.h"

#include "lcmlite.h"

#include "servo_state_msg.h"
#include "servo_cmd_msg.h"

#include "circular_heap.h"
//-----------------------------------------------
struct udp_pcb *upcb;

lcmlite_t lcm;

#define LOCAL_PORT 1555
#define REMOTE_PORT 1556
uint8_t RMT_IP_ADDRESS[4] = {10,127,0,1};

struct udp_pcb *upcb_1;

extern FDCAN_HandleTypeDef * FDCAN_Handles_Map[3];

char hl_command_charname[] = "SERVO_CMD";

uint8_t LCM_rx_flag = 0;
uint8_t LCM_tx_flag = 0;

//-----------------------------------------------
static inline uint32_t word_reverse(uint32_t i)
{
  uint32_t res = 0;
  __asm("REV %[result], %[input_i]": [result] "=r" (res): [input_i] "r" (i));
  return res;
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

servo_cmd_msg rx_lcm_msg = {0};
uint64_t servo_cmd_hash = 0xc57d054638269a59;

//#pragma optimize s=none
static void hl_command_callback(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user)
{
  // the first 64 bits of provided buffer contain hash number,
  // which is unique for the message type (see LCM generated headers)
  uint64_t buf_msg_hash = *(uint64_t*)buf;
  
  if( buf_msg_hash != servo_cmd_hash )
  {
    Error_Handler();
  }
  
  // copy the data from the provided buffer into the global LCM ctucture
  // reverse data from the big-endian into small-endian simultaneously
  for( int i = 0; i < sizeof(rx_lcm_msg)/4; i++)
  {
    uint32_t var = *((uint32_t*)buf + 2 + i); // read word from the input buffer
    *((uint32_t*)&rx_lcm_msg + i) = word_reverse(var); // write reversed word into the output buffer
  }
  
  LCM_rx_flag = 1;
  
  return ;
}

servo_state_msg tx_lcm_msg = {0};
uint8_t lcm_tx_buf[512] = {0};
uint64_t servo_state_hash = 0xf006a2fd1ea7342f;

void transmit_servo_state(void)
{
  /*
  for( int i = 0; i < 12; i++ )
  {
    tx_lcm_msg.position[i] = i;
    tx_lcm_msg.velocity[i] = i;
    tx_lcm_msg.torque[i] = i;
  }
  */
  *(uint64_t*)lcm_tx_buf = servo_state_hash;  // write 8-byte long servo_state_hash into the 0-7 bytes of array

  for( int i = 0; i < sizeof(tx_lcm_msg)/4; i++)
  {
    uint32_t var = *((uint32_t*)&tx_lcm_msg + (int)i); // read word from the input buffer
    *((uint32_t*)&lcm_tx_buf + 2 + i) = word_reverse(var); // write reversed word into the output buffer
  }
  
  lcmlite_publish(&lcm, "SERVO_STATE", &lcm_tx_buf, sizeof(tx_lcm_msg) + 8);
}

ip_addr_t Multicast_Addr;
void transmit_LCM_packet(const void *_buf, int buf_len, void *user)
{
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, buf_len, PBUF_RAM); // allocate LWIP memory for outgoing UDP packet

  if (p != NULL)
  {
    pbuf_take(p, (void *) _buf, buf_len);
    udp_sendto(upcb_1, p, &Multicast_Addr, 1557);
    pbuf_free(p);
  }
  else
  {
    Error_Handler();
  }
  
  return ;
}

//-----------------------------------------------
void lcm_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  lcmlite_receive_packet( &lcm, p->payload, p->len, NULL);
  pbuf_free(p);
  return ;
}
//-----------------------------------------------
void udp_lcm_connect(void)
{
  IP4_ADDR(&Multicast_Addr, 224, 0, 0, 7 ); 
#if LWIP_IGMP
    igmp_joingroup(IP_ADDR_ANY,&Multicast_Addr);
#endif
    
  err_t err;
  upcb_1 = udp_new();
  if (upcb_1!=NULL)
  {
    upcb_1->local_port = LOCAL_PORT;
    err= udp_bind(upcb_1, IP_ADDR_ANY, 1557);
    if (err == ERR_OK)
    {
      udp_recv(upcb_1, lcm_receive_callback, NULL);
    }
  }
  
  lcmlite_init(&lcm, transmit_LCM_packet, NULL);
  
  // subscribe to HL_COMMAND messages
  lcmlite_subscription_t *sub = malloc(sizeof(lcmlite_subscription_t));
  sub->channel = hl_command_charname;
  sub->callback = hl_command_callback;
  sub->user = NULL;
  lcmlite_subscribe(&lcm, sub);
}
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
extern queue spi_tx_queue;
extern circular_heap_t spi_tx_heap;

extern queue *can_tx_queues[3];
extern circular_heap_t *can_tx_heaps[3];

volatile uint32_t debug_counter = 0;

// this function serves as abstraction layer between application-level can-buses
// and actual can-hardware on the main or companion chip. 
void distribute_vb_frame( uint8_t * vb_frame )
{
  uint8_t bus;
  uint32_t id;
  size_t frame_len;

  uint8_t *data_pointer = deserialize_can_frame( &bus, &id, &frame_len, NULL, vb_frame); // only extracts service info

  if( bus < 3 )
  {
    FDCAN_HandleTypeDef * FDCAN_Handle = FDCAN_Handles_Map[bus];

    if( FDCAN_Handle != NULL )
    {
      if( HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_Handle) > 0 )
      {
        // if interrupt fires right now it will try access TX_buffer in line below
        push_can_frame( FDCAN_Handle, id, frame_len, data_pointer);
      }
      else
      {
        // this section should be critical
        uint32_t primask_bit = __get_PRIMASK();       // backup PRIMASK bit
        __disable_irq();                              // Disable all interrupts by setting PRIMASK bit on Cortex
        
          item* new_element = circular_heap_alloc( can_tx_heaps[bus], sizeof(void*) + frame_len + 5 );
          
          if( new_element != NULL )
          {
            memcpy( &new_element->payload, vb_frame, frame_len + 5 );
            enqueue( can_tx_queues[bus], new_element );
          }
          else
          {
            // buffer is full
          }

        __set_PRIMASK(primask_bit);                   // Restore PRIMASK bit
      }
    }
    else
    {
      // this bus is disabled
    }
  }
  else
  {
    debug_counter++;
    // SPI transfer section. 
    // There's at least one frame ready to be processed. 

    uint32_t primask_bit = __get_PRIMASK();   // backup PRIMASK bit
    __disable_irq();                          // Disable all interrupts by setting PRIMASK bit on Cortex
    // while we are accessing the buffer the SPI interrupt can occur - it will corrupt it. 

    item* new_element = circular_heap_alloc( &spi_tx_heap, sizeof(void*) + frame_len + 5 );
    if( new_element != NULL )
    {
      memcpy( &new_element->payload, vb_frame, frame_len + 5 );
      enqueue( &spi_tx_queue, new_element );
    }
    else
    {
      // buffer is full
      Error_Handler();
    }
    
    if( !LL_SPI_IsActiveMasterTransfer(SPI4) )
    {
      send_frame_SPI();
    }

    __set_PRIMASK(primask_bit);               // Restore PRIMASK bit
  }
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  uint8_t * data = p->payload; // pointer to the dynamically allocated UDP packet payload buffer. safe to use untill buffer is freed. 
  uint16_t packet_length = p->len;

  uint16_t index = 0; // specifies number of bytes read from the udp packet. 
  
  while( index < packet_length ) // iterate over UDP packet
  {
    // parse one vb frame from the rx UDP packet
    uint8_t vb_frame_length = data[index];

    distribute_vb_frame( &data[index] );

    index += vb_frame_length;
  }
  
  if( index != packet_length )
  {
    Error_Handler();
  }
  
  pbuf_free(p);
}

void wake_on_lan( uint8_t * mac_addr )
{
  uint8_t my_packet[102] = {0};
  
  for( int i = 0; i < 6; i++ )
  {
    my_packet[i] = 0xFF;
  }
  
  for( int i = 6; i < 102; i += 6 )
  {
    memcpy( &my_packet[i], mac_addr, 6 );
  }
  
  ip_addr_t Broadcast_Addr;
  IP4_ADDR(&Broadcast_Addr, 255, 255, 255, 255 ); 
  
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(my_packet), PBUF_RAM); // allocate LWIP memory for outgoing UDP packet
  
  pbuf_take(p, (void *) my_packet, sizeof(my_packet));
  udp_sendto(upcb_1, p, &Broadcast_Addr, 0);
  pbuf_free(p);
}