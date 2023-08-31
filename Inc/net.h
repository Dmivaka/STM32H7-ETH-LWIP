#ifndef NET_H
#define NET_H

#ifdef __cplusplus
extern "C" {
#endif
  
//-----------------------------------------------
#include "circ_buffer.h"
//-----------------------------------------------
//-----------------------------------------------
void udp_client_connect(void);
void TIM1_Callback(void);
void udp_client_send(void);
void udp_lcm_connect(void);
void transmit_servo_state(void);
void distribute_vb_frame( uint8_t * vb_frame );
//-----------------------------------------------
#ifdef __cplusplus
}
#endif

#endif /* NET_H */
