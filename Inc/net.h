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
//-----------------------------------------------
#ifdef __cplusplus
}
#endif

#endif /* NET_H */
