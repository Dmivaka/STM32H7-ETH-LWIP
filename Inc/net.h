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

uint8_t LengthDecoder( uint32_t length );
uint32_t LengthCoder( uint8_t length );

uint8_t write_can_frame(buffer_instance * s, FDCAN_RxHeaderTypeDef * head, uint8_t *data);
uint8_t read_can_frame(buffer_instance * s, FDCAN_TxHeaderTypeDef * head, uint8_t *data);
//-----------------------------------------------
#ifdef __cplusplus
}
#endif

#endif /* NET_H */
