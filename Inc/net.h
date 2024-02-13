#ifndef NET_H
#define NET_H

#ifdef __cplusplus
extern "C" {
#endif
//-----------------------------------------------
//-----------------------------------------------
void udp_client_connect(void);
void TIM1_Callback(void);
void udp_client_send(void);
void udp_lcm_connect(void);
void transmit_servo_state(void);
void distribute_vb_frame( uint8_t * vb_frame );
void wake_on_lan( uint8_t * mac_addr );
//-----------------------------------------------
#ifdef __cplusplus
}
#endif

#endif /* NET_H */
