#ifndef NET_H_
#define NET_H_
//-----------------------------------------------
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "lwip.h"
#include "lwip/udp.h"
//-----------------------------------------------
void udp_client_connect(void);
void TIM1_Callback(void);
//-----------------------------------------------
#endif /* NET_H_ */
