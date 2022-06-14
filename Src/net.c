#include "main.h"
#include "net.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "lwip.h"
#include "lwip/udp.h"
//-----------------------------------------------
struct udp_pcb *upcb;
char str1[30];

#define LOCAL_PORT 1555
#define REMOTE_PORT 1556
uint8_t RMT_IP_ADDRESS[4] = {10,127,0,2};
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
void udp_client_send(void)
{
  struct pbuf *p;
  sprintf(str1,"%lu\r\n",HAL_GetTick());
  p = pbuf_alloc(PBUF_TRANSPORT, strlen(str1), PBUF_RAM);
  if (p != NULL)
  {
    pbuf_take(p, (void *) str1, strlen(str1));
    udp_send(upcb, p);
    pbuf_free(p);
  }
}
//-----------------------------------------------
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  strncpy(str1,p->payload,p->len);
  str1[p->len]=0;
  pbuf_free(p);
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}
//-----------------------------------------------
void TIM1_Callback(void)
{
	udp_client_send();
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}
//--------------------------------------------------
