#include "net.h"
//-----------------------------------------------
struct udp_pcb *upcb;
char str1[30];
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
  	IP4_ADDR(&DestIPaddr, 10, 127, 0, 0);
  	upcb->local_port = 1555;
  	err= udp_connect(upcb, &DestIPaddr, 1556);
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
