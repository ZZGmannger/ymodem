## How to use?

```c
#include "ymodem.h"

static ym_session_t iap_session;

void iap_rx_callback(uint8_t* buf ,uint16_t len)
{
    /*there you can write to flash*/
    printf("file name: %s\r\n",iap_session.file.name);
    printf("file size: %dBytes\r\n",iap_session.file.size);
    printf("pecent: %d%\r\n",iap_session.file.percent);
}
void iap_tx(uint8_t data)
{
	uart_send(&data ,1);
}

int main(void)
{ 
  HAL_Init();

  static uint32_t evt_tick;
  ym_session_init(&iap_session,  iap_rx_callback , iap_tx);
  ym_session_start(&iap_session);

  while (1)
  {
	  if(!evt_tick)
	  {
		 evt_tick = HAL_GetTick();
	  }
	  if((HAL_GetTick() - evt_tick)>1)
	  {
	  	 evt_tick = 0;
		 ym_session_evt_dispatch();
	  }
  }
}

```

