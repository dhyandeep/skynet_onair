//always use uint_16 to read from buffer



#include "contiki.h"
#include "net/rime/trickle.h"
#include "net/packetbuf.h"
#include "dev/button-sensor.h"

#include "dev/leds.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(example_trickle_process, "Trickle example");
AUTOSTART_PROCESSES(&example_trickle_process);
/*---------------------------------------------------------------------------*/
static void
trickle_recv(struct trickle_conn *c)
{ 
	//packetbuf_set_datalen(64);

	uint8_t *packetbufptr=(uint8_t*)packetbuf_dataptr();
	
	int i;
	int datalen=packetbuf_datalen();

	printf("\n%d.%d: trickle message received %d items",
	rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1],datalen/sizeof(uint8_t));
	for(i=0;i<datalen/sizeof(uint8_t);i++)
	{
		printf("%d ",packetbufptr[i]);
	}
	printf("\n");
}
const static struct trickle_callbacks trickle_call = {trickle_recv};
static struct trickle_conn trickle;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_trickle_process, ev, data)
{
	packetbuf_set_datalen(64);
	PROCESS_EXITHANDLER(trickle_close(&trickle);)
	PROCESS_BEGIN();
	uint8_t arr[100];
	int i;
	arr[0]=arr[1]=arr[2]=arr[3]=arr[4]=arr[6]=0;
	for(i=6;i<100;i++)
	{
		arr[i]=i-6;
	}
	trickle_open(&trickle, CLOCK_SECOND, 145, &trickle_call);
	SENSORS_ACTIVATE(button_sensor);
	i=0;
	while(1) {
		PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event &&data == &button_sensor);	
		packetbuf_copyfrom(arr,64);
		trickle_send(&trickle);
		packetbuf_clear();
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
