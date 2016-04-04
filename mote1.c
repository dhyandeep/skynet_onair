#include "contiki.h"
#include "net/rime.h"
#include "net/rime/mesh.h"
#include "dev/button-sensor.h"
#include <math.h>

#include <stdio.h>

#define N 8
#define LENGTH 8


float result1[64],result2[64],inverseres[64];
float image1[64],image2[64];

int currentnum=0,val=0,counter=0;
unsigned char c;


static inline void fdct_1d(float *dst, const float *src,
                           int dst_stridea, int dst_strideb,
                           int src_stridea, int src_strideb)
{
    int i;

    for (i = 0; i < N; i++) {
        const float x00 = src[0*src_stridea] + src[7*src_stridea];
        const float x01 = src[1*src_stridea] + src[6*src_stridea];
        const float x02 = src[2*src_stridea] + src[5*src_stridea];
        const float x03 = src[3*src_stridea] + src[4*src_stridea];
        const float x04 = src[0*src_stridea] - src[7*src_stridea];
        const float x05 = src[1*src_stridea] - src[6*src_stridea];
        const float x06 = src[2*src_stridea] - src[5*src_stridea];
        const float x07 = src[3*src_stridea] - src[4*src_stridea];
        const float x08 = x00 + x03;
        const float x09 = x01 + x02;
        const float x0a = x00 - x03;
        const float x0b = x01 - x02;
        const float x0c = 1.38703984532215*x04 + 0.275899379282943*x07;
        const float x0d = 1.17587560241936*x05 + 0.785694958387102*x06;
        const float x0e = -0.785694958387102*x05 + 1.17587560241936*x06;
        const float x0f = 0.275899379282943*x04 - 1.38703984532215*x07;
        const float x10 = 0.353553390593274 * (x0c - x0d);
        const float x11 = 0.353553390593274 * (x0e - x0f);
        dst[0*dst_stridea] = 0.353553390593274 * (x08 + x09);
        dst[1*dst_stridea] = 0.353553390593274 * (x0c + x0d);
        dst[2*dst_stridea] = 0.461939766255643*x0a + 0.191341716182545*x0b;
        dst[3*dst_stridea] = 0.707106781186547 * (x10 - x11);
        dst[4*dst_stridea] = 0.353553390593274 * (x08 - x09);
        dst[5*dst_stridea] = 0.707106781186547 * (x10 + x11);
        dst[6*dst_stridea] = 0.191341716182545*x0a - 0.461939766255643*x0b;
        dst[7*dst_stridea] = 0.353553390593274 * (x0e + x0f);
        dst += dst_strideb;
        src += src_strideb;
    }
}

static void fdct(float *dst, const float *src)
{
    float tmp[N*N];
    fdct_1d(tmp, src, 1, N, 1, N);
    fdct_1d(dst, tmp, N, 1, N, 1);
}

static inline void idct_1d(float *dst, const float *src,
                           int dst_stridea, int dst_strideb,
                           int src_stridea, int src_strideb)
{
    int i;

    for (i = 0; i < N; i++) {
        const float x00 = 1.4142135623731*src[0*src_stridea];
        const float x01 = 1.38703984532215*src[1*src_stridea] + 0.275899379282943*src[7*src_stridea];
        const float x02 = 1.30656296487638*src[2*src_stridea] + 0.541196100146197*src[6*src_stridea];
        const float x03 = 1.17587560241936*src[3*src_stridea] + 0.785694958387102*src[5*src_stridea];
        const float x04 = 1.4142135623731*src[4*src_stridea];
        const float x05 = -0.785694958387102*src[3*src_stridea] + 1.17587560241936*src[5*src_stridea];
        const float x06 = 0.541196100146197*src[2*src_stridea] - 1.30656296487638*src[6*src_stridea];
        const float x07 = -0.275899379282943*src[1*src_stridea] + 1.38703984532215*src[7*src_stridea];
        const float x09 = x00 + x04;
        const float x0a = x01 + x03;
        const float x0b = 1.4142135623731*x02;
        const float x0c = x00 - x04;
        const float x0d = x01 - x03;
        const float x0e = 0.353553390593274 * (x09 - x0b);
        const float x0f = 0.353553390593274 * (x0c + x0d);
        const float x10 = 0.353553390593274 * (x0c - x0d);
        const float x11 = 1.4142135623731*x06;
        const float x12 = x05 + x07;
        const float x13 = x05 - x07;
        const float x14 = 0.353553390593274 * (x11 + x12);
        const float x15 = 0.353553390593274 * (x11 - x12);
        const float x16 = 0.5*x13;
        const float x08 = -x15;
        dst[0*dst_stridea] = 0.25 * (x09 + x0b) + 0.353553390593274*x0a;
        dst[1*dst_stridea] = 0.707106781186547 * (x0f - x08);
        dst[2*dst_stridea] = 0.707106781186547 * (x0f + x08);
        dst[3*dst_stridea] = 0.707106781186547 * (x0e + x16);
        dst[4*dst_stridea] = 0.707106781186547 * (x0e - x16);
        dst[5*dst_stridea] = 0.707106781186547 * (x10 - x14);
        dst[6*dst_stridea] = 0.707106781186547 * (x10 + x14);
        dst[7*dst_stridea] = 0.25 * (x09 + x0b) - 0.353553390593274*x0a;
        dst += dst_strideb;
        src += src_strideb;
    }
}

static void idct(float *dst, const float *src)
{
    float tmp[N*N];
    idct_1d(tmp, src, 1, N, 1, N);
    idct_1d(dst, tmp, N, 1, N, 1);
}




int prevroot=-1;

static struct mesh_conn mesh;

 /*---------------------------------------------------------------------------*/
PROCESS(example_mesh_process, "Mesh example");
AUTOSTART_PROCESSES(&example_mesh_process);

 /*---------------------------------------------------------------------------*/
static void sent(struct mesh_conn *c)
{
    static int sent=0;
	printf("packet sent, sent=%d\n", ++sent);
	if(sent  == 100) {
       printf("tx completed\n");
	}
}
static void timedout(struct mesh_conn *c)
{
    static int timeout=0;
	printf("packet timedout, timeout=%d\n", ++timeout);
}
static void recv(struct mesh_conn *c, const rimeaddr_t *from, uint8_t hops)
{
    static int recv=0;
    int i=0;
    uint8_t *data=packetbuf_dataptr();
   uint8_t root=from->u8[0];

   if(prevroot==-1)
   {
	   int k=1;
	   prevroot=root;
	   for(i=0;i<64;i++)
	   {
				image1[i]=data[k];
				printf("%d ",(int)image1[i]);
				k++;
			
	   }
	   fdct(result1,image1);
	  
	  
   }
   else if(prevroot!=root)
   {
	    int k=1;
	   
	   for(i=0;i<64;i++)
	   {
		  
				image2[i]=data[k];
				printf("%d ",(int)image2[i]);
				k++;
	   }
	   fdct(result2,image2);
	   for (i=0;i<64;i++)
		{
			result1[i]=(result1[i]+result2[i])/2;
		}
		idct(inverseres,result1);
		printf("\n");
		for (i=0;i<N*N;i++)
		{
			printf("%d ",(int)inverseres[i]);
		}
   }
   
 printf("from: %d",root);
	printf("Data received from %d.%d: %.*s (%d), recv=%d\n",from->u8[0], from->u8[1],packetbuf_datalen(), (char *)packetbuf_dataptr(), packetbuf_datalen(),++recv);
	
}
const static struct mesh_callbacks callbacks = {recv, sent, timedout};
 /*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_mesh_process, ev, data)
{
	static struct etimer et;
	PROCESS_EXITHANDLER(mesh_close(&mesh);)
	PROCESS_BEGIN();
	mesh_open(&mesh, 132, &callbacks);
	SENSORS_ACTIVATE(button_sensor);
	etimer_set(&et, CLOCK_SECOND * 10);
	while(1) {
     rimeaddr_t addr;
     PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);
	 uint8_t arr[65];
	 int i;
	 for(i=1;i<65;i++)
	 arr[i]=i-1;
	 arr[0]=0;
     static int tx=0;
     packetbuf_copyfrom(arr,65);
     addr.u8[0] = 1;
     addr.u8[1] = 0;
     if(!rimeaddr_cmp(&rimeaddr_node_addr, &addr)) {
         int res = mesh_send(&mesh, &addr);
         if ( res > 0 ) {
             printf("mesh sending,res=%d,tx=%d\n",res,  ++tx);
             etimer_set(&et, CLOCK_SECOND * 4);
         }
         else {
             printf("mesh sending,res=%d,tx=%d\n",res,  ++tx);
             etimer_set(&et, CLOCK_SECOND * 15);
         }
     }
   }
   PROCESS_END();
 }
