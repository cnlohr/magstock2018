//Copyright 2015 <>< Charles Lohr Under the MIT/x11 License, NewBSD License or
// ColorChord License.  You Choose.

#include "mem.h"
#include "c_types.h"
#include "user_interface.h"
#include "ets_sys.h"
#include "uart.h"
#include "osapi.h"
#include "espconn.h"
#include "esp82xxutil.h"
#include "ws2812_i2s.h"
#include "hpatimer.h"
#include "DFT32.h"
#include "ccconfig.h"
#include <embeddednf.h>
#include <embeddedout.h>
#include <commonservices.h>
#include "ets_sys.h"
#include "gpio_buttons.h"
#include "gpio.h"
#include "ssid.h"

int in_modem_only_mode;

//#define SWADGE

#define REMOTE_IP_CODE 0x0a00c90a // = 10.201.0.10

#define SERVER_TIMEOUT 1500
#define MAX_CONNS 5
#define MAX_FRAME 2000

#define procTaskPrio        0
#define procTaskQueueLen    2

struct CCSettings CCS;
static volatile os_timer_t some_timer;
static struct espconn * pUdpSender;
static struct espconn  *pUdpServer;


void EnterCritical();
void ExitCritical();

extern volatile uint8_t sounddata[HPABUFFSIZE];
extern volatile uint16_t soundhead;
uint16_t soundtail;

static uint8_t hpa_running = 0;
static uint8_t hpa_is_paused_for_wifi;



int soft_ap_mode = 0;
int udp_pending = 0; //If nonzero, do not attempt to send UDP packet. 
int got_an_ip = 0;
int wifi_fails = 0;

uint32_t force_sleep_time = 0;
int need_to_do_scan = 0;

uint32_t last_ccount;
uint32_t scan_ccount;
int ledqtybytes = 12;
int do_led_push;

uint32_t frame = 0;
uint8_t mymac[6];


uint8_t mypacket[30+200] = {  //256 = max size of additional payload
	0x08, //Frame type, 0x80 = beacon, Tried data, but seems to have been filtered on RX side by other ESP
	0x00, 0x00, 0x00, 
	0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,
	0x00, 0x00,  //Sequence number, cleared by espressif
	0x82, 0x66,	 //"Mysterious OLPC stuff"
	0x82, 0x66, 0x00, 0x00, //????
};
#define PACK_PAYLOAD_START 30
unsigned packet_tx_time; //UNUSED IN THIS FIRMWARE.
#define DISABLE_RAW_PACKET_ACCESS


void ICACHE_FLASH_ATTR CustomStart( );

void ICACHE_FLASH_ATTR user_rf_pre_init()
{
}


//Tasks that happen all the time.
os_event_t    procTaskQueue[procTaskQueueLen];


#ifdef SWADGE




#define UDP_TIMEOUT 500000  //0.5 seconds.

static int make_led_rssi = 0;
static int led_rssi = 0;
static int ledrssi_min;
static int ledrssi_max;
static int ledrssi_intensity;

int rainbow_run;
int rainbow_intensity;
int rainbow_speed;
int rainbow_offset;

int send_back_on_port;
uint32_t send_back_on_ip;


volatile int requested_update = 0;
int disable_push = 0;
int disable_raw_broadcast = 0;
int raw_broadcast_rate = 0x1a;


uint8_t last_button_event_btn;
uint8_t last_button_event_dn;

static uint8_t global_scan_complete = 0;
static uint8_t global_scan_elements = 0;
static uint8_t global_scan_data[1024];  //Where SSID, channel, power go.
int force_channel = 0; //for SoftAP mode.


static int ICACHE_FLASH_ATTR SwitchToSoftAP( int chadvance )
{
	struct softap_config c;
	wifi_softap_get_config_default(&c);
	memcpy( c.ssid, "MAGBADGE_", 9 );
	wifi_get_macaddr(SOFTAP_IF, mymac);
	ets_sprintf( c.ssid+9, "%02x%02x%02x", mymac[3],mymac[4],mymac[5] ); 
	c.password[0] = 0;
	c.ssid_len = ets_strlen( c.ssid );

	//Advance/loop channel.
	if( chadvance < -1 ) chadvance = -1;
	if( chadvance > 1 ) chadvance = 1;
	c.channel += chadvance;
	if( c.channel <= 0 ) c.channel = 13;
	if( c.channel > 13 ) c.channel = 1;

	if( force_channel ) c.channel  = force_channel;

	c.authmode = NULL_MODE;
	c.ssid_hidden = 0;
	c.max_connection = 4;
	c.beacon_interval = 1000;
	wifi_softap_set_config(&c);
	wifi_softap_set_config_current(&c);
	wifi_set_opmode_current( 2 );
	wifi_softap_set_config_current(&c);
	wifi_set_channel( c.channel );	
	printf( "Making it a softap, channel %d\n", c.channel );
	got_an_ip = 1;
	soft_ap_mode = 1;

	int b1 = (c.channel&1)?40:0;
	int b2 = (c.channel&2)?40:0;
	int b3 = (c.channel&4)?40:0;
	int b4 = (c.channel&8)?40:0;
	ledOut[0] = b1; ledOut[1] = b1; ledOut[2] = b1;
	ledOut[3] = b2; ledOut[4] = b2; ledOut[5] = b2;
	ledOut[6] = b3; ledOut[7] = b3; ledOut[8] = b3;
	ledOut[9] = b4; ledOut[10] = b4; ledOut[11] = b4;
	ws2812_push( ledOut, 12 ); //Buffersize = Nr LEDs * 3

	return c.channel;
}

static void ICACHE_FLASH_ATTR scandone(void *arg, STATUS status)
{
	uint8_t * gsp = global_scan_data;
	global_scan_elements = 0;
	scaninfo *c = arg;
	struct bss_info *inf;
	scan_ccount = last_ccount;
	if( !c->pbss ) {global_scan_elements = 0;  global_scan_complete = 1; return;  }
	STAILQ_FOREACH(inf, c->pbss, next) {
		global_scan_elements++;
		//not using inf->authmode;
		ets_memcpy( gsp, inf->bssid, 6 );
		gsp+=6;
		*(gsp++) = inf->rssi;
		*(gsp++) = inf->channel;
		if( gsp - global_scan_data >= sizeof(global_scan_data)-8 ) break;
	}
	global_scan_complete = 1;
}

uint32_t ICACHE_FLASH_ATTR lEHSVtoHEX( uint8_t hue, uint8_t sat, uint8_t val )
{
	EHSVtoHEX( hue, sat, val );
}

void ICACHE_FLASH_ATTR procTaskDeep()
{
	static uint32_t sleepy, nosleepy;
	static int joined_igmp = 0;
	static int ict = 0;
	uint32_t ccount;
	uint32_t delta;
	static uint8_t notfirst;
	static uint16_t status_update_count;

	ccount = system_get_time();
	if( !notfirst )
	{
		//This code is excuted on first powerup.
		int firstbuttons = GetButtons();

		if( firstbuttons & 0x04 ) //Left is being held.  FLASH LIGHT
		{
			force_sleep_time = 0xfffffffe;
			ledOut[0] = 0xff; ledOut[1] = 0xff; ledOut[2] = 0xff;
			ledOut[3] = 0xff; ledOut[4] = 0xff; ledOut[5] = 0xff;
			ledOut[6] = 0xff; ledOut[7] = 0xff; ledOut[8] = 0xff;
			ledOut[9] = 0xff; ledOut[10] = 0xff; ledOut[11] = 0xff;
			ws2812_push( ledOut, 12 ); //Buffersize = Nr LEDs * 3
/*			ets_delay_us(10000);
			ws2812_disable();
			system_deep_sleep_set_option( 0 );
			system_deep_sleep( 0xfffffffe );*/
			ets_delay_us(1000000);
		}

		if( firstbuttons & 0x08 ) //Disable deep sleep
		{
			//disable_deep_sleep = 1;
		}

		last_ccount = ccount;
		notfirst = 1;
	}
	delta = ccount - last_ccount;

	if( delta > 10000 )
	{
		sleepy += delta;
	}
	else
	{
		nosleepy += delta;
	}

/*
	if( do_deep_sleep && !disable_deep_sleep )
	{
		wifi_set_sleep_type( 0 );
		ledOut[0] = 0; ledOut[1] = 1; ledOut[2] = 0;
		ledOut[3] = 0; ledOut[4] = 1; ledOut[5] = 0;
		ledOut[6] = 0; ledOut[7] = 1; ledOut[8] = 0;
		ledOut[9] = 0; ledOut[10] = 1; ledOut[11] = 0;
		ws2812_push( ledOut, 12 ); //Buffersize = Nr LEDs * 3
		ets_delay_us(10000);
		ws2812_disable();
		system_deep_sleep_set_option( 0 );
		system_deep_sleep( force_sleep_time?force_sleep_time:30000000 );
		ets_delay_us(1000000);
	}

	if( wifi_fails > 1 )
	{
		do_deep_sleep = 1;
	}*/

	if( wifi_fails > 1 )
	{
		//XXX XXX TODO: consider going into AP mode here?
	}

	int do_status_update = 0;
	static int time_since_update = 0;
	static int just_joined_network = 0;

	time_since_update += delta;


	//Time out pending UDP transactions
	if( udp_pending )
	{
		int newpending = udp_pending - delta;
		if( newpending < 0 )
			udp_pending = 0;
		else
			udp_pending = newpending;
	}

	//User-selected softAP mode, check to see if 
	if( soft_ap_mode && !in_modem_only_mode )
	{
		static int last_chsetstate;
		int chsetstate = LastGPIOState & 0xc0;
		if( (chsetstate & 0x80) && !(last_chsetstate&0x80) )
		{
			//Increment channel.
			SwitchToSoftAP( 1 );
		}
		if( (chsetstate & 0x40) && !(last_chsetstate&0x40) )
		{
			//Decrement channel.
			SwitchToSoftAP( -1 );
		}

		last_chsetstate = chsetstate;
	}



	if( last_button_event_btn ) 			do_status_update = 1;  //GPIO event?
	if( time_since_update > 1000000 * 10 )	do_status_update = 2;  //>10 seconds between updates.
	if( just_joined_network )				do_status_update = 1;  //Just joined?  Status update.
	if( requested_update )					do_status_update = 1;  //Requested an update?

	//Determine if we want to send a status packet.
	if( do_status_update && !udp_pending )
	{
		uint8_t * pp = &mypacket[PACK_PAYLOAD_START];
		static int sleepperformance = 0;

		if( do_status_update == 2 )
		{
			//Long-term interval.  report sleep perfomance.
			sleepperformance = (sleepy * 255) / (sleepy+nosleepy);
			sleepy = 0;
			nosleepy = 0;
			if( disable_push ) goto skip_status;
		}

		#define STATUS_PACKSIZE 32

		pp+=6; //MAC is first.		

		*(pp++) = 0x01; //Message type
		*(pp++) = 0;
		*(pp++) = wifi_station_get_rssi();	//Wifi power

		{
			struct station_config stationConf;
			wifi_station_get_config(&stationConf);
			ets_memcpy( pp, stationConf.bssid, 6 );
		}
		pp+=6;

		*(pp++) = LastGPIOState;          							//Last GPIO State
		//printf( "%d %d %d\n", LastGPIOState, last_button_event_btn,last_button_event_dn);
		*(pp++) = last_button_event_btn; last_button_event_btn = 0; //Last GPIO Event (0 = none)
		*(pp++) = last_button_event_dn;  							 //Was last GPIO Event down? 
		//computer power for LED.
		int i;
		int sum;
		for( i = 0; i < 12; i++ )
		{
			sum += ledOut[i];
		}
		*(pp++) = sum / 12;						//Average power to LEDs

		uint16_t vv = system_get_vdd33();					//System 3.3v
		*(pp++) = vv>>8;
		*(pp++) = vv & 0xff;

		*(pp++) = status_update_count>>8;						//# of packets sent.
		*(pp++) = status_update_count&0xff;

		uint16_t heapfree = system_get_free_heap_size();
		*(pp++) = heapfree>>8;
		*(pp++) = heapfree & 0xff;

		//Metric discussing how effective is sleeping.
		*(pp++) = sleepperformance & 0xff;
		*(pp++) = 0;

		*(pp++) = ccount>>24;
		*(pp++) = ccount>>16;
		*(pp++) = ccount>>8;
		*(pp++) = ccount>>0;

		status_update_count++;
		time_since_update = 0;
		just_joined_network = 0;
		requested_update = 0;

		if( got_an_ip )
		{
			if( send_back_on_ip && send_back_on_port )
			{
				pUdpServer->proto.udp->remote_port = send_back_on_port;
				uint32_to_IP4(send_back_on_ip,pUdpServer->proto.udp->remote_ip);
				send_back_on_ip = 0; send_back_on_port = 0;
			}
			else
			{
				pUdpServer->proto.udp->remote_port = 8000;
				uint32_to_IP4(REMOTE_IP_CODE,pUdpServer->proto.udp->remote_ip);  
			}



			udp_pending = UDP_TIMEOUT;
			espconn_send( (struct espconn *)pUdpServer, &mypacket[PACK_PAYLOAD_START], STATUS_PACKSIZE );
		}

		if( !disable_raw_broadcast )
		{
			packet_tx_time = 0;
			wifi_set_phy_mode(PHY_MODE_11N);
			wifi_set_user_fixed_rate( 3, raw_broadcast_rate );
			wifi_send_pkt_freedom( mypacket, 30 + STATUS_PACKSIZE, true) ; 
		}
	}

skip_status:

	if( do_led_push )
	{
		ws2812_push( ledOut, ledqtybytes );
		do_led_push = 0;
	}
	else if( rainbow_run )
	{
		int i;
		int new_rainbow = rainbow_run - delta;

		if( new_rainbow <= 0 )
		{
			rainbow_run = 0;
			ws2812_push( ledOut, 12 ); //Buffersize = Nr LEDs * 3
		}
		else
		{
			uint8_t rainbow_leds[12];
			rainbow_run = new_rainbow;

			for( i = 0; i < 4; i++ )
			{
				uint32_t h = lEHSVtoHEX( rainbow_run * rainbow_speed + rainbow_offset * i, 255, rainbow_intensity );
				rainbow_leds[i*3+0] = h>>8;
				rainbow_leds[i*3+1] = h;
				rainbow_leds[i*3+2] = h>>16;
			}
			ws2812_push( rainbow_leds, 12 ); //Buffersize = Nr LEDs * 3
		}

	}


	if( make_led_rssi )
	{

		int i;
		sint8 rssi = 0;  //Lower number is worse. Valid range = 0 to ~60-80?
		if( led_rssi == -1 )
		{
			rssi = wifi_station_get_rssi()+100;
		}
		else
		{
			rssi = led_rssi;
		}

		int rs = (rssi - ledrssi_min)*1024;
		rs /= (ledrssi_max-ledrssi_min);

		for( i = 0; i < 4; i++ )
		{
			uint32_t h = lEHSVtoHEX( rs, 1.0, ledrssi_intensity );
			ledOut[i*3+0] = h>>8;
			ledOut[i*3+1] = h;
			ledOut[i*3+2] = h>>16;
		}


		//Update LEDs
		ws2812_push( ledOut, 12 ); //Buffersize = Nr LEDs * 3

		make_led_rssi = 0;
	}

	if( need_to_do_scan )
	{
		int r;
		struct scan_config sc;
		sc.ssid = 0;  sc.bssid = 0;  sc.channel = 0;  sc.show_hidden = 1;
		global_scan_complete = 0;
		global_scan_elements = 0;
		r = wifi_station_scan(&sc, scandone );
		if( r )
		{
			//Scan good
		}
		else
		{
			global_scan_complete = 1;
			global_scan_elements = 0;
		}
		//send out globalscanpacket.
		need_to_do_scan = 0;
	}

	//XXX Tricky, after a scan is complete, start shifting the UDP data out.
	if( global_scan_complete && !udp_pending )
	{
		int i;
		uint8_t * pp = &mypacket[PACK_PAYLOAD_START];
		pp+=6; //MAC is first.		
		*(pp++) = 0x05; //Message type

		//do this here so we always have a "scan complete" packet sent back to host.
		if( global_scan_elements == 0 )
		{
			send_back_on_ip = 0;
			send_back_on_port = 0;
			global_scan_complete = 0;
		}

#define MAX_STATIONS_PER_PACKET 130

		int stations = global_scan_elements;
		if( stations > MAX_STATIONS_PER_PACKET ) stations = MAX_STATIONS_PER_PACKET;
		*(pp++) = scan_ccount>>24;
		*(pp++) = scan_ccount>>16;
		*(pp++) = scan_ccount>>8;
		*(pp++) = scan_ccount>>0;

		*(pp++) = stations;

		ets_memcpy( pp, global_scan_data, stations*8 );
		pp += stations*8;
		//Slide all remaining stations up.
		for( i = 0; i < global_scan_elements - stations; i++ )
		{
			ets_memcpy( global_scan_data + i*8, global_scan_data + (i+stations)*8, 8 );
		}
		global_scan_elements -= stations;


		if( send_back_on_ip && send_back_on_port)
		{
			pUdpServer->proto.udp->remote_port = send_back_on_port;
			uint32_to_IP4(send_back_on_ip,pUdpServer->proto.udp->remote_ip);  
		}
		else
		{
			pUdpServer->proto.udp->remote_port = 8000;
			uint32_to_IP4(REMOTE_IP_CODE,pUdpServer->proto.udp->remote_ip);  
		}


		udp_pending = UDP_TIMEOUT;
		espconn_send( (struct espconn *)pUdpServer, &mypacket[PACK_PAYLOAD_START], pp - &mypacket[PACK_PAYLOAD_START]  );
	}

#ifdef ADMIN_BADGE
	if( do_admin_send )
	{
		uint8_t * pp = &mypacket[PACK_PAYLOAD_START];
		pp+=6; //MAC is first.		
		int apl = adminsendplace;
		ets_memcpy( pp, adminsendbuffer, apl );
		adminsendplace = 0;
		do_admin_send = 0;
		packet_tx_time = 0;
		wifi_set_phy_mode(PHY_MODE_11N);
		wifi_set_user_fixed_rate( 3, raw_broadcast_rate );
		wifi_send_pkt_freedom( mypacket, 30 + 6 + apl, true) ; 
	}

	static uint32_t time_since_stream = 0;
	time_since_stream+=delta;
	static uint16_t ssframe;
	//Tricky: While so we can break out easily.
	while( time_since_stream > 30000 ) //1/33rd? of a second.
	{
		time_since_stream = 0;
		ssframe++;
		uint8_t * pp = &mypacket[PACK_PAYLOAD_START];
		pp+=6; //MAC is first.		

		if( LastGPIOState & 0x01 )
		{
			*(pp++) = 0x02; //Message type (1 = set colors)
			*(pp++) = 0x00;
			*(pp++) = 0x00;
			*(pp++) = 0x00;

			int i;
			for( i = 0; i < 4; i++ )
			{
				uint32_t h = lEHSVtoHEX( ssframe + i*10,255,255 );
				*(pp++) = h>>8;
				*(pp++) = h;
				*(pp++) = h>>16;
			}
		}
		else if( ( (LastGPIOState & 0x1c) == 0x1c ) && !in_modem_only_mode )
		{
			printf( "Entering modem-only mode\n" );
			SwitchToSoftAP( 0 );
			printf( "Operating on channel %d\n", wifi_get_channel() );
			//Tricky: If this button is clicked, enter modem-only mode.
			ets_delay_us(20000);
			ws2812_disable();

			PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
			PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
			PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, 0); //default RX.
			PIN_DIR_INPUT = 1<<3;
			in_modem_only_mode = 1;

			//Timer example
			os_timer_disarm(&force_timer);
			os_timer_setfn(&force_timer, (os_timer_func_t *)ForceTimer, NULL);
			os_timer_arm(&force_timer, 1, 1);

		}
		else if( LastGPIOState & 0x02 )
		{
			*(pp++) = 0x02; //Message type (1 = set colors)
			*(pp++) = 0x00;
			*(pp++) = 0x00;
			*(pp++) = 0x00;

			int i;
			for( i = 0; i < 4; i++ )
			{
				uint32_t h = lEHSVtoHEX( ssframe + i*10, 255, 2 );
				*(pp++) = h>>8;
				*(pp++) = h;
				*(pp++) = h>>16;
			}
		}
		else if( LastGPIOState & 0x04 )
		{
			*(pp++) = 0x03; //Message typ (0x13 = RSSI Response)
			*(pp++) = 0x08; //Min
			*(pp++) = 0x38; //Max
			*(pp++) = 0xc0; //1/2 brightness
		}
		else
		{
			//Nothing to do? Break!
			break;
		}

		packet_tx_time = 0;
		wifi_set_phy_mode(PHY_MODE_11N);
		wifi_set_user_fixed_rate( 3, raw_broadcast_rate );
		wifi_send_pkt_freedom( mypacket, 30 + 22, true) ; 
	}

#endif


	static uint32_t time_since_slowtick = 0;
	time_since_slowtick+=delta;
	if( time_since_slowtick > 100000 ) //1/10th of a second.
	{
		ict = 0;
		struct station_config wcfg;
		struct ip_info ipi;
		int stat = wifi_station_get_connect_status();
		if( stat == STATION_WRONG_PASSWORD || stat == STATION_NO_AP_FOUND || stat == STATION_CONNECT_FAIL ) {
			wifi_station_disconnect();
			wifi_fails++;
			printf( "Connection failed with code %d... Retrying, try: ", stat, wifi_fails );
			wifi_station_connect();
			printf("\n");
			got_an_ip = 0;
		} else if( stat == STATION_GOT_IP && !got_an_ip ) {
			wifi_station_get_config( &wcfg );
			wifi_get_ip_info(0, &ipi);
			printf( "STAT: %d\n", stat );
			#define chop_ip(x) (((x)>>0)&0xff), (((x)>>8)&0xff), (((x)>>16)&0xff), (((x)>>24)&0xff)
			printf( "IP: %d.%d.%d.%d\n", chop_ip(ipi.ip.addr)      );
			printf( "NM: %d.%d.%d.%d\n", chop_ip(ipi.netmask.addr) );
			printf( "GW: %d.%d.%d.%d\n", chop_ip(ipi.gw.addr)      );
			printf( "Connected to: /%s/\n"  , wcfg.ssid );
			got_an_ip = 1;
			wifi_fails = 0;
		}

		//printf(".%02x %08x %d %d\n", LastGPIOState, PIN_IN, debugccount, debugccount2 );

		time_since_slowtick = 0;

		static int did_init = 0;
		if( !did_init && got_an_ip )
		{
			//For sending raw packets.
			//SetupRawsend(); Not needed.

			just_joined_network = 1;
#ifndef DISABLE_RAW_PACKET_ACCESS
			wifi_set_raw_recv_cb( rx_func );
			wifi_register_send_pkt_freedom_cb( sent_freedom_cb );
#endif
			did_init = 1;

			//Setup our send packet with our MAC address.
			wifi_get_macaddr(STATION_IF, mypacket + 10);
			wifi_get_macaddr(STATION_IF, mypacket + PACK_PAYLOAD_START);

			//LED Stuff when initialization started.
			ledOut[0] = 1; ledOut[1] = 0;  ledOut[2] = 2;
			ledOut[3] = 1; ledOut[4] = 0;  ledOut[5] = 2;
			ledOut[6] = 1; ledOut[7] = 0;  ledOut[8] = 2;
			ledOut[9] = 1; ledOut[10] = 0; ledOut[11] = 2;
			ws2812_push( ledOut, 12 ); //Buffersize = Nr LEDs * 3
		}
		//
		packet_tx_time = 0;
	}


end:
	last_ccount = ccount;
}


#endif


//Call this once we've stacked together one full colorchord frame.
static void NewFrame()
{
	if( !COLORCHORD_ACTIVE ) return;

	int i;
	HandleFrameInfo();

	switch( COLORCHORD_OUTPUT_DRIVER )
	{
	case 0:
		UpdateLinearLEDs();
		break;
	case 1:
		UpdateAllSameLEDs();
		break;
	};

	ws2812_push( ledOut, USE_NUM_LIN_LEDS * 3 );
}

uint32_t samp_iir = 0;
int wf = 0;

//Tasks that happen all the time.

void ICACHE_FLASH_ATTR procTask2(os_event_t *events)
{
	if( events->sig == 0 && events->par == 0 )
	{
		CSTick( 0 );
		//procTaskDeep();
	}
}


static void procTask(os_event_t *events)
{
	system_os_post(procTaskPrio, 0, 0 );

	if( COLORCHORD_ACTIVE && !hpa_running )
	{
		ExitCritical();
		hpa_running = 1;
	}

	if( !COLORCHORD_ACTIVE && hpa_running )
	{
		EnterCritical();
		hpa_running = 0;
	}
	
	//For profiling so we can see how much CPU is spent in this loop.
#ifdef PROFILE
	WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + GPIO_ID_PIN(0), 1 );
#endif
	while( soundtail != soundhead )
	{
		int32_t samp = sounddata[soundtail];
		samp_iir = samp_iir - (samp_iir>>10) + samp;
		samp = (samp - (samp_iir>>10))*16;
		samp = (samp * CCS.gINITIAL_AMP) >> 4;
		PushSample32( samp );
		soundtail = (soundtail+1)&(HPABUFFSIZE-1);

		wf++;
		if( wf == 128 )
		{
			NewFrame();
			wf = 0; 
		}
	}
#ifdef PROFILE
	WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + GPIO_ID_PIN(0), 0 );
#endif
	procTask2( events );
}


//Timer event.
static void ICACHE_FLASH_ATTR myTimer(void *arg)
{
	CSTick( 1 );

	if( hpa_is_paused_for_wifi && printed_ip )
	{
		StartHPATimer(); //Init the high speed  ADC timer.
		hpa_running = 1;
		hpa_is_paused_for_wifi = 0; // only need to do once prevents unstable ADC
	}
	uart0_sendStr(".");
//	printf( "%d/%d\n",soundtail,soundhead );
//	printf( "%d/%d\n",soundtail,soundhead );
//	uint8_t ledout[] = { 0x00, 0xff, 0xaa, 0x00, 0xff, 0xaa, };
//	ws2812_push( ledout, 6 );
}


#ifdef SWADGE

void ICACHE_FLASH_ATTR HandleButtonEvent( uint8_t stat, int btn, int down )
{
	//XXX WOULD BE NICE: Implement some sort of event queue.
	last_button_event_btn = btn+1;
	last_button_event_dn = down;
	system_os_post(0, 0, 0 );
}


void ICACHE_FLASH_ATTR ProcessData( uint8_t * data, int len )
{
	//PROTOCOL:
	//Bytes [0..5 MAC] ->
	// MAC from the server is ignored.
	//Starting at byte 6.

	//packets are sent to 10.201.0.2, UDP port 8000
	//Packets can come from anywhere.
	//Packets send packets to device on port 8001

	//Badge Status Update, sent at connect, after 10 seconds, upon request and button click.
	//  TO BADGE 0x11: Request status update.
	//  FROM BADGE 0x01: [0 reserved (could be version in future)] [RSSI] [BSSID of currently connected AP x6] [GPIO State] [Last button event, 0 if no events, 1-8 for button] [was last button event down] [Average LED power] [system 3.3v 2bytes] [status update count 2bytes] [heap free 2bytes] [sleep performance ratio] [0, reserved] [current time 4bytes]


	//Control WS2812Bs on badge.
	//    TO BADGE 0x02: [MAtch] [Mask] [Reserved, 0]   [GRB GRB GRB GRB ...]  NOTE: For raw packets, only 4 LEDs may be controlled.
	//		if (! ((mymac[5] ^ data[7])&data[8]) )

	//Glow LED to porportion of RSSI
	//    TO BADGE 0x03: [min RSSI] [max RSSI] [LED intensity]

	//Initiate SSID SCAN
	//    TO BADGE 0x04: (no parameters, ONLY UDP can control this!)
	//  FROM BADGE 0x05: [scan_timestamp (4 bytes)] [stations count in this message] [ [BSSIDx6bytes] [RSSI] [Channel] ]

	//Rainbow effect
	//    TO BADGE 0x07: [Reserved, 0] [Reserved, 0] [Reserved, 0] [Run time, MSB] [Run Time, LSB] [rainbow speed] [rainbow intensity] [rainbow offset per led]    Run time in ms.

	//Device configure
	//    TO BADGE 0x08: [requested update 1/0] [disable_push 1/0] [disable raw broadcast 1/0] [raw broadcast rate, recommended 0x1a]

	//Force deep sleep (UDP only)
	//    TO PADGE 0x09: [sleep ms MSB (4 bytes)] [0xaa, must be 0xaa]

	if( data[6] == 0x11 && len > 1 )
	{
		requested_update = 1;
	}

	//0x01 is from badge, status packet
	if( data[6] == 0x02 )
	{
		//Update LEDs
		rainbow_run = 0;
		if( len <= 0 )
		{
			len = 22;
		}
		if (! ((mymac[5] ^ data[7])&data[8]) )
		{
			ets_memcpy( ledOut, data + 10, len - 10 );
			ledqtybytes = len-10;
			do_led_push = 1;
		}
	}
	if( data[6] == 0x03 )  //RSSI the LED.
	{
		if( len == -1 )
		{
			static int rl1;
			static int rl2;
			static int rl3;
			int rl4;
			rl4 = rl3;
			rl3 = rl2;
			rl2 = rl1;
			rl1 = data[-42];
			led_rssi = (rl1+rl2+rl3+rl1)/4;
		}
		else
		{
			led_rssi = -1;
		}

		rainbow_run = 0;

		ledrssi_min = data[7];
		ledrssi_max = data[8];
		ledrssi_intensity = data[9];

		make_led_rssi = 1;
	}

	if( data[6] == 0x04 && len > 1 )  //Scan; make sure this can only be done via 
	{
		need_to_do_scan = 1;
	}

	//0x05 is from badge, browse response.

	if( data[6] == 0x07 )
	{
		rainbow_run = ((data[10]<<8) | data[11])*1000;
		rainbow_speed = data[12];
		rainbow_intensity = data[13];
		rainbow_offset = data[14];
	}
	if( data[6] == 0x08 && len > 1 )
	{
		requested_update = data[7];
		disable_push = data[8];
		disable_raw_broadcast = data[9];
		raw_broadcast_rate = data[10];
	}
	if( data[6] == 0x09 && len > 1 )
	{
		force_sleep_time = (data[7]<<24) || (data[8]<<16) || (data[9]<<8) || (data[10]);
		//do_deep_sleep = data[11] == 0xAA;
		//XXX Forced deep sleep disabled. 
	}
}


//Called when new packet comes in.
static void ICACHE_FLASH_ATTR udpserver_recv(void *arg, char *pusrdata, unsigned short len)
{
	struct espconn * rc = (struct espconn *)arg;
	remot_info * ri = 0;
	espconn_get_connection_info( rc, &ri, 0);

	if( pusrdata[6] == 0x11 || pusrdata[6] == 0x04 )
	{
		send_back_on_ip = IP4_to_uint32(ri->remote_ip);
		send_back_on_port = ri->remote_port;
	}
	struct espconn *pespconn = (struct espconn *)arg;
	ProcessData( pusrdata, len );
}

#else 
static void ICACHE_FLASH_ATTR udpserver_recv(void *arg, char *pusrdata, unsigned short len)
{
	printf( "UDP\n" );
}

#endif

//	uint8_t buffer[MAX_FRAME];
//	uint8_t ledout[] = { 0x00, 0xff, 0xaa, 0x00, 0xff, 0xaa, };
//	uart0_sendStr("X");
//	ws2812_push( pusrdata+3, len );

void ICACHE_FLASH_ATTR charrx( uint8_t c )
{
	//Called from UART.
}


void ICACHE_FLASH_ATTR user_init(void)
{
	wifi_fails = 1;  //Tricky if just booting, like from a wakeup, only look once for an AP, otherwise fail.
	//If you just failed, wait once.
#ifndef DISABLE_RAW_PACKET_ACCESS
	debugcontrol = 0xffffffff;
#endif
	//Uncomment this to force a system restore.
	system_restore();

#ifdef SWADGE
	rainbow_run = 500000;
	rainbow_intensity = 30;
	rainbow_speed = 10;
	rainbow_offset = 30;
#endif
	uart_init(BIT_RATE_115200, BIT_RATE_115200);

	uart0_sendStr("\r\nMAGStock2018\r\n");

#ifdef SWADGE
	SetupGPIO();


	int firstbuttons = GetButtons();
	//if( firstbuttons & 0x20 ) disable_deep_sleep = 1;
	if( (firstbuttons & 0x10) || force_channel )
	{
		SwitchToSoftAP( 0 );
		uart0_sendStr( "Booting in SoftAP\n" );
		rainbow_run = 6000000;
		rainbow_intensity = 250;
		rainbow_speed = 3;
		rainbow_offset = 0;
	}
	else
	{
		struct station_config stationConf;
		wifi_station_get_config(&stationConf);
		wifi_get_macaddr(STATION_IF, mymac);
		uart0_sendStr( "Connecting to infrastructure\n" );
		LoadSSIDAndPassword( stationConf.ssid, stationConf.password );
		stationConf.bssid_set = 0;
		wifi_set_opmode_current( 1 );
		wifi_set_opmode( 1 );
		wifi_station_set_config(&stationConf);
		wifi_station_connect();
		wifi_station_set_config(&stationConf);  //I don't know why, doing this twice seems to make it store more reliably.
		ledOut[0] = 1; ledOut[1] = 0; ledOut[2] = 0;
		ledOut[3] = 0; ledOut[4] = 1; ledOut[5] = 0;
		ledOut[6] = 0; ledOut[7] = 1; ledOut[8] = 0;
		ledOut[9] = 0; ledOut[10] = 1; ledOut[11] = 0;
		soft_ap_mode = 0;
	}

#endif
	uart0_sendStr( "Continuing boot.\n" );


	CustomStart();

	uart0_sendStr( "Doing preinit.\n" );

#ifdef PROFILE
	#error Profiling no allowed on MAGStock swadge.
	GPIO_OUTPUT_SET(GPIO_ID_PIN(0), 0);
#endif

	CSPreInit();

	uart0_sendStr( "Creating Socket.\n" );

    pUdpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
	ets_memset( pUdpServer, 0, sizeof( struct espconn ) );
	espconn_create( pUdpServer );
	pUdpServer->type = ESPCONN_UDP;
	pUdpServer->proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));
	pUdpServer->proto.udp->local_port = 8001;
	pUdpServer->proto.udp->remote_port = 8000;
	uint32_to_IP4(REMOTE_IP_CODE,pUdpServer->proto.udp->remote_ip); 
	espconn_regist_recvcb(pUdpServer, udpserver_recv);
	if( espconn_create( pUdpServer ) )
	{
		while(1) { uart0_sendStr( "\r\nFAULT\r\n" ); }
	}

	uart0_sendStr( "Starting common system.\n" );

	CSInit();

	while(1);

	//Add a process
	system_os_task(procTask, procTaskPrio, procTaskQueue, procTaskQueueLen);

	//Timer example
	os_timer_disarm(&some_timer);
	os_timer_setfn(&some_timer, (os_timer_func_t *)myTimer, NULL);
	os_timer_arm(&some_timer, 100, 1);

	//Set GPIO16 for Input
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1); 	// mux configuration for XPD_DCDC and rtc_gpio0 connection

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0);	//mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe);	//out disable

	uart0_sendStr( "Initializing ColorChord.\n" );

	InitColorChord(); //Init colorchord

	//Tricky: If we are in station mode, wait for that to get resolved before enabling the high speed timer.
	if( wifi_get_opmode() == 1 )
	{
		hpa_is_paused_for_wifi = 1;
	}
	else
	{
		StartHPATimer(); //Init the high speed  ADC timer.
		hpa_running = 1;
	}
	

	ws2812_init();

	uart0_sendStr( "Boot complete.\n" );

	// Attempt to make ADC more stable
	// https://github.com/esp8266/Arduino/issues/2070
	// see peripherals https://espressif.com/en/support/explore/faq
	//wifi_set_sleep_type(NONE_SLEEP_T); // on its own stopped wifi working
	//wifi_fpm_set_sleep_type(NONE_SLEEP_T); // with this seemed no difference

	system_os_post(procTaskPrio, 0, 0 );
}



void EnterCritical()
{
	PauseHPATimer();
	//ets_intr_lock();
}

void ExitCritical()
{
	//ets_intr_unlock();
	ContinueHPATimer();
}


