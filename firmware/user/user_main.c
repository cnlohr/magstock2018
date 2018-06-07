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
#include "gpio.h"
//#define PROFILE

#define PORT 7777
#define SERVER_TIMEOUT 1500
#define MAX_CONNS 5
#define MAX_FRAME 2000

#define procTaskPrio        0
#define procTaskQueueLen    1

struct CCSettings CCS;
static volatile os_timer_t some_timer;
static struct espconn *pUdpServer;

void EnterCritical();
void ExitCritical();

extern volatile uint8_t sounddata[HPABUFFSIZE];
extern volatile uint16_t soundhead;
uint16_t soundtail;

static uint8_t hpa_running = 0;
static uint8_t hpa_is_paused_for_wifi;

int got_an_ip = 0;
int soft_ap_mode = 0;
int wifi_fails;
uint8_t mymac[6];

uint8_t last_button_event_btn;
uint8_t last_button_event_dn;



static int ICACHE_FLASH_ATTR SwitchToSoftAP( )
{
	struct softap_config c;
	wifi_softap_get_config_default(&c);
	memcpy( c.ssid, "MAGBADGE_", 9 );
	wifi_get_macaddr(SOFTAP_IF, mymac);
	ets_sprintf( c.ssid+9, "%02x%02x%02x", mymac[3],mymac[4],mymac[5] ); 
	c.password[0] = 0;
	c.ssid_len = ets_strlen( c.ssid );
	c.channel = 1;
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
	return c.channel;
}



void ICACHE_FLASH_ATTR CustomStart( );

void ICACHE_FLASH_ATTR user_rf_pre_init()
{
}


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

os_event_t    procTaskQueue[procTaskQueueLen];
uint32_t samp_iir = 0;
int wf = 0;

//Tasks that happen all the time.

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

	if( events->sig == 0 && events->par == 0 )
	{
		CSTick( 0 );
	}

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


	{
		struct station_config wcfg;
		struct ip_info ipi;
		int stat = wifi_station_get_connect_status();
		if( stat == STATION_WRONG_PASSWORD || stat == STATION_NO_AP_FOUND || stat == STATION_CONNECT_FAIL ) {
			wifi_station_disconnect();
			wifi_fails++;
			printf( "Connection failed with code %d... Retrying, try: %d", stat, wifi_fails );
			wifi_station_connect();
			printf("\n");
			if( wifi_fails == 4 )
			{
				SwitchToSoftAP();
				got_an_ip = 1;
			}
			memset(  ledOut + wifi_fails*3, 255, 3 );
			ws2812_push( ledOut, USE_NUM_LIN_LEDS * 3 );
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
	}



//	uart0_sendStr(".");
//	printf( "%d/%d\n",soundtail,soundhead );
//	printf( "%d/%d\n",soundtail,soundhead );
//	uint8_t ledout[] = { 0x00, 0xff, 0xaa, 0x00, 0xff, 0xaa, };
//	ws2812_push( ledout, 6 );
}


//Called when new packet comes in.
static void ICACHE_FLASH_ATTR udpserver_recv(void *arg, char *pusrdata, unsigned short len)
{
	struct espconn *pespconn = (struct espconn *)arg;
//	uint8_t buffer[MAX_FRAME];
//	uint8_t ledout[] = { 0x00, 0xff, 0xaa, 0x00, 0xff, 0xaa, };
	uart0_sendStr("X");
	ws2812_push( pusrdata+3, len );
}

void ICACHE_FLASH_ATTR charrx( uint8_t c )
{
	//Called from UART.
}


 
void ICACHE_FLASH_ATTR HandleButtonEvent( uint8_t stat, int btn, int down )
{
	//XXX WOULD BE NICE: Implement some sort of event queue.
	last_button_event_btn = btn+1;
	last_button_event_dn = down;
	system_os_post(0, 0, 0 );
}


void ICACHE_FLASH_ATTR user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	int wifiMode = wifi_get_opmode();

	uart0_sendStr("\r\nColorChord\r\n");

//Uncomment this to force a system restore.
//	system_restore();

	CustomStart();

#ifdef PROFILE
	GPIO_OUTPUT_SET(GPIO_ID_PIN(0), 0);
#endif
	SetupGPIO();

	int firstbuttons = GetButtons();
	//if( firstbuttons & 0x20 ) disable_deep_sleep = 1;
	if( (firstbuttons & 0x10) )
	{
		SwitchToSoftAP( 0 );
		uart0_sendStr( "Booting in SoftAP\n" );
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
		soft_ap_mode = 0;
	}

 

	CSPreInit();

    pUdpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
	ets_memset( pUdpServer, 0, sizeof( struct espconn ) );
	espconn_create( pUdpServer );
	pUdpServer->type = ESPCONN_UDP;
	pUdpServer->proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));
	pUdpServer->proto.udp->local_port = 7777;
	espconn_regist_recvcb(pUdpServer, udpserver_recv);

	if( espconn_create( pUdpServer ) )
	{
		while(1) { uart0_sendStr( "\r\nFAULT\r\n" ); }
	}

	CSInit();

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

	// Attempt to make ADC more stable
	// https://github.com/esp8266/Arduino/issues/2070
	// see peripherals https://espressif.com/en/support/explore/faq
	//wifi_set_sleep_type(NONE_SLEEP_T); // on its own stopped wifi working
	//wifi_fpm_set_sleep_type(NONE_SLEEP_T); // with this seemed no difference

	memset( ledOut, 255, 3 );
	ws2812_push( ledOut, USE_NUM_LIN_LEDS * 3 );

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


