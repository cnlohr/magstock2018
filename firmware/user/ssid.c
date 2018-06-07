#include "mem.h"
#include "c_types.h"
#include "user_interface.h"

void ICACHE_FLASH_ATTR LoadSSIDAndPassword( char * ssid, char * password )
{
	ets_memcpy( ssid, "BadgeFi", 7 );
	ets_memcpy( password, "youmustconstructadditonalpylons", 31 );
}


