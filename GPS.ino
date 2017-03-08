// ***** GPS ROUTINES *****

#ifdef USE_GPS
#include "GpsDataType.h"

extern struct gpsData gpsData; 
extern GeodeticPosition currentPosition;
uint8_t initialLock = 0;

#ifdef GPS_SEND_EVERY
uint16_t gps_counter = 1;
uint8_t sec_counter = 0;
#endif

/* 
void sendGPS(void)
{
	Serial.end();
	
	rfm_init();
	rfm_tx();
	beaconSquelch();	

	if(initialLock == 0)
	{
		#ifdef GPS_NO_FIX_MSG
		morseEncode(GPS_NO_FIX_MSG);
		#else
		beaconCE3K();
		#endif
	}
	#ifdef GPS_SEND_EVERY
	else if(gps_counter < GPS_SEND_EVERY) {
		beaconCE3K();
		gps_counter++;
	}
	#endif
	else
	{
		gps_counter = 1;
		char tmp0[40], tmp1[15];
		
		#ifdef GPS_FLOAT
		float latlon = ( (((float) currentPosition.latitude) / 10000000. ) + GPS_LAT_OFFSET );
		dtostrf( latlon, 11, 7, tmp0 );
		
		latlon = ( (((float) currentPosition.longitude) / 10000000. ) + GPS_LON_OFFSET );
		dtostrf( latlon, 12, 7, tmp1 );			
		
		#else
		sprintf(tmp1, "%ld", ( currentPosition.latitude + GPS_LAT_OFFSET ));
		sprintf(tmp2, "%ld",( currentPosition.longitude + GPS_LON_OFFSET ));		
		#endif
		
		sprintf(tmp0,"%s %s",tmp0, tmp1);
		
		beaconTone(900, 100);
		delay(80);
		beaconTone(800, 80);
		delay(500);
		
		#ifdef GPS_FIX_TONE
		if(haveAGpsLock())
		{
			beaconTone(500, 500);
		} else {
			beaconTone(150, 500);
		}
		delay(1000);	
		#endif
		
		morseEncode(":: ");
		morseEncode(tmp0);	

		#ifdef GPS_USE_CALLSIGN
		delay(MORSE_WORDSPACE_LEN);
		morseEncode(GPS_USE_CALLSIGN);
		#endif
		morseEncode(" ::");	
		
		delay(1000);
		beaconTone(500, 120);
		delay(80);
		beaconTone(500, 80);
		delay(800);
	}
	rfm_deinit();
	resetGpsPort();
}
 */

#endif

