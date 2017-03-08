// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.
//
// Now for something a bit more challenging.
//
// Building sentences by program.
//
// The sayNumber() function can say any number under a million by
// building the number from short phrases,
//
// Connect a sensor to Analog 0, and this program will read the sensor voltage.

#include "talkie.h"
#include "voice.h"

Talkie voice;

uint16_t wdDelay = 200;	
extern uint8_t lastCallRSSI;
extern GeodeticPosition currentPosition;

void sayRSSI(){
	voice.say(spPOWER);
	delay(wdDelay);
	uint8_t rssiPercent = constrain((((uint16_t) lastCallRSSI * 100) / RSSI_MAX),0,100);
	sayNumberFancy(rssiPercent);
	voice.say(spPERCENT);
}

void sayCallsign(){
	char *c=CALLSIGN;
	sayAlphaNumString(c);
}

void vtest() {
	Serial.end();
	rfm_init();
	rfm_tx();
	beaconSquelch();	
	voice.begin();
	voice.setVol(100);
	
	if(lastCallRSSI){
		sayRSSI();
		lastCallRSSI=0;
		delay(1000);
	}
	sayCoords();
	delay(1000);
	sayCallsign();
	voice.end();
	rfm_deinit();
	resetGpsPort();
}

void sayCoords(){
	char tmp[15];
	int16_t n;

	n = (( currentPosition.latitude + GPS_LAT_OFFSET)/ 10000000 );
	sprintf(tmp,"%07ld", abs(currentPosition.latitude % 10000000) );
	
	tmp[0]='0';
	tmp[1]='\0';
	
	uint32_t nl = abs(currentPosition.latitude % 10000000) ;
	if(nl){
		sprintf(tmp,"%07ld", nl);
	}
	sayNumberFancy(n);
	voice.say(spPOINT);
	sayAlphaNumString(tmp);
	
	delay(1000);
	
	n = (( currentPosition.longitude + GPS_LON_OFFSET) / 10000000 );

	tmp[0]='0';
	tmp[1]='\0';
	
	nl = abs(currentPosition.longitude % 10000000) ;
	if(nl){
		sprintf(tmp,"%07ld", nl);
	}
	sayNumberFancy(n);
	voice.say(spPOINT);
	sayAlphaNumString(tmp);
}

/* Say any number between -999,999 and 999,999 */
void sayNumberFancy(long n) {
  if (n<0) {
    voice.say(spMINUS);
    sayNumberFancy(-n);
  } else if (n==0) {
    voice.say(spZERO);
  } else {
    if (n>=1000) {
      int thousands = n / 1000;
      sayNumberFancy(thousands);
      voice.say(spTHOUSAND);
      n %= 1000;
      //if ((n > 0) && (n<100)) voice.say(spAND);
    }
    if (n>=100) {
      int hundreds = n / 100;
      sayNumberFancy(hundreds);
      voice.say(spHUNDRED);
      n %= 100;
      // if (n > 0) voice.say(spAND);
    }
    if (n>19) {
      int tens = n / 10;
      switch (tens) {
        case 2: voice.say(spTWENTY); break;
        case 3: voice.say(spTHIRTY); break;
        case 4: voice.say(spFOURTY);  break;
        case 5: voice.say(spFIFTY); break;
        case 6: voice.say(spSIXTY); break;
        case 7: voice.say(spSEVENTY); break;
        case 8: voice.say(spEIGHTY); break;
        case 9: voice.say(spNINETY);  break;
      }
      n %= 10;
    }
    switch(n) {
      case 1: voice.say(spONE); break;
      case 2: voice.say(spTWO); break;
      case 3: voice.say(spTHREE); break;
      case 4: voice.say(spFOUR); break;
      case 5: voice.say(spFIVE); break;
      case 6: voice.say(spSIX); break;
      case 7: voice.say(spSEVEN); break;
      case 8: voice.say(spEIGHT); break;
      case 9: voice.say(spNINE); break;
      case 10: voice.say(spTEN); break;
      case 11: voice.say(spELEVEN); break;
      case 12: voice.say(spTWELVE); break;
      case 13: voice.say(spTHIR_); voice.say(sp_TEEN); break;
      case 14: voice.say(spFOUR); voice.say(sp_TEEN);break;
      case 15: voice.say(spFIF_); voice.say(sp_TEEN); break;
      case 16: voice.say(spSIX); voice.say(sp_TEEN); break;
      case 17: voice.say(spSEVEN); voice.say(sp_TEEN); break;
      case 18: voice.say(spEIGHT); voice.say(sp_TEEN); break;
      case 19: voice.say(spNINE); voice.say(sp_TEEN); break;
    }
  }
}

void sayAlphaNumString(const char *s){
	while(*s){
		switch(*s++){
			case '.': voice.say(spPOINT);break;
			case '-': voice.say(spMINUS); break;
			case '0': voice.say(spZERO); break;
			case '1': voice.say(spONE); break;
			case '2': voice.say(spTWO); break;
			case '3': voice.say(spTHREE); break;
			case '4': voice.say(spFOUR); break;
			case '5': voice.say(spFIVE); break;
			case '6': voice.say(spSIX); break;
			case '7': voice.say(spSEVEN); break;
			case '8': voice.say(spEIGHT); break;
			case '9': voice.say(spNINER); break;
			case 'A': voice.say(spALPHA);break;
			case 'B': voice.say(spBRAVO); break;
			case 'C': voice.say(spCHARLIE); break;
			case 'D': voice.say(spDELTA); break;
			case 'E': voice.say(spECHO); break;
			case 'F': voice.say(spFOXTROT); break;
			case 'G': voice.say(spGOLF); break;
			case 'H': voice.say(spHOTEL); break;
			case 'I': voice.say(spINDIA); break;
			case 'J': voice.say(spJULIET); break;
			case 'K': voice.say(spKILO); break;
			case 'L': voice.say(spLIMA); break;
			case 'M': voice.say(spMIKE);break;
			case 'N': voice.say(spNOVEMBER); break;
			case 'O': voice.say(spOSCAR); break;
			case 'P': voice.say(spPAPA); break;
			case 'Q': voice.say(spQUEBEC); break;
			case 'R': voice.say(spROMEO); break;
			case 'S': voice.say(spSIERRA); break;
			case 'T': voice.say(spTANGO); break;
			case 'U': voice.say(spUNIFORM); break;
			case 'V': voice.say(spVICTOR); break;
			case 'W': voice.say(spWHISKEY); break;
			case 'X': voice.say(spXRAY); break;
			case 'Y': voice.say(spYANKEE); break;
			case 'Z': voice.say(spZULU); break;
		}
	}
}
