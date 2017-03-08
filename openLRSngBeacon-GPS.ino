// OpenLRSngBeacon - GPS  version by csurf
// TO DO:
// move defines to configurable/EEPROM-saveable options
// clean up & test PPM/PWM stuff
// consider adding on/off (digital hi/low) swtching trigger on IO pin
// add MSP support
// add support for parsed coords input over serial (avoid direct GPS parsing)

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "hardware.h"
#include "rfm.h"
#include "GpsDataType.h"


// ***** BEACON ROUTINES *****

uint32_t beaconDelay = BEACON_DEADTIME;
uint16_t beaconRSSIavg = 255;
uint8_t lastCallRSSI = 0;
uint8_t checkBeaconRSSI(void)
{
	uint8_t r = 0;
	uint8_t brssi = beaconGetRSSI();
	if (brssi > ((beaconRSSIavg >> 2) + BEACON_RSSI_TRIGGER_THRESHOLD )) {
		Serial.println();
		Serial.println(brssi);
		lastCallRSSI = brssi;
		r = 1;
	}
	beaconRSSIavg = (beaconRSSIavg * 3 + brssi * 4) >> 2;
	return r;
}

uint8_t beaconGetRSSI(void)
{
	uint16_t rssiSUM = 0;

	rfm_init();
	rfm_rx();
	spiWriteRegister(0x79, 0); // ch 0 to avoid offset
	delay(1);
	rssiSUM += rfmGetRSSI();
	delay(1);
	rssiSUM += rfmGetRSSI();
	delay(1);
	rssiSUM += rfmGetRSSI();
	delay(1);
	rssiSUM += rfmGetRSSI();
	rfm_deinit();

	return rssiSUM >> 2;
}

void beaconCE3K(void)
{
	// close encounters tune
	// G, A, F, F (lower octave), C
	// octave 3:  392  440  349  175   261

	spiWriteRegister(0x6d, 0x07);   // 7 set max power 100mW
	delay(10);
	beaconTone(392, 1000);

	spiWriteRegister(0x6d, 0x05);   // 5 set mid power 25mW
	delay(10);
	beaconTone(440,1000);

	spiWriteRegister(0x6d, 0x04);   // 4 set mid power 13mW
	delay(10);
	beaconTone(349, 1000);

	spiWriteRegister(0x6d, 0x02);   // 2 set min power 3mW
	delay(10);
	beaconTone(175,1000);

	spiWriteRegister(0x6d, 0x00);   // 0 set min power 1.3mW
	delay(10);
	beaconTone(261, 2000);

}

void beaconTone(uint16_t hz, uint16_t lengthMs)
{
	// LIMITS:
	// min tone frequency: 8Hz
	// max tone length: 6.5 sec (6500ms)
	
	uint16_t HectoUsecsPerCycle = ( 10000 / hz );					// in (microseconds*100), for improved resolution
	uint16_t lenHectoUsecs = ( lengthMs * 10 );							// convert ms to (usec*100) ~=> 6.5 seconds max tone length
	uint16_t cycles = ( lenHectoUsecs / HectoUsecsPerCycle );

	for (uint16_t i = 0; i < cycles; i++) 
	{
		SDI_on;
		delayMicroseconds( HectoUsecsPerCycle * 50 ); // convert to microsecs (x100) && divide by 2 (half-cycle) => mult by 50 
		SDI_off;
		delayMicroseconds( HectoUsecsPerCycle * 50 ) ; // min frequency == 8Hz to avoid overflow 
	}
}

void beaconSquelch(void)
{
	beaconTone(SQUELCH_TONE_HZ, SQUELCH_OPEN_LEN);
	delay(SQUELCH_OPEN_DELAY);
}


// ***** PPM INPUT ROUTINES *****

// #define TIMER1_PRESCALER    8
// volatile uint16_t startPulse = 0;

// void setupPPMinput(void)
// {
	// // Setup timer1 for input capture (PSC=8 -> 0.5ms precision)
	// TCCR1A = ((1 << WGM10) | (1 << WGM11));
	// TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1 << ICES1));
	// OCR1A = 0xffff;
	// TIMSK1 |= (1 << ICIE1);   // Enable timer1 input capture interrupt
// }

// ISR(TIMER1_CAPT_vect)
// {
  // if (TCCR1B & (1 << ICES1)) {
    // startPulse = ICR1;
  // } else {
    // uint16_t pulseWidth = ICR1 - startPulse;
    // if ((pulseWidth >= MINPWM*2) && (pulseWidth <= MAXPWM * 2)) {
      // beaconDelay = BEACON_DEADTIME;
    // }
  // }
  // TCCR1B ^= (1 << ICES1); // change trigger edge
// }


// ***** MAIN LOOP FUNCTIONS *****
#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_1HZ 100

uint32_t currTime = 0;
uint32_t prevTime = micros();
uint32_t deltaTime = 0;
uint16_t frameCounter = 0;

#ifdef USE_GPS
extern struct gpsData gpsData; 
extern GeodeticPosition currentPosition;

extern uint16_t gps_counter;
extern uint8_t sec_counter;
extern uint8_t initialLock;

#endif



void setup(void)
{
	//RF module pins
	pinMode(SDO_pin, INPUT);   //SDO
	pinMode(SDI_pin, OUTPUT);   //SDI
	pinMode(SCLK_pin, OUTPUT);   //SCLK
	pinMode(IRQ_pin, INPUT);   //IRQ
	pinMode(nSel_pin, OUTPUT);   //nSEL

	//LED and other interfaces
	pinMode(Red_LED, OUTPUT);   //RED LED
	pinMode(Green_LED, OUTPUT);   //GREEN LED

	// pinMode(PPM_IN, INPUT);   //PPM from TX
	//digitalWrite(PPM_IN, HIGH); // enable pullup for TX:s with open collector output

	Serial.begin(SERIAL_BAUD_RATE);

	//setupPPMinput();
	//pinMode(11,OUTPUT);

	sei();

	beaconDelay = BEACON_DEADTIME;

	#ifdef USE_GPS
	delay(1000);
	//Serial.begin(115200);
	initializeGps();
	#endif
}

void loop(void)
{
	currTime = micros();
	deltaTime = ( currTime - prevTime );
	
	if( deltaTime >= 10000 )
	{
		frameCounter++;
		loop100Hz();
		
		if( frameCounter % TASK_10HZ == 0 )
		{
			loop10Hz();
		}
		if( frameCounter % TASK_5HZ == 0 )
		{
			loop5Hz();
		}
		if( frameCounter % TASK_1HZ == 0 )
		{
			loop1Hz();
		}
		
		if( frameCounter >= 100) {
			frameCounter = 0;
		}
		prevTime = currTime;
	}
}

void loop100Hz(void)
{
	#ifdef USE_GPS
	updateGps();
	#endif
}

void loop50Hz(){}

void loop10Hz(){}

void loop5Hz(void)
{
	if(checkBeaconRSSI())
	{
		beaconDelay = BEACON_RSSI_TRIGGER_DELAY;
		#ifdef USE_GPS && defined GPS_SEND_EVERY
		gps_counter = GPS_SEND_EVERY;
		#endif
	}
}

void loop1Hz(void)
{
	#ifdef USE_GPS
	if(initialLock == 0 && haveAGpsLock())
	{
		initialLock = 1;
	}
	#endif
	if (0 == beaconDelay) 
	{
		Red_LED_ON;
		Green_LED_ON;

		#ifdef USE_GPS
		//sendGPS();
		vtest();
		#else
		// rfm_init();
		// rfm_tx();
		// beaconSquelch();
		// beaconCE3K();	

		
		vtest();
		// rfm_deinit();
		#endif
		
		Green_LED_OFF;
		Red_LED_OFF;
		beaconDelay = BEACON_INTERVAL;
	} 
	else 
	{
			Red_LED_ON;
			#ifdef USE_GPS
			if(gpsData.state > 0){
				Green_LED_ON;
			}			
			#endif
			delay(2);
			Red_LED_OFF;
			Green_LED_OFF;
			beaconDelay--;
	}
}
