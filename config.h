#ifndef _CONFIG_H_
#define _CONFIG_H_

// #define BOARD_TYPE 3	// OpenLRS Rx v2 Board or OrangeRx UHF RX
#define BOARD_TYPE 5

//###### SERIAL PORT SPEED - just debugging atm. #######
#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed

// #define BEACON_FREQUENCY US_FRS_CH(1)
#define BEACON_FREQUENCY 438000000

// power levels with RFM22B
// 7 - 100mW
// 6 - 50mW
// 5 - 25mW
// 4 - 13mW
// 3 - 6mW
// 2 - 3mW
// 1 - 1.6mW
// 0 - 1.3mW
#define BEACON_POWER_LEVEL 0x06

#define BEACON_DEADTIME 0 // time to wait until going into beacon mode (s)
#define BEACON_INTERVAL 120 // interval between beacon transmits (s)
#define BEACON_RSSI_TRIGGER_DELAY 2 // time delay (in sec) after receiving RSSI trigger & before tx'ing beacon
#define BEACON_RSSI_TRIGGER_THRESHOLD 20 // rssi threshold value needed to trigger beacon

#define SQUELCH_TONE_HZ 10  // frequency (in hertz) of sub-audible tone used to open squelch
#define SQUELCH_OPEN_LEN 100 // length of squelch open tone transmission
#define SQUELCH_OPEN_DELAY 500 // time delay (in ms) after opening squelch & before begining beacon tx

#define RSSI_MAX 240

#define CALLSIGN "KK6SGI"

// GPS MORSE BEACON
// USAGE NOTES:
// - GPS coords are sent separately
// ** lat is sent 1st, indicated by high-pitched tone sequence ("bee-boop")
// ** lon is sent 2nd, indicated by low-pitched tone sequence ("boo-beep")
// - fix tone (if enabled) is sent next, followed by morse code sequence
// - transmissions can be manually initiated by tx'ing on beacon frequency within range of beacon
// - GREEN LED will flash @ 1Hz (along w/ red LED) if GPS packets are being received & processed

#define USE_GPS				// enable for GPS morse beacon, disable for standard 'close encounters' power ladder beacon
#define GPS_NO_FIX_MSG "NO FIX"				// message sent prior to valid fix
// #define GPS_USE_CALLSIGN "CSRF"
#define GPS_FIX_TONE				// send fix state tone: high-pitch = 3D fix / tx current coords :: low-pitch = no 3D lock / tx "last known coords"
#define GPS_FLOAT				// send float values instead of scaled integers
#define GPS_LAT_OFFSET  0 // 5.65*10000000				// add a 'privacy' offset to latitude value
#define GPS_LON_OFFSET 0 // 10.85*10000000				// same as above, offset for longitude value
#define GPS_SEND_EVERY 3
#define GPS_SERIAL Serial

//#define DEBUG_ENCODE		// enable serial debug of morse encoding
#define MORSE_TONE_HZ 750	// frequency of morse tone; spec says 600-800Hz is standard for CW comm's
#define MORSE_WPM 30			// set morse speed in words-per-minute
// #define MORSE_DOT_MS 85	// set morse unit ('dot') speed in millisec's


// Servovalues considered 'good' i.e. beacon will not activate when it is fed
// with PWM within these limits (feed via ch4 connector)
#define MINPWM 1000
#define MAXPWM 1500

#endif