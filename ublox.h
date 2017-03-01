/*
  AeroQuad v3.x - Sep 2012
  www.AeroQuad.com
  Copyright (c) 2012 AeroQuad developers.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
/* simple UBLOX parser */

#ifndef _AQ_ublox_H_
#define _AQ_ublox_H_

static const unsigned char UBX_5HZ[] = {
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 
	0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A 
};

static const uint8_t UBX_MSG_NAV[] = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,           // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,           // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,           // set SOL MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x00, 0x3B, 0xA2,           // disable SVINFO
//    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67            // set VELNED MSG rate
};

static const uint8_t UBX_MSG_NMEA[] = {
    // DISABLE NMEA messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,           // VGS: Course over ground and Ground speed
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,           // GSV: GNSS Satellites in View
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,           // GLL: Latitude and longitude, with time of position fix and status
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,           // GGA: Global positioning system fix data
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,           // GSA: GNSS DOP and Active Satellites
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17            // RMC: Recommended Minimum data
};

// need to set this up
#ifdef UBLOX_PED_MODE
static const uint8_t ubloxInit_NAV5_Pedestrian[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,           // CFG-NAV5 - Set engine settings (original MWII code)
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Pedistrian and
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xC2
};
#endif
#ifdef UBLOX_AIR1G_MODE
static const uint8_t ubloxInit_NAV5_Airborne1G[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Airborne <1G
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x28
};
#endif
#ifdef UBLOX_AIR4G_MODE
static const uint8_t ubloxInit_NAV5_Airborne4G[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Airborne <4G
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x6C
};
#endif

#define UBLOX_5HZ   {UBX_5HZ, sizeof(UBX_5HZ)}
#define UBLOX_MSG_NAV  {UBX_MSG_NAV, sizeof(UBX_MSG_NAV)}
#define UBLOX_MSG_NMEA {UBX_MSG_NMEA, sizeof(UBX_MSG_NMEA)}
#define UBLOX_38400 {(unsigned char *)"$PUBX,41,1,0003,0003,38400,0*24\r\n",0}
#define UBLOX_CONFIGS UBLOX_5HZ, UBLOX_MSG_NMEA, UBLOX_MSG_NAV, UBLOX_38400

// UBLOX binary message definitions
struct ublox_NAV_STATUS { // 01 03 (16)
  uint32_t iTow;
  uint8_t  gpsFix;
  uint8_t  flags;
  uint8_t  fixStat;
  uint8_t  flags2;
  uint32_t ttfx;
  uint32_t msss;
};

struct ublox_NAV_POSLLH { // 01 02 (28)
  uint32_t iTow;
  int32_t lon; // 1e-7 degrees
  int32_t lat; // 1e-7 degrees
  int32_t height; // mm
  int32_t hMSL; // mm
  uint32_t hAcc; //mm
  uint32_t vAcc; //mm
};

// remove/not necessary?
// only used for sat count
 struct ublox_NAV_SOL { // 01 6 (52)
  uint32_t iTow;
  int32_t  fTow;
  int16_t  week;
  uint8_t  gspFix;
  uint8_t  flags;
  int32_t  ecefX;
  int32_t  ecefY;
  int32_t  ecefZ;
  int32_t  pAcc;
  int32_t  ecefVX;
  int32_t  ecefVY;
  int32_t  ecefVZ;
  int32_t  sAcc;
  uint16_t pDOP;
  uint8_t  res1;
  uint8_t  numSV;
  uint32_t res2;
};
 
// not necessary?
/* struct ublox_NAV_VELNED { // 01 12h (36)
  uint32_t iTow;
  int32_t  velN; // cm/s
  int32_t  velE; // cm/s
  int32_t  velD; // cm/s
  uint32_t  speed; // cm/s
  uint32_t  gSpeed; // cm/s
  int32_t  heading; // dev 1e-5
  uint32_t sAcc; // cm/s
  uint32_t cAcc; // deg 1e-5
};
 */
union ublox_message {
  struct ublox_NAV_STATUS nav_status;
  struct ublox_NAV_POSLLH nav_posllh;
  // struct ublox_NAV_VELNED nav_velned;
  struct ublox_NAV_SOL nav_sol;
  unsigned char raw[52];
} ubloxMessage;

unsigned short ubloxExpectedDataLength;
unsigned short ubloxDataLength;
unsigned short ubloxClass,ubloxId;
unsigned char  ubloxCKA,ubloxCKB;

enum ubloxState{ WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID, GET_LL, GET_LH, GET_DATA, GET_CKA, GET_CKB  } ubloxProcessDataState;

// Initialize parser
void ubloxInit() {
  
  ubloxProcessDataState = WAIT_SYNC1;
}

// process complete binary packet
void ubloxParseData() {// uses publib vars

  gpsData.sentences++;
  if (ubloxClass==1) { // NAV
    if (ubloxId==2) { // NAV:POSLLH
      gpsData.lat = ubloxMessage.nav_posllh.lat;
      gpsData.lon = ubloxMessage.nav_posllh.lon;
      gpsData.height = ubloxMessage.nav_posllh.height;
      gpsData.accuracy = ubloxMessage.nav_posllh.hAcc;
      gpsData.fixtime = ubloxMessage.nav_posllh.iTow;
    }
    else if (ubloxId==3) { //NAV:STATUS
      switch (ubloxMessage.nav_status.gpsFix) {
        case 2: 
          gpsData.state = GPS_FIX2D;
          break;
		  
        case 3:
          gpsData.state = GPS_FIX3D;
          break;
		  
        default:
          gpsData.state = GPS_NOFIX;
          break;
      }
    }
    else if (ubloxId==6) { // NAV:SOL
      gpsData.sats = ubloxMessage.nav_sol.numSV;
    }
/*     else if (ubloxId==18) { // NAV:VELNED
      gpsData.course = ubloxMessage.nav_velned.heading / 100; // 10E-5 to millidegrees
      gpsData.speed = ubloxMessage.nav_velned.gSpeed;
    } */
  } 
}

// process serial data
int ubloxProcessData(unsigned char data) {
  
  int parsed = 0;
  
  switch (ubloxProcessDataState) {
  case WAIT_SYNC1:
    if (data == 0xb5) {
      ubloxProcessDataState = WAIT_SYNC2;
    }
    break;
	
  case WAIT_SYNC2:
    if (data == 0x62) {
      ubloxProcessDataState = GET_CLASS;
    }
    else if (data == 0xb5) {
      // ubloxProcessDataState = GET_SYNC2;
    }
    else {
      ubloxProcessDataState = WAIT_SYNC1;
    }
    break;
  case GET_CLASS:
    ubloxClass=data;
    ubloxCKA=data;
    ubloxCKB=data;
    ubloxProcessDataState = GET_ID;
    break;
	
  case GET_ID:
    ubloxId=data;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxProcessDataState = GET_LL;
    break;
	
  case GET_LL:
    ubloxExpectedDataLength = data;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxProcessDataState = GET_LH;
    break;
	
  case GET_LH:
    ubloxExpectedDataLength += data << 8;
    ubloxDataLength=0;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    if (ubloxExpectedDataLength <= sizeof(ubloxMessage)) {
      ubloxProcessDataState = GET_DATA;
    }
    else {
      // discard overlong message
      ubloxProcessDataState = WAIT_SYNC1;
    }
    break;
	
  case GET_DATA:
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    // next will discard data if it exceeds our biggest known msg
    if (ubloxDataLength < sizeof(ubloxMessage)) {
      ubloxMessage.raw[ubloxDataLength++] = data;
    }
    if (ubloxDataLength >= ubloxExpectedDataLength) {
      ubloxProcessDataState = GET_CKA;
    }
    break;
	
  case GET_CKA:
    if (ubloxCKA != data) {
      ubloxProcessDataState = WAIT_SYNC1;
    } 
	else {
      ubloxProcessDataState = GET_CKB;
    }
    break;
	
  case GET_CKB:
    if (ubloxCKB == data) {
      parsed = 1;
      ubloxParseData();
    }
    ubloxProcessDataState = WAIT_SYNC1;
    break;
	
  }
  return parsed;
}

#endif
