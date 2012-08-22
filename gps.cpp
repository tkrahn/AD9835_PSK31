/////////////////////////////////////////////////////////////////
// AD9835_PSK31
//
// Simple 20m PSK31 beacon transmitting GPS coordinates using 
// an ATMega328 and an AD9835 DDS
//
// by Thomas Krahn KT5TK / DL4MDW (2012)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//


/* 
 * Original from trackuino copyright (C) 2010  EA5HAV Javi
 * Modified 2012 to include uBlox mode setting by VK5QI
 * Modified 2012 by KT5TK for use in PSK31 beacon
 *
 */

#define DEBUG_GPS

#include "config.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "gps.h"

// Module declarations
static void parse_sentence_type(const char * token);
static void parse_time(const char *token);
static void parse_status(const char *token);
static void parse_lat(const char *token);
static void parse_lat_hemi(const char *token);
static void parse_lon(const char *token);
static void parse_lon_hemi(const char *token);
static void parse_speed(const char *token);
static void parse_course(const char *token);
static void parse_altitude(const char *token);
static void parse_satellites(const char *token);

// Module types
typedef void (*t_nmea_parser)(const char *token);

enum t_sentence_type {
  SENTENCE_UNK,
  SENTENCE_GGA,
  SENTENCE_RMC
};

// Module constants
static const t_nmea_parser unk_parsers[] = {
  parse_sentence_type,    // $GPxxx
};

static const t_nmea_parser gga_parsers[] = {
  NULL,             // $GPGGA
  parse_time,       // Time
  NULL,             // Latitude
  NULL,             // N/S
  NULL,             // Longitude
  NULL,             // E/W
  NULL,             // Fix quality 
  parse_satellites, // Number of satellites
  NULL,             // Horizontal dilution of position
  parse_altitude,   // Altitude
  NULL,             // "M" (mean sea level)
  NULL,             // Height of GEOID (MSL) above WGS84 ellipsoid
  NULL,             // "M" (mean sea level)
  NULL,             // Time in seconds since the last DGPS update
  NULL              // DGPS station ID number
};

static const t_nmea_parser rmc_parsers[] = {
  NULL,             // $GPRMC
  parse_time,       // Time
  parse_status,     // A=active, V=void
  parse_lat,        // Latitude,
  parse_lat_hemi,   // N/S
  parse_lon,        // Longitude
  parse_lon_hemi,   // E/W
  parse_speed,      // Speed over ground in knots
  parse_course,     // Track angle in degrees (true)
  NULL,             // Date (DDMMYY)
  NULL,             // Magnetic variation
  NULL              // E/W
};

static const int NUM_OF_UNK_PARSERS = (sizeof(unk_parsers) / sizeof(t_nmea_parser));
static const int NUM_OF_GGA_PARSERS = (sizeof(gga_parsers) / sizeof(t_nmea_parser));
static const int NUM_OF_RMC_PARSERS = (sizeof(rmc_parsers) / sizeof(t_nmea_parser));

// Module variables
static t_sentence_type sentence_type = SENTENCE_UNK;
static bool at_checksum = false;
static unsigned char our_checksum = '$';
static unsigned char their_checksum = 0;
static char token[16];
static int num_tokens = 0;
static unsigned int offset = 0;
static bool fix_ok = false;
static char gga_time[7], rmc_time[7];
static char new_time[7];
static float new_lat;
static float new_lon;
static char new_aprs_lat[9];
static char new_aprs_lon[10];
static float new_course;
static float new_speed;
static float new_altitude;
static char new_satellites = 0;

// Public (extern) variables, readable from other modules
char gps_time[7];       // HHMMSS
float gps_lat = 0;
float gps_lon = 0;
char gps_aprs_lat[9];
char gps_aprs_lon[10];
float gps_course = 0;
float gps_speed = 0;
float gps_altitude = 0;
char gps_fix = false;
char gps_satellites = 0;

// Module functions
unsigned char from_hex(char a) 
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else if (a >= '0' && a <= '9')
    return a - '0';
  else
    return 0;
}

void parse_sentence_type(const char *token)
{
  if (strcmp(token, "$GPGGA") == 0) {
    sentence_type = SENTENCE_GGA;
  } else if (strcmp(token, "$GPRMC") == 0) {
    sentence_type = SENTENCE_RMC;
  } else {
    sentence_type = SENTENCE_UNK;
  }
}

void parse_time(const char *token)
{
  // Time can have decimals (fractions of a second), but we only take HHMMSS
  strncpy(new_time, token, 6);
}

void parse_status(const char *token)
{
  // "A" = active, "V" = void. We shoud disregard void sentences
  if (strcmp(token, "A") == 0)
    fix_ok = true;
  else
    fix_ok = false;
}

void parse_lat(const char *token)
{
  // Parses latitude in the format "DD" + "MM" (+ ".M{...}M")
  char degs[3];
  if (strlen(token) >= 4) {
    degs[0] = token[0];
    degs[1] = token[1];
    degs[2] = '\0';
    new_lat = atof(degs) + atof(token + 2) / 60;
  }
  // APRS-ready latitude
  strncpy(new_aprs_lat, token, 7);
}

void parse_lat_hemi(const char *token)
{
  if (token[0] == 'S')
    new_lat = -new_lat;
  new_aprs_lat[7] = token[0];
  new_aprs_lon[8] = '\0';
}

void parse_lon(const char *token)
{
  // Longitude is in the format "DDD" + "MM" (+ ".M{...}M")
  char degs[4];
  if (strlen(token) >= 5) {
    degs[0] = token[0];
    degs[1] = token[1];
    degs[2] = token[2];
    degs[3] = '\0';
    new_lon = atof(degs) + atof(token + 3) / 60;
  }
  // APRS-ready longitude
  strncpy(new_aprs_lon, token, 8);
}

void parse_lon_hemi(const char *token)
{
  if (token[0] == 'W')
    new_lon = -new_lon;
  new_aprs_lon[8] = token[0];
  new_aprs_lon[9] = '\0';
}

void parse_speed(const char *token)
{
  new_speed = atof(token);
}

void parse_course(const char *token)
{
  new_course = atof(token);
}

void parse_altitude(const char *token)
{
  new_altitude = atof(token);
}

void parse_satellites(const char *token)
{
  /* convert our token to an integer */
  int temp = atoi(token);

  /* range check it's between 0-32 satellites */  
  if(temp < 0)
    temp = 0;
    
  if(temp > 32)
    temp = 32;

  /* store value away */
  new_satellites = (char)temp;
}

//
// Exported functions
//
void gps_setup()
{
  strcpy(gps_time, "000000");  
  strcpy(gps_aprs_lat, "0000.00N");
  strcpy(gps_aprs_lon, "00000.00E");

  pinMode(GPS_RESET_PIN, OUTPUT);
  digitalWrite(GPS_RESET_PIN, HIGH);
  pinMode(GPS_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, HIGH); // keep Venus module off for a second
  delay(1000);
  digitalWrite(GPS_POWER_PIN, LOW);  // Now turn Venus on.
  delay(1000);
  digitalWrite(GPS_RESET_PIN, LOW);  // Reset the Venus GPS to make 100% sure it starts properly
  delay(100);
  digitalWrite(GPS_RESET_PIN, HIGH);
  pinMode(GPS_RESET_PIN, INPUT);



/*
  // Set the navigation mode (Airborne, 1G)
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  //getUBX_ACK(setNav);
  
*/  
}

bool gps_decode(char c)
{
  int ret = false;

  switch(c) {
    case '\r':
    case '\n':
      // End of sentence

      if (num_tokens && our_checksum == their_checksum) {
#ifdef DEBUG_GPS
        Serial.print(" (OK!)");
#endif
        // Return a valid position only when we've got two rmc and gga
        // messages with the same timestamp.
        switch (sentence_type) {
          case SENTENCE_UNK:
            break;    // Keeps gcc happy
          case SENTENCE_GGA:
            strcpy(gga_time, new_time);
            break;
          case SENTENCE_RMC:
            strcpy(rmc_time, new_time);
            break;
        }

        // Valid position scenario:
        //
        // 1. The timestamps of the two previous GGA/RMC sentences must match.
        //
        // 2. We just processed a known (GGA/RMC) sentence. Suppose the
        //    contrary: after starting up this module, gga_time and rmc_time
        //    are both equal (they're both initialized to ""), so (1) holds
        //    and we wrongly report a valid position.
        //
        // 3. The GPS has a valid fix. For some reason, the Venus 634FLPX
        //    reports 24 deg N, 121 deg E (the middle of Taiwan) until a valid
        //    fix is acquired:
        //
        //    $GPGGA,120003.000,2400.0000,N,12100.0000,E,0,00,0.0,0.0,M,0.0,M,,0000**69 (OK!)
        //    $GPGSA,A,1,,,,,,,,,,,,,0.0,0.0,0.0**30 (OK!)
        //    $GPRMC,120003.000,V,2400.0000,N,12100.0000,E,000.0,000.0,280606,,,N**78 (OK!)
        //    $GPVTG,000.0,T,,M,000.0,N,000.0,K,N**02 (OK!)

        if (sentence_type != SENTENCE_UNK &&      // Known sentence?
            strcmp(gga_time, rmc_time) == 0 &&    // RMC/GGA times match?
            fix_ok) {                             // Valid fix?
          // Atomically merge data from the two sentences
          strcpy(gps_time, new_time);
          gps_lat = new_lat;
          gps_lon = new_lon;
          strcpy(gps_aprs_lat, new_aprs_lat);
          strcpy(gps_aprs_lon, new_aprs_lon);
          gps_course = new_course;
          gps_speed = new_speed;
          gps_altitude = new_altitude;
          gps_satellites = new_satellites;
          ret = true;
        }

        /* update the GPS fix flag */
        gps_fix = (fix_ok) ? 1 : 0;
      }
#ifdef DEBUG_GPS
      if (num_tokens)
        Serial.println();
#endif
      at_checksum = false;        // CR/LF signals the end of the checksum
      our_checksum = '$';         // Reset checksums
      their_checksum = 0;
      offset = 0;                 // Prepare for the next incoming sentence
      num_tokens = 0;
      sentence_type = SENTENCE_UNK;
      break;
    
    case '*':
      // Begin of checksum and process token (ie. do not break)
      at_checksum = true;
      our_checksum ^= c;
#ifdef DEBUG_GPS
      Serial.print(c);
#endif

    case ',':
      // Process token
      token[offset] = '\0';
      our_checksum ^= c;  // Checksum the ',', undo the '*'

      // Parse token
      switch (sentence_type) {
        case SENTENCE_UNK:
          if (num_tokens < NUM_OF_UNK_PARSERS && unk_parsers[num_tokens])
            unk_parsers[num_tokens](token);
          break;
        case SENTENCE_GGA:
          if (num_tokens < NUM_OF_GGA_PARSERS && gga_parsers[num_tokens])
            gga_parsers[num_tokens](token);
          break;
        case SENTENCE_RMC:
          if (num_tokens < NUM_OF_RMC_PARSERS && rmc_parsers[num_tokens])
            rmc_parsers[num_tokens](token);
          break;
      }

      // Prepare for next token
      num_tokens++;
      offset = 0;
#ifdef DEBUG_GPS
      Serial.print(c);
#endif
      break;

    default:
      // Any other character
      if (at_checksum) {
        // Checksum value
        their_checksum = their_checksum * 16 + from_hex(c);
      } else {
        // Regular NMEA data
        if (offset < 15) {  // Avoid buffer overrun (tokens can't be > 15 chars)
          token[offset] = c;
          offset++;
          our_checksum ^= c;
        }
      }
#ifdef DEBUG_GPS
      Serial.print(c);
#endif
  }
  return ret;
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(unsigned char *MSG, uint8_t len)
{
        for(int i=0; i<len; i++)
                Serial.write(MSG[i]);
//    Serial.print(MSG[i], BYTE);

}
 
// Calculate expected UBX ACK packet and parse UBX response from GPS
int getUBX_ACK(unsigned char *MSG)
{
  unsigned char b;
  unsigned char ackByteID = 0;
  unsigned char ackPacket[10];
  int startTime = millis();
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];        // ACK class
  ackPacket[7] = MSG[3];        // ACK id
  ackPacket[8] = 0;             // CK_A
  ackPacket[9] = 0;             // CK_B
 
  // Calculate the checksums
  for (unsigned char i=2; i<8; i++)
  {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1)
  {
    // Test for success
    if (ackByteID > 9)
      return true;
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000)
      return false;
 
    // Make sure data is available to read
    if (Serial.available())
    {
      b = Serial.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID])
      { 
        ackByteID++;
      }
      else
      {
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
  }
}

