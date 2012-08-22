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

#ifndef __GPS_H__
#define __GPS_H__

extern char gps_time[7];       // HHMMSS
extern char gps_date[7];       // DDMMYY
extern float gps_lat;
extern float gps_lon;
extern char gps_aprs_lat[9];    //DDMM.MMH0
extern char gps_aprs_lon[10];   //DDDMM.MMH0
extern float gps_course;
extern float gps_speed;
extern float gps_altitude;
extern char gps_fix;
extern char gps_satellites;

void gps_setup();
bool gps_decode(char c);
void sendUBX(unsigned char *MSG, unsigned char len);
int getUBX_ACK(unsigned char *MSG);

#endif

