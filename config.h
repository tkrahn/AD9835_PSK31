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


#ifndef __CONFIG_H__
#define __CONFIG_H__

  #define BAUD 4800                      // The baud rate for the GPS
  #define TRANSMIT_PERIOD_SECONDS 20     // How often to transmit?
  
  // I/O pin assignments
  #define LED_PIN 3
  #define GPS_POWER_PIN 4
  #define GPS_RESET_PIN 5
  
  #define SCLKpin  13   // SCK
  // 12 = MISO
  #define SDATApin 11   // MOSI
  // 10 = SS
  #define FSYNCpin 9    // PORTB.1
  #define FSELpin  8    // PORTB.0
  #define PSEL1pin 7    // PORTD.7
  #define PSEL0pin 6    // PORTD.6



#endif
