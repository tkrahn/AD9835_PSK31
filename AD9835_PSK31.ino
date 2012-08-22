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


// This part of the code was based on:
//
// Play around with the AD9835 DDS breakout board from SparkFun
// http://www.sparkfun.com/products/9169
//
// by Mark J. Blair, NF6X
//
// Some bits taken from tjfreebo's comment post on 3/22/2011
//
// License: Do whatever you want with this. No warranty. Thppppt.


#include <SPI.h>
#include <math.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/crc16.h>
#include "config.h"
#include "varicode.h"   // include varicode table
#include "gps.h"


volatile int wd_counter;
bool gpsIsOn;
bool newPositionStillUnknown;
int sentence_id = 1;

const unsigned int LUTsize = 1<<8; // Look Up Table size: has to be power of 2 so that the modulo LUTsize
                                   // can be done by picking bits from the phase avoiding arithmetic
int8_t sintable[LUTsize] PROGMEM = { // already biased with +127
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,
  176,179,182,184,187,190,193,195,198,200,203,205,208,210,213,215,
  217,219,221,224,226,228,229,231,233,235,236,238,239,241,242,244,
  245,246,247,248,249,250,251,251,252,253,253,254,254,254,254,254,
  255,254,254,254,254,254,253,253,252,251,251,250,249,248,247,246,
  245,244,242,241,239,238,236,235,233,231,229,228,226,224,221,219,
  217,215,213,210,208,205,203,200,198,195,193,190,187,184,182,179,
  176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,
  127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,
  78,75,72,70,67,64,61,59,56,54,51,49,46,44,41,39,
  37,35,33,30,28,26,25,23,21,19,18,16,15,13,12,10,
  9,8,7,6,5,4,3,3,2,1,1,0,0,0,0,0,
  0,0,0,0,0,0,1,1,2,3,3,4,5,6,7,8,
  9,10,12,13,15,16,18,19,21,23,25,26,28,30,33,35,
  37,39,41,44,46,49,51,54,56,59,61,64,67,70,72,75,
  78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124
};



String beacon_text;


const int timerPrescale=1<<9;

struct Psk31Encoder
{
   int8_t send_ok;
   int8_t doing_char;
   int8_t bit_wait;
   int8_t bit_send;
   int8_t mbit_send;
   int16_t b_count;
   int8_t last_bit;
   char v_byte;
   unsigned int idx;
   int8_t mbit_flag;
   int8_t OUT_BIT;
   unsigned long fracfreq;
} psk1;

struct oscillator
{
  uint32_t phase;
  int32_t phase_increment;
  int32_t frequency_increment;
  int16_t amplitude;
  int16_t amplitude_increment;
  uint32_t framecounter; 
  uint32_t phase_counter; 
} o1;


const int fractionalbits = 16; // 16 bits fractional phase
// compute a phase increment from a frequency
unsigned long phaseinc(float frequency_in_Hz)
{
   return LUTsize *(1l<<fractionalbits)* frequency_in_Hz/(F_CPU/timerPrescale);
}

// The above requires floating point and is robust for a wide range of parameters
// If we constrain the parameters and take care we can go much
// faster with integer arithmetic
// We control the calculation order to avoid overflow or resolution loss
 //
 // we chose "predivide" so that (pow(2,predivide) divides F_CPU,so 4MHz (1.7v), 8Mhz, 12Mhz (3.3v) and 16Mhz 20Mhz all work
 // AND note that "frequency_in_Hz" is not too large. We only have about 16Khz bandwidth to play with on
 // Arduino timers anyway
const int predivide = 8;
unsigned long phaseinc_from_fractional_frequency(unsigned long frequency_in_Hz_times_256)
{
    return (1l<<(fractionalbits-predivide))* ((LUTsize*(timerPrescale/(1<<predivide))*frequency_in_Hz_times_256)/(F_CPU/(1<<predivide)));

}

// DDS clock frequency (Hz)
#define FCLK 50000000

// Phase and frequency calculation debugging
// #define DEBUG_CALC

// Write directly to AVR port in SPIwrite() instead of using digitalWrite()?
#define FAST_IO


// Timer setup constants

#if defined(__AVR_ATmega8__)

// On old ATmega8 boards, output is on pin 11
#define PWM_PIN       11
#define PWM_VALUE_DESTINATION     OCR2
#define PWM_INTERRUPT TIMER2_OVF_vect
#elif defined(__AVR_ATmega1280__)

#define PWM_PIN       3
#define PWM_VALUE_DESTINATION     OCR3C
#define PWM_INTERRUPT TIMER3_OVF_vect
#else

// For modern ATmega168 boards, output is on pin 3
#define PWM_PIN       3
#define PWM_VALUE_DESTINATION     OCR2B
#define PWM_INTERRUPT TIMER2_OVF_vect
#endif




// SPI write to AD9835 register
void SPIwrite(int byte1, int byte2) {
    // take the FSYNC pin low to select the chip:
#ifdef FAST_IO
    PORTB &= ~0x02;
#else
    digitalWrite(FSYNCpin, LOW);
#endif

    //  send in the address and value via SPI:
    SPI.transfer(byte1);
    SPI.transfer(byte2);

    // take the FSYNC pin high to de-select the chip:
#ifdef FAST_IO
    PORTB |= 0x02;
#else
    digitalWrite(FSYNCpin, LOW);
#endif
}


// Calculate the frequency tuning word for a DDS with a 32 bit
// phase register. Desired frequency and DDS clock frequency
// specified in Hz.
unsigned long calcFTW32(unsigned long freq, unsigned long fclk) {

    unsigned long FTW;
    double FTWf;

    // This calculation loses precision on the Arduino because doubles
    // are presently implemented as floats. So, for example,
    // calcFTW32(10000000,50000000) should return 0x33333333, but it
    // instead returns 0x33333340 on the Arduino. A good future improvement
    // to this code would be to implement an efficient calculation that maintains
    // the full 32 bit FTW precision for some desired resolution (say, 1 Hz).
    FTWf = pow(2,32) * (double)freq / (double)fclk;
    FTW = (unsigned long) FTWf;
   
#ifdef DEBUG_CALC
    Serial.print("calcFTW32(");
    Serial.print(freq);
    Serial.print(", ");
    Serial.print(fclk);
    Serial.print(") = 0x");
    Serial.println(FTW, HEX);
#endif

    return FTW;
}


// Calculate the phase tuning word for a DDS with a 12 bit
// phase offset register. Desired phase specified in Hz (0-359)
unsigned int calcPTW12d(unsigned int deg) {

    unsigned int PTW;
    double PTWf;
   
    PTWf = ((double)(deg % 360) / 360.0) * pow(2,12);
    PTW = (unsigned int) PTWf;
   
#ifdef DEBUG_CALC
    Serial.print("calcPTW12d(");
    Serial.print(deg);
    Serial.print(") = 0x");
    Serial.println(PTW, HEX);
#endif

    return PTW;
}


// Write to speficied frequency register (0-1) of AD9835
void writeFTW(unsigned long FTW, int reg) {
    int regaddr;
    regaddr = (reg & 0x01) << 2;

    SPIwrite(0x33 + regaddr, ((FTW & 0xFF000000) >> 24));
    SPIwrite(0x22 + regaddr, ((FTW & 0x00FF0000) >> 16));
    SPIwrite(0x31 + regaddr, ((FTW & 0x0000FF00) >> 8));
    SPIwrite(0x20 + regaddr, ((FTW & 0x000000FF)));
}


// Write to specified phase register (0-3) of AD9835
void writePTW(unsigned int PTW, int reg) {
    int regaddr;
    regaddr = (reg & 0x03) << 1;

    SPIwrite(0x19 + regaddr, ((PTW & 0x0F00) >> 8));
    SPIwrite(0x08 + regaddr, ((PTW & 0x00FF)));
}


// the following function interfaces with the ISR and the send_varichar() function and
// actually diddles the bit sending data
void bitsend(void)
{
   if(psk1.send_ok) {                       // is there a bit ready to be sent?
      //Serial.println("send_ok"); 
      if(!psk1.bit_send)  {                 // yes - is it a zero?
         //Serial.println("bit_send"); 
         psk1.OUT_BIT = !psk1.OUT_BIT;      // toggle send bit (This represents a zero)
         if (psk1.OUT_BIT){
           //Serial.println("OUT_BIT HIGH"); 
           //digitalWrite(PskOutPin, HIGH);
           digitalWrite(PSEL1pin, HIGH);
           //Serial.println("OUT_BIT written");
         }
         else {
           //Serial.println("OUT_BIT LOW");
           //digitalWrite(PskOutPin, LOW);
           digitalWrite(PSEL1pin, LOW);
           //Serial.println("OUT_BIT written");
         }
      }
      psk1.send_ok = 0;                    // clear the send-bit flag
      //Serial.println("send_ok reset"); 
      psk1.bit_wait = 1;                   // let it be known that the current bit has been sent...
      //Serial.print("bit_wait = ");
      //Serial.println(psk1.bit_wait, DEC);
      
   }
}

// the following function shifts out the bits and returns them in 'bit_send'
void sendbit(void)
{
   psk1.bit_wait = 0;                            // reset bit indicator
   psk1.last_bit = psk1.bit_send;                // get previous bit
   psk1.bit_send = bitRead(psk1.v_byte, 7);      // get the current bit to send
   //Serial.print("bit_send = ");
   //Serial.print(psk1.bit_send, DEC);
   //Serial.print("; ");
   if(!psk1.mbit_flag) {                         // copy current bit if it hasn't been done before...
      psk1.mbit_flag = 1;
      psk1.mbit_send = psk1.bit_send;            // actually copy the bit...
   }
   psk1.b_count++;                               // bump bit count
   //Serial.print("b_count = ");
   //Serial.println(psk1.b_count, DEC);
   psk1.v_byte <<= 1;                            // shift the current varicode byte left 1 bit
   if((!psk1.last_bit) && (!psk1.bit_send))   {  // are the past two recent bits both zero?
      //Serial.println("00");
      psk1.doing_char = 0;                       // signal that this is the last bit of this character
   }
   if(psk1.b_count > 7)   {                      // if this is the last bit of this word, get the next one...
      psk1.idx++;                                // look at the next bit of the
      psk1.v_byte = varicode[psk1.idx];          // get the remaining bits of the varicode
      //Serial.print("2nd byte = ");
      //Serial.println(psk1.v_byte, BIN);
      psk1.b_count = 0;                          // reset the bit counter
   }
   psk1.mbit_flag = 0;                           // clear flag in preparation for next bit
}


void initializeTimer() {
 // Set up PWM  with Clock/256 (i.e.  31.25kHz on Arduino 16MHz;
 // and  phase accurate

#if defined(__AVR_ATmega8__)
  // ATmega8 has different registers
  TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
  TIMSK = _BV(TOIE2);
#elif defined(__AVR_ATmega1280__)
  TCCR3A = _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TIMSK3 = _BV(TOIE3);
#else
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
#endif
//  pinMode(PWM_PIN,OUTPUT);
}

// the following function converts the input character (c) into the varicode character to be
void lookup_varichar(char c)
{
   if(c > 127)    {                 // is it out of the valid character range?
      psk1.doing_char = 0;          // clear flag to allow next character to be processed
      return;                       // yes - ignore the character
   }
   else {                           // in the valid character range - look it up
      psk1.idx = c;
      psk1.idx += c;                // double the index
      psk1.v_byte = varicode[psk1.idx];  // get bits of varicode
      psk1.doing_char = 1;          // indicate that a character is ready to be sent
      psk1.b_count = 0;             // init bit counter
      psk1.bit_send = 1;            // init holder for current bit
      psk1.last_bit = 1;            // init holder for the last bit sent
   }
}




void send_message()
{
  
  Serial.print(beacon_text);
  
  o1.phase_increment = phaseinc_from_fractional_frequency(psk1.fracfreq);  // PSK has 31.25 Hz * 256 = 8000
  
  //psk1.fracfreq += 1; // Increase fractional frequency each cycle by 1 Hz for testing
  
  char c = 32; // space char. Irrelevant as long as > 0.
  int cnt = 0; // Pointer to current letter in string
  
  for(cnt = 0; cnt < 40; cnt++) {     // send some zeroes before each message playback cycle
    psk1.bit_send = 0;
    psk1.mbit_send = 0;
    psk1.bit_wait = 0;
    while(!psk1.bit_wait)  {
      //Serial.println("calling bitsend()");
      bitsend();
      //Serial.println("bitsend() done");
      //Serial.print("bit_wait = ");
      //Serial.println(psk1.bit_wait, DEC);
    }
  }
  
  cnt = 0;
  
  
  
  while (c > 0) //exit when string is terminated by a zero
  {
    c = beacon_text.charAt(cnt);
    //if((c > 31) && (c < 127)) { Serial.print(c);} // Print printable characters through serial port
    cnt++;
    lookup_varichar(c);
    while(psk1.doing_char) {              // hang around here until the character is done
      sendbit();
      while(!psk1.bit_wait)  {
        bitsend();                        // hang around until the current bit is sent
      }
    }
    
  }
}  

uint16_t gps_CRC16_checksum (char *string)
{
	size_t i;
	uint16_t crc;
	uint8_t c;
 
	crc = 0xFFFF;
 
	// Calculate checksum ignoring the \n and the two $s
	for (i = 3; i < strlen(string); i++)
	{
		c = string[i];
		crc = _crc_xmodem_update (crc, c);
	}
 
	return crc;
}

void print_string_in_hex(char *string)
{
	size_t i;
	uint16_t crc;
	uint8_t c;

        Serial.print("hex: ");
	for (i = 0; i < (1 + strlen(string)); i++)
	{
		c = string[i];
		Serial.print(c, HEX);
                Serial.print(" ");
	}
        Serial.println();
}  


//void disable_bod_and_sleep()
//{
  /* This will turn off brown-out detection while
   * sleeping. Unfortunately this won't work in IDLE mode.
   * Relevant info about BOD disabling: datasheet p.44
   *
   * Procedure to disable the BOD:
   *
   * 1. BODSE and BODS must be set to 1
   * 2. Turn BODSE to 0
   * 3. BODS will automatically turn 0 after 4 cycles
   *
   * The catch is that we *must* go to sleep between 2
   * and 3, ie. just before BODS turns 0.
   */
//  unsigned char mcucr;

//  cli();
//  mcucr = MCUCR | (_BV(BODS) | _BV(BODSE));
//  MCUCR = mcucr;
//  MCUCR = mcucr & (~_BV(BODSE));
//  sei();
//  sleep_mode();    // Go to sleep
//}

void power_save()
{
  /* Enter power saving mode. SLEEP_MODE_IDLE is the least saving
   * mode, but it's the only one that will keep the UART running.
   * In addition, we need timer0 to keep track of time, timer 1
   * to drive the buzzer and timer2 to keep pwm output at its rest
   * voltage. TK: changed to conditional SLEEP_MODE_PWR_DOWN
   */
//   if (! modem_busy()) {  // Don't sleep if we're txing.
  
    if (newPositionStillUnknown == true)
    {

      // Here we still need the serial port to
      // capture GPS NMEA data
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_enable();
      power_adc_disable();
      power_spi_disable();
      power_twi_disable();
      power_timer1_disable();
      digitalWrite(LED_PIN, LOW);
      sleep_mode();    // Go to sleep

    }
    else
    {
      //Serial.end();    
      // Now that we know the position, the serial 
      // port is no longer needed
      // and we can go to full SLEEP_MODE_PWR_DOWN.
      // Only the watchdog interrupt will wake us up.
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    
      // Shut down GPS hardware
    
      digitalWrite(GPS_POWER_PIN, HIGH);
      delay(500);   // Give time to discharge power capacitor of GPS board   
      gpsIsOn = false;
      sleep_enable();
      power_adc_disable();
      power_spi_disable();
      power_twi_disable();
      power_timer1_disable();

      digitalWrite(LED_PIN, LOW);
      //disable_bod_and_sleep();    // Go to sleep
      sleep_mode();    // Go to sleep
    }
  

    digitalWrite(LED_PIN, HIGH);
  
    sleep_disable();  // Resume after wake up
    power_all_enable();

//  }
}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) 
{
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
//  Serial.println(ww);
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}

//****************************************************************  
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) 
{
  wd_counter++; // count the seconds for the sleep period
}

float meters_to_feet(float m)
{
  // 10000 ft = 3048 m
  return m / 0.3048;
}

void blink3() // for debug
{
  
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
    
}

// Initialization code
void setup() {

  
    Serial.begin(BAUD);

#ifdef DEBUG_CALC
    Serial.println("reset");
#endif

    digitalWrite(FSYNCpin, HIGH);
    digitalWrite(SCLKpin,  LOW);
    digitalWrite(FSELpin,  LOW);
    digitalWrite(PSEL0pin, LOW);
    digitalWrite(PSEL1pin, LOW);
    pinMode(FSYNCpin, OUTPUT);
    pinMode(SCLKpin,  OUTPUT);
    pinMode(FSELpin,  OUTPUT);
    pinMode(PSEL0pin, OUTPUT);
    pinMode(PSEL1pin, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.begin();

    // Reset and clear device
    SPIwrite(0xF8, 0x00);
    delay(1);
 
    // Power up device
    SPIwrite(0xC0, 0x00);

    // Set sync and FSEL source (external pins)
    SPIwrite(0x80, 0x00);

    psk1.send_ok = 0;         
    psk1.doing_char = 0;
    psk1.bit_wait = 0;
    psk1.bit_send = 0;
    psk1.mbit_send = 0;
    psk1.b_count = 0;
    psk1.last_bit = 0;
    psk1.OUT_BIT = 0;
    psk1.fracfreq = 8000UL;

    initializeTimer();
    gps_setup(); 
    gpsIsOn = true;
    newPositionStillUnknown = true;
    setup_watchdog(9); // set watchdog timeout to (approximately) 8 seconds
    wd_counter = (int)(TRANSMIT_PERIOD_SECONDS / 8); // Transmit right away after switching on.
   
    
    // Set up frequency registers for toggle
    // between 5MHz and 14.071 MHz with FSEL pin   
    writeFTW(calcFTW32(    1000, 50000000), 0);
    writeFTW(calcFTW32(14070500, 50000000), 1);

    // Set up phase registers for QPSK with PSEL1, PSEL0 pins
    writePTW(calcPTW12d(0),   0);
    writePTW(calcPTW12d(90),  1);
    writePTW(calcPTW12d(180), 2);
    writePTW(calcPTW12d(270), 3);
    
    o1.amplitude = 255*256; // full amplitude

    blink3();
    
}

// Main loop
void loop() 
{
  int c;
  char temp[120];
  if (wd_counter >= (int)(TRANSMIT_PERIOD_SECONDS / 8)) 
  {
    beacon_text = "\n$$KT5TK,";
    beacon_text = beacon_text + sentence_id + ",";
    
    if (newPositionStillUnknown)
    {
      
      beacon_text = beacon_text + "000000,0.0,0.0,0,No GPS lease... ";
    }
    else
    {
      // we have GPS coordinates

      beacon_text = beacon_text + gps_time + ",";
      
      dtostrf(gps_lat,5,4,temp);
      beacon_text = beacon_text + temp + ",";
      
      dtostrf(gps_lon,5,4,temp);
      beacon_text = beacon_text + temp + ",";


      dtostrf(gps_altitude,1,0,temp);
      beacon_text = beacon_text + temp + ",";
      
//      sprintf(temp, "%ld", (long)((gps_altitude) + 0.5));
//      beacon_text = beacon_text + temp;
    }
    
    
        
    if (sentence_id % 3 == 0) 
    {
      beacon_text = beacon_text + "high altitude balloon"; 
    }
    
    if (sentence_id % 3 == 1)
    {
      beacon_text = beacon_text + "http://kt5tk.tkrahn.com"; 
    }
    
    if (sentence_id % 3 == 2)
    {
      beacon_text = beacon_text + "solar powered"; 
    }
     
    // Add the CRC to the end of the beacon string
    beacon_text.toCharArray( temp, ( 1 + beacon_text.length() ) );
    print_string_in_hex( temp );
    
    sprintf(temp,"*%04X\n", gps_CRC16_checksum( temp ));    
    beacon_text = beacon_text + temp;
    
    digitalWrite(FSELpin,  HIGH); // Set HF signal to TX frequency
    send_message();
    digitalWrite(FSELpin,  LOW);  // Set 1kHz signal (essentially the TX is off)
    wd_counter = 0;
    newPositionStillUnknown = true;
    sentence_id++;
  }    
    
  if (Serial.available() && newPositionStillUnknown) 
  {
    c = Serial.read();
    if (gps_decode(c)) 
    {
      // We have received and decoded our location
      newPositionStillUnknown = false;
      
      blink3();  // Show the user that we have a valid GPS lease
   
    }
  }
  else 
  {
    power_save();
  }
  
  
}


// This is the heart beat of the PSK31 synthesis. 
SIGNAL(PWM_INTERRUPT)
{
   if(o1.phase_counter >= 1000)
   {
     psk1.send_ok = 1;
     o1.phase_counter = 0;
   }
   o1.phase_counter++;
   //o1.phase += (uint32_t)o1.phase_increment;
  
}


