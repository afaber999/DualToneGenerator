
// A simple dual tone generator for testing SSB tranceivers
// Runs on a Arduino Nano 168
// A.L. Faber (c)2021

#include <Arduino.h>
#include "avr/pgmspace.h"

const uint8_t PIN_LED = 13;
const uint8_t PIN_ISR_TIMING = 7;
const uint8_t PIN_LOOP_TIMING = 6;

const uint8_t PIN_TONE_A = 3;
const uint8_t PIN_TONE_B = 11;

// Sine table
const PROGMEM  uint8_t SIN_TABLE[]  = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124
};

// Bit manupilation functions
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// variables both used in ISR and main loop
// therefore make them static volatile
static volatile uint16_t counter;

static volatile uint32_t phaccu_a;
static volatile uint32_t phinc_a;

static volatile uint32_t phaccu_b;
static volatile uint32_t phinc_b;


static volatile uint32_t latest_start;
static volatile uint32_t latest_stop;

// PWM clock is 16 MHz / 1 (prescaler)
// PWM period = 16.000.000 / 256 = 62500 Hz
// Phase accu of 32 bits
// steps per Hz = 2^32 / 62500 = 68719 (rounded)
// phaseInc = steps per Hz * desired freq

uint32_t FrequencyToPhaseIncrement( uint32_t desired_freq) {
    const uint32_t PHASE_STEPS_PER_HZ = 68719;
    return PHASE_STEPS_PER_HZ * desired_freq;
}

//******************************************************************
// Timer2 Interrupt Service at 62500 Hz
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
// runtime : tbd 
ISR(TIMER2_OVF_vect) {

    // use timing pin to measure ISR time with oscope
    sbi(PORTD,PIN_ISR_TIMING);
    latest_start = TCNT2;

    // increment the phase accu
    phaccu_a = phaccu_a + phinc_a;
    phaccu_b = phaccu_b + phinc_b;

    uint8_t accu_increment = (phaccu_a >> 24);
    OCR2A=pgm_read_byte_near(SIN_TABLE + accu_increment); 
    
    accu_increment = (phaccu_b >> 24);
    OCR2B=pgm_read_byte_near(SIN_TABLE + accu_increment); 

    counter++;

    cbi(PORTD,PIN_ISR_TIMING);
    latest_stop = TCNT2;

}



void setup()
{
    Serial.begin(115200);
    Serial.println("Dual Tone DDS generator, version 0.2a");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    pinMode(PIN_LOOP_TIMING, OUTPUT);
    digitalWrite(PIN_LOOP_TIMING, LOW);

    pinMode(PIN_ISR_TIMING, OUTPUT);
    digitalWrite(PIN_ISR_TIMING, LOW);

    pinMode( PIN_TONE_A, OUTPUT);
    pinMode( PIN_TONE_B, OUTPUT);

    phaccu_a = 0;
    phaccu_b = 0;

    phinc_a = FrequencyToPhaseIncrement(  700 );
    phinc_b = FrequencyToPhaseIncrement( 1900 );

    // Timer2 Clock Prescaler to divide by 1
    sbi(TCCR2B, CS20);
    cbi(TCCR2B, CS21);
    cbi(TCCR2B, CS22);

    // Timer 2 Mode 3  / FAST PWM 
    sbi(TCCR2A, WGM20);  
    sbi(TCCR2A, WGM21);
    cbi(TCCR2B, WGM22);

    // Timer2 Set port A, clear Compare Match
    cbi(TCCR2A, COM2A0);
    sbi(TCCR2A, COM2A1);

    // Timer2 Set port B, clear Compare Match
    cbi(TCCR2A, COM2B0);
    sbi(TCCR2A, COM2B1);

    // disable Timer0 interrupts to avoid timing distortion
    // delay() is now not available
    // enable Timer2 Interrupt
    // NOT SURE IF REALY NEEDED cbi(TIMSK0,TOIE0);
    sbi(TIMSK2,TOIE2);
}


void loop()
{
    if (counter >= 62500) {
        uint8_t lt = latest_start;
        uint8_t ls = latest_stop;

        counter = 0;

        sbi(PORTD,PIN_LOOP_TIMING);
        Serial.print("Timer tick ....  ");
        Serial.print(lt);
        Serial.print(" ");
        Serial.println(ls);

        cbi(PORTD,PIN_LOOP_TIMING);
    }

    //   cbi (TIMSK2,TOIE2);              // disble Timer2 Interrupt
    //   phinc_a=pow(2,32)*dfreq/refclk;  // calulate DDS new tuning word
    //   sbi (TIMSK2,TOIE2);              // enable Timer2 Interrupt 

    //   Serial.print(dfreq);
    //   Serial.print("  ");
    //   Serial.println(phinc_a);
 }


