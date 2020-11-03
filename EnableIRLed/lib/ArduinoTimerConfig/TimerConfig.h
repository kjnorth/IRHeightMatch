/** 
 * @Author: Kodiak North 
 * @Date: 2020-08-26 12:25:18 
 * @Desc: defines the TimerConfig class for optimal control
 * of the Arduino's timer's Output Compare interrupts.
 * @Note: Timer0 is left out because it controls delay(),
 * millis(), and micros(), and I do not want to alter
 * their functionality
 * @Note: See ARDUINO TIMER REGISTER NOTES for register
 * documentation
 */

#ifndef TIMER_CONFIG_H_
#define TIMER_CONFIG_H_

#include <Arduino.h>
#include <util/atomic.h>

// **** DEBUG DEFINES ****
// enable _DEBUG to display debug info
// #define _DEBUG
#ifdef _DEBUG
#define DEBUG(x) Serial.print(x)
#else
#define DEBUG(x)
#endif
// **** END DEBUG DEFINES ****

// **** GLOBAL DEFINES ****
/** pass a mask such as (A | C) to Timern start/stop functions to
 * start/stop compare match interrupts on Timern A and C channels */
#define A 0x01
#define B 0x02
#define C 0x04
// **** END GLOBAL DEFINES ****

// **** GLOBAL STRUCTS ****
typedef struct {
  uint32_t aPeriod;
  uint32_t bPeriod;
  uint32_t cPeriod; // note that timers 0 and 2 do not have a C output compare register
} timer_ocr_period_t; // holds periods for timer output compare registers A,B,C
// **** END GLOBAL STRUCTS ****

// **** DECLARATION OF GLOBAL VARIABLES ****
// NOTE: these are defined in TimerConfig.cpp
extern timer_ocr_period_t timer1;
extern timer_ocr_period_t timer2;
extern timer_ocr_period_t timer3;
// extern timer_ocr_period_t timer4;
// extern timer_ocr_period_t timer5;
// **** END DECLARATION OF GLOBAL VARIABLES ****

class TimerConfig {
  private:
    // **** PRIVATE FUNCTIONS ****
    TimerConfig() {} // do not allow instantiation of objects
    // **** END PRIVATE FUNCTIONS ****

  public:
    // **** PUBLIC FUNCTIONS ****
    // if Timer0 is altered for interrupts, PWM unusable on Mega pins 4, 13; on Uno/Nano pins 5, 6
    /* Timer0 functions here */

    // PWM unusable on Mega pins 11, 12; on Uno/Nano pins 9, 10
    static void InitTimer1ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC);
    static void StartTimer1ISRs(uint8_t startMask);
    static void StopTimer1ISRs(uint8_t stopMask);

    // PWM unusable on Mega pins 9, 10; on Uno/Nano pins 3, 11
    static void InitTimer2ISRs(uint16_t freqHzA, uint16_t freqHzB);
    static void StartTimer2ISRs(uint8_t startMask);
    static void StopTimer2ISRs(uint8_t stopMask);

#if defined(__AVR_ATmega2560__) // only prototype/implement these functions if using Atmega2560 chip
    // PWM unusable on pins 2, 3, 5
    static void InitTimer3ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC);
    static void StartTimer3ISRs(uint8_t startMask);
    static void StopTimer3ISRs(uint8_t stopMask);

    // PWM unusable on pins 6, 7, 8
    static void InitTimer4ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC);
    static void StartTimer4ISRs(uint8_t startMask);
    static void StopTimer4ISRs(uint8_t stopMask);

    // PWM unusable on pins 44, 45, 46
    static void InitTimer5ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC);
    static void StartTimer5ISRs(uint8_t startMask);
    static void StopTimer5ISRs(uint8_t stopMask);
#endif
    // **** END PUBLIC FUNCTIONS ****
};

// **** ARDUINO TIMER REGISTER NOTES ****
  /** Register config settings for use of all OCRnx compare match interrupts
   * NOTE: CTC mode must be turned OFF. When ON, the timers use OCnA as their
   * TOP value, and reset upon reaching it. This renders the OCnB/C compare
   * match interrupts useless unless they are setup to run at the same
   * frequency as OCnA which is no good for many scenarios.
   * BASICALLY: Leave the TCCRnx registers unchanged (equal to 0).
   * This means all timers are left free-running, so their OCRnx registers
   * must be incremented by the desired period each time a compare match
   * occurs like in the code example below.
   * **** CODE ****
   * ISR(TIMERn_COMPx_vect) {
   *   OCRnx += timern.xPeriod;
   *   // do stuff
   * }
   * **** END CODE **** */

  /** BIGNOTE: Atmega2560 datasheet section 17-19 covers timer configurations for timers 0,1,3,4,5.
   * Section 20 covers timer configs SPECIFIC to timer 2!
   * Timer2 is quite finicky. It operates best when its compare match registers, OCR2A/B are closest to the
   * maximum value of 255. I'm assuming this is because it is an 8 bit timer (so maybe Timer0 has similar
   * issues). This means that the frequency range for a given prescaler is extremely limited, so be sure
   * to test the TIMER0/2_COMPA/B_vect ISR for expected frequency when changing settings. */

  /** TCCRnA
   * [7:6] is compare output mode (COMn) for channel A
   * [5:4] COMnB
   * [3:2] COMnC
   * [1:0] is waveform generation mode, WGMn[1:0]. see tables 17-2, 17-3, 17-4, 17-5 (or 20-8 for timer 2)
   * in Atmega2560 datasheet */

  /** TCCRnB
   * [7] is input capture noise canceller (ICNCn), basically a built in debouncer for 4 clock cycles
   * [6] input capture edge select (ICESn), selects which edge is used to trigger an input capture event
   * [5] is reserved
   * [4:3] is WGMn[3:2] see tables 17-2, 17-3, 17-4, 17-5 (or 20-8 for timer 2) in Atmega2560 datasheet
   * [2:0] is clock source (prescaler) bits. see table 17-6 (or 20-9 for timer 2) in Atmega2560 datasheet */

  /** TCCRnC:
   * [7:5] control forced output compare (FOCn) A,B,C, respectively
   * [4:0] are reserved. set all to 0 for ISR functionality */

  /** TIMSKn
   * [5] is ICIEn (interrupt capture interrupt enable), set to 1 to enable IC interrupt
   * [3] is OCIEnC (output compart match C interrupt enable), set to 1 to enable OC interrupt on timer channel C
   * [2] is OCIEnB, set to 1 to enable OC interrupt on timer channel B
   * [1] is OCIEnA, set to 1 to enable OC interrupt on timer channel A
   * [0] is TOIEn (timer overflow interrupt enable), set to 1 to enable interrupt when timer n overflows 
   * other bits in register are unused */
// **** END ARDUINO TIMER REGISTER NOTES ****

#endif /* TIMER_CONFIG_H_ */