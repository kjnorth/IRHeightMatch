/** 
 * @Author: Kodiak North 
 * @Date: 2020-08-26 14:01:36 
 * @Desc: implements the TimerConfig class
 * @Note: see notes at the bottom of TimerConfig.h
 * when changing settings
 */

#include "TimerConfig.h"

// **** FUNCTION NOTES ****
// NOTE: n = 1-5, x = A, B, or C
/** @InitTimernISRs
 * @Desc: initializes Timernx compare match interrupts on all x channels
 * @Param - freqHzx: interrupt frequency in Hz for the x channel
 * @Note: pass 0 for freqHzx if you don't want that channel to interrupt,
 * or do not start that channel with the StartTimernISRs function
 * @Note: DO NOT init Timern's channel A in one call, and then Timern's
 * channel B/C in another call without passing channel A's frequency; otherwise
 * the call to init channel B/C will reset channel A's compare register
 * @Note: Timernx ISRs do not start firing until StartTimernISRs is called
 */

/** @StartTimernISRs
 * @Desc: Starts a timer's output compare interrupts on channels
 * passed in startMask param
 * @Param - startMask: a mask such as (A | C) 
 * @Note: does not start channel if it's already started */

/** @StopTimernISRs
 * @Desc: Stops a timer's output compare interrupts on channels
 * passed in stopMask param
 * @Param - stopMask: a mask such as (A | C) 
 * @Note: does not stop channel if it's already stopped */
// **** END FUNCTION NOTES ****

// **** PRIVATE DEFINES ****
#define CLK_SPEED           16000000
/** NOTE: if changing PRESCALER_Tx, must change TCCRxB register appropriately to match value
 * NOTE: max frequencies are extremely unreliable. pick frequencies closer to the min range,
 * especially for the 8 bit timers 0 and 2 */
#define PRESCALER_T0        64 // min freq is 980 Hz, max freq is 1010 Hz (max freq tested)
#define PRESCALER_T1        8 // min freq is 31 Hz, max freq is 2 MHz
#define PRESCALER_T2        64 // min freq is 980 Hz, max freq is 1010 Hz (max freq tested)
#define PRESCALER_T3        8 // min freq is 31 Hz, max freq is 2 MHz
#define PRESCALER_T4        8 // min freq is 31 Hz, max freq is 2 MHz
#define PRESCALER_T5        8 // min freq is 31 Hz, max freq is 2 MHz
#define TIMER_8BIT_MAX      255 // timer 0 and 2 are 8 bit timers
#define TIMER_16BIT_MAX     65535 // timer 1, 3, 4, and 5 are 16 bit timers
#define OCRnx_CONFIG_PERIOD(freqHz,prescaler) ((CLK_SPEED / (freqHz * prescaler))-1) // n: 0-5; x: A,B,C
// **** END PRIVATE DEFINES ****

// **** DEFINITION OF GLOBAL VARIABLES ****
timer_ocr_period_t timer1;
timer_ocr_period_t timer2;
timer_ocr_period_t timer3;
// timer_ocr_period_t timer4;
// timer_ocr_period_t timer5;
// **** END DEFINITION OF GLOBAL VARIABLES ****

// PWM unusable on Mega pins 11, 12; on Uno/Nano pins 9, 10
void TimerConfig::InitTimer1ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC) {
  TCCR1A = 0; TCCR1B = 0; TCCR1C = 0; // set TCCRnx registers to 0
  /** NOTE: if changing TCCRxB prescaler bits, must make PRESCALER_Tx match new value in #define above */
  TCCR1B |= (1 << CS11); // set CS11 bit for 8 prescaler

  if (freqHzA) {
    timer1.aPeriod = OCRnx_CONFIG_PERIOD(freqHzA, PRESCALER_T1);
    if (timer1.aPeriod > TIMER_16BIT_MAX) {
      timer1.aPeriod = 0;
      DEBUG("TimerConfig::InitTimer1ISRs - ERROR timer1 A frequency is too low, increase frequency or alter prescaler\n");
    }
  }

  if (freqHzB) {
    timer1.bPeriod = OCRnx_CONFIG_PERIOD(freqHzB, PRESCALER_T1);
    if (timer1.bPeriod > TIMER_16BIT_MAX) {
      timer1.bPeriod = 0;
      DEBUG("TimerConfig::InitTimer1ISRs - ERROR timer1 B frequency is too low, increase frequency or alter prescaler\n");
    }
  }

  if (freqHzC) {
    timer1.cPeriod = OCRnx_CONFIG_PERIOD(freqHzC, PRESCALER_T1);
    if (timer1.cPeriod > TIMER_16BIT_MAX) {
      timer1.cPeriod = 0;
      DEBUG("TimerConfig::InitTimer1ISRs - ERROR timer1 C frequency is too low, increase frequency or alter prescaler\n");
    }
  }
}

void TimerConfig::StartTimer1ISRs(uint8_t startMask) {
  uint8_t aStatus = TIMSK1 & (1 << OCIE1A); // nonzero if A channel is already started
  uint8_t bStatus = TIMSK1 & (1 << OCIE1B); // nonzero if B channel is already started
  uint8_t cStatus = TIMSK1 & (1 << OCIE1C); // nonzero if C channel is already started

  if ((startMask & A) && !aStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 |= (1 << OCIE1A);
      OCR1A = TCNT1 + timer1.aPeriod;
    }
  }

  if ((startMask & B) && !bStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 |= (1 << OCIE1B);
      OCR1B = TCNT1 + timer1.bPeriod;
    }
  }

  if ((startMask & C) && !cStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 |= (1 << OCIE1C);
      OCR1C = TCNT1 + timer1.cPeriod;
    }
  }
}

void TimerConfig::StopTimer1ISRs(uint8_t stopMask) {
  uint8_t aStatus = TIMSK1 & (1 << OCIE1A); // nonzero if A channel is started
  uint8_t bStatus = TIMSK1 & (1 << OCIE1B); // nonzero if B channel is started
  uint8_t cStatus = TIMSK1 & (1 << OCIE1C); // nonzero if C channel is started

  if ((stopMask & A) && aStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 &= ~(1 << OCIE1A);
      OCR1A = TCNT1 + timer1.aPeriod;
    }
  }

  if ((stopMask & B) && bStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 &= ~(1 << OCIE1B);
      OCR1B = TCNT1 + timer1.bPeriod;
    }
  }

  if ((stopMask & C) && cStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK1 &= ~(1 << OCIE1C);
      OCR1C = TCNT1 + timer1.cPeriod;
    }
  }
}

// PWM unusable on Mega pins 9, 10; on Uno/Nano pins 3, 11
void TimerConfig::InitTimer2ISRs(uint16_t freqHzA, uint16_t freqHzB) {
  TCCR2A = 0; TCCR2B = 0; // set TCCRnx registers to 0
  /** NOTE: if changing TCCRxB prescaler bits, must make PRESCALER_Tx match new value in #define above */
  TCCR2B |= (1 << CS22); // set CS22 bit for 64 prescaler

  if (freqHzA) {
    timer2.aPeriod = OCRnx_CONFIG_PERIOD(freqHzA, PRESCALER_T2);
    if (timer2.aPeriod > TIMER_8BIT_MAX) {
      timer2.aPeriod = 0;
      DEBUG("TimerConfig::InitTimer2ISRs - ERROR timer2 A frequency is too low, increase frequency or alter prescaler\n");
    }
  }

  if (freqHzB) {
    timer2.bPeriod = OCRnx_CONFIG_PERIOD(freqHzB, PRESCALER_T2);
    if (timer2.bPeriod > TIMER_8BIT_MAX) {
      timer2.bPeriod = 0;
      DEBUG("TimerConfig::InitTimer2ISRs - ERROR timer2 B frequency is too low, increase frequency or alter prescaler\n");
    }
  }
}

void TimerConfig::StartTimer2ISRs(uint8_t startMask) {
  uint8_t aStatus = TIMSK2 & (1 << OCIE2A); // nonzero if A channel is already started
  uint8_t bStatus = TIMSK2 & (1 << OCIE2B); // nonzero if B channel is already started

  if ((startMask & A) && !aStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK2 |= (1 << OCIE2A);
      OCR2A = TCNT2 + timer2.aPeriod;
    }
  }

  if ((startMask & B) && !bStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK2 |= (1 << OCIE2B);
      OCR2B = TCNT2 + timer2.bPeriod;
    }
  }
}

void TimerConfig::StopTimer2ISRs(uint8_t stopMask) {
  uint8_t aStatus = TIMSK2 & (1 << OCIE2A); // nonzero if A channel is started
  uint8_t bStatus = TIMSK2 & (1 << OCIE2B); // nonzero if B channel is started

  if ((stopMask & A) && aStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK2 &= ~(1 << OCIE2A);
      OCR2A = TCNT2 + timer2.aPeriod;
    }
  }

  if ((stopMask & B) && bStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK2 &= ~(1 << OCIE2B);
      OCR2B = TCNT2 + timer2.bPeriod;
    }
  }
}

#if defined(__AVR_ATmega2560__) // only prototype/implement these functions if using Atmega2560 chip
// PWM unusable on Mega pins 2, 3, 5
void TimerConfig::InitTimer3ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC) {
  TCCR3A = 0; TCCR3B = 0; TCCR3C = 0; // set TCCRnx registers to 0
  /** NOTE: if changing TCCRxB prescaler bits, must make PRESCALER_Tx match new value in #define above */
  TCCR3B |= (1 << CS31); // set CS31 bit for 8 prescaler

  if (freqHzA) {
    timer3.aPeriod = OCRnx_CONFIG_PERIOD(freqHzA, PRESCALER_T3);
    if (timer3.aPeriod > TIMER_16BIT_MAX) {
      timer3.aPeriod = 0;
      DEBUG("TimerConfig::InitTimer3ISRs - ERROR timer3 A frequency is too low, increase frequency or alter prescaler\n");
    }
  }

  if (freqHzB) {
    timer3.bPeriod = OCRnx_CONFIG_PERIOD(freqHzB, PRESCALER_T3);
    if (timer3.bPeriod > TIMER_16BIT_MAX) {
      timer3.bPeriod = 0;
      DEBUG("TimerConfig::InitTimer3ISRs - ERROR timer3 B frequency is too low, increase frequency or alter prescaler\n");
    }
  }

  if (freqHzC) {
    timer3.cPeriod = OCRnx_CONFIG_PERIOD(freqHzC, PRESCALER_T3);
    if (timer3.cPeriod > TIMER_16BIT_MAX) {
      timer3.cPeriod = 0;
      DEBUG("TimerConfig::InitTimer3ISRs - ERROR timer3 C frequency is too low, increase frequency or alter prescaler\n");
    }
  }
}

void TimerConfig::StartTimer3ISRs(uint8_t startMask) {
  uint8_t aStatus = TIMSK3 & (1 << OCIE3A); // nonzero if A channel is already started
  uint8_t bStatus = TIMSK3 & (1 << OCIE3B); // nonzero if B channel is already started
  uint8_t cStatus = TIMSK3 & (1 << OCIE3C); // nonzero if C channel is already started

  if ((startMask & A) && !aStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK3 |= (1 << OCIE3A);
      OCR3A = TCNT3 + timer3.aPeriod;
    }
  }

  if ((startMask & B) && !bStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK3 |= (1 << OCIE3B);
      OCR3B = TCNT3 + timer3.bPeriod;
    }
  }

  if ((startMask & C) && !cStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK3 |= (1 << OCIE3C);
      OCR3C = TCNT3 + timer3.cPeriod;
    }
  }
}

void TimerConfig::StopTimer3ISRs(uint8_t stopMask) {
  uint8_t aStatus = TIMSK3 & (1 << OCIE3A); // nonzero if A channel is started
  uint8_t bStatus = TIMSK3 & (1 << OCIE3B); // nonzero if B channel is started
  uint8_t cStatus = TIMSK3 & (1 << OCIE3C); // nonzero if C channel is started

  if ((stopMask & A) && aStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK3 &= ~(1 << OCIE3A);
      OCR3A = TCNT3 + timer3.aPeriod;
    }
  }

  if ((stopMask & B) && bStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK3 &= ~(1 << OCIE3B);
      OCR3B = TCNT3 + timer3.bPeriod;
    }
  }

  if ((stopMask & C) && cStatus) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // disable interrupts when modifying registers, reenable upon exit of scope
      TIMSK3 &= ~(1 << OCIE3C);
      OCR3C = TCNT3 + timer3.cPeriod;
    }
  }
}

// PWM unusable on Mega pins 6, 7, 8
void TimerConfig::InitTimer4ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC) {
  return;
}

void TimerConfig::StartTimer4ISRs(uint8_t startMask) {
  return;
}

void TimerConfig::StopTimer4ISRs(uint8_t stopMask) {
  return;
}

// PWM unusable on Mega pins 44, 45, 46
void TimerConfig::InitTimer5ISRs(uint16_t freqHzA, uint16_t freqHzB, uint16_t freqHzC) {
  return;
}

void TimerConfig::StartTimer5ISRs(uint8_t startMask) {
  return;
}

void TimerConfig::StopTimer5ISRs(uint8_t stopMask) {
  return;
}
#endif