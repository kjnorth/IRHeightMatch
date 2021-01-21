/** 
 * @Author: Kodiak North 
 * @Date: 2021-01-21 11:25:01 
 * @Desc: this code enables/disables the IR LED on the Truck
 */

#include <Arduino.h>
#include <util/atomic.h>

#include "../lib/ArduinoTimerConfig/TimerConfig.h"
#include "../lib/DataLog/DataLog.h"

#define PWM_OUT 2
#define LED_OUT 3
#define BTN_IN  7

void InitIRLedTimer(void);
void StartIRLedTimer(void);
void StopIRLedTimer(void);
void StartIRLedBurst(void);
void StopIRLedBurst(void);

typedef enum {
  START_BURST_7ms,
  START_DELAY_3_5ms,
  BURSTING,
  BURST_GAP_TRANSMIT_1,
  BURST_GAP_TRANSMIT_0,
  SIGNAL_GAP,
} ir_pulse_state_t;

// voltalie variables because they're modified by the StartIRLedTimer function
static volatile ir_pulse_state_t curState = START_BURST_7ms;
static volatile uint16_t cycleCount = 0;

int main() {
  init(); // inits the Arduino's registers for time-keeping, pwm, and such
  Serial.begin(115200);
  delay(100); // small delay to give the serial port time to boot up
  LogInfo("Enable IR LED project begins\n");

  pinMode(LED_OUT, OUTPUT);
  pinMode(BTN_IN, INPUT_PULLUP);

  InitIRLedTimer(); // init the timer before setting the OC pin to output as per section 17.7.3 in ATmega2560 datasheet
  pinMode(PWM_OUT, OUTPUT); // pin D2 must be used for the PWM output
  StartIRLedTimer();

  while (1) {
    static bool stopped = false;
    if (digitalRead(BTN_IN) == LOW && !stopped) {
      StopIRLedTimer();
      stopped = true;
    }
    else if (digitalRead(BTN_IN) == HIGH && stopped) {
      StartIRLedTimer();
      stopped = false;
    }
  }
  return 0;
}

// NOTE: 37.9kHz is a 26.385us period
#define NUM_CYCLES_FOR_1_125ms_AT_37_9_kHz    43
#define NUM_CYCLES_FOR_2_25ms_AT_37_9_kHz     85
#define NUM_CYCLES_FOR_3_5ms_AT_37_9_kHz      133
#define NUM_CYCLES_FOR_7ms_AT_37_9_kHz        265
#define NUM_CYCLES_FOR_20ms_AT_37_9_kHz       758 // DO NOT USE - rec increments at 8MHz to 20ms which causes
                                                  // 16-bit timer to rollover 2.44 times.. 44% of 65535 is
                                                  // 28835.4 which is far too close to 3.5ms at 8MHz val of 28000
#define NUM_CYCLES_FOR_30ms_AT_37_9_kHz       1137
#define NUM_CYCLES_FOR_BURST                  10
/** 
 * @Author: Kodiak North 
 * @Date: 2021-01-21 14:41:34 
 * @Desc: IR LED Timer's interrupt vector for transmitting data via IR
 * to the T1838 receiver based on a modified version of NEC Code -
 * repetitive data with the LSB transmitted first
 */
ISR(TIMER3_COMPB_vect) {
  static uint8_t byteToTransmit = 0xD2; // 11010010
  static uint8_t curBit = 0;
  cycleCount++;

  switch (curState) {
    case START_BURST_7ms: // start with a 7ms burst for automatic gain control (AGC) stage
      if (cycleCount >= NUM_CYCLES_FOR_7ms_AT_37_9_kHz) {
        cycleCount = 0;
        curBit = 0;
        StopIRLedBurst();
        curState = START_DELAY_3_5ms;
      }
      break;
    case START_DELAY_3_5ms: // delay for 3.5ms before sending data
      if (cycleCount >= NUM_CYCLES_FOR_3_5ms_AT_37_9_kHz) {
        cycleCount = 0;
        StartIRLedBurst();
        curState = BURSTING;
      }
      break;
    case BURSTING: // send a burst
      if (cycleCount >= NUM_CYCLES_FOR_BURST) {
        cycleCount = 0;
        StopIRLedBurst();
        if (curBit > 7) {
          curState = SIGNAL_GAP;
        }
        else {
          if ((byteToTransmit & (1 << curBit)) == 0) {
            curState = BURST_GAP_TRANSMIT_0; // gap for the amount of time to transmit a 0
          }
          else {
            curState = BURST_GAP_TRANSMIT_1; // gap for the amount of time to transmit a 1
          }
          curBit++;
        }
      }
      break;
    case BURST_GAP_TRANSMIT_0: // transmit a 0 by gapping for 1.125ms before next burst
      if (cycleCount >= NUM_CYCLES_FOR_1_125ms_AT_37_9_kHz) {
        cycleCount = 0;
        StartIRLedBurst();
        curState = BURSTING;
      }
      break;
    case BURST_GAP_TRANSMIT_1: // transmit a 1 by gapping for 2.25ms before next burst
      if (cycleCount >= NUM_CYCLES_FOR_2_25ms_AT_37_9_kHz) {
        cycleCount = 0;
        StartIRLedBurst();
        curState = BURSTING;
      }
      break;
    case SIGNAL_GAP:
      /** gap for >15ms (as per 1838 IR rec datasheet) before sending the next signal
       * NOTE: cannot gap for 20ms because timer rolls over and causes issues. 30ms works fine */
      if (cycleCount >= NUM_CYCLES_FOR_30ms_AT_37_9_kHz) {
        cycleCount = 0;
        StartIRLedBurst();
        curState = START_BURST_7ms;
      }
      break;
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2021-01-21 14:09:35 
 * @Desc: initializes Timer3 for Output Compare mode, see section 17.7 in ATmega2560 datasheet 
 * IR LED must burst at 37.9 kHz and 33% duty for the T1838 receiver to accept the signal. Therefore,
 * for pwm freq of 37.9 kHz, want prescaler of 1, and TOP = (clkSpeed / (freqHz * prescaler)) - 1
 * -> (16000000 / (37900 * 1)) - 1 = 421.16, round down to 421
 * for 33% duty, OCR3B = TOP * 0.33 = 421 * 0.33 = 138.93, round up to 139
 * OC3A is PortE3 which is pin D5
 * OC3B is PortE4 which is pin D2 - This one is used!
 * OC3C is PortE5 which is pin D3 
 * NOTE: using channel B for waveform, must set D2 to output pin
 */
void InitIRLedTimer(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR3A = 0; TCCR3B = 0; TCCR3C = 0; // set TCCRnx registers to 0
    /** config WGM3[3:0] = 14 for fast PWM mode with ICR3 as the timer's TOP value,
     * therefore need to set WGM3 bits 3, 2, and 1 */
    TCCR3A |= (1 << WGM31); // TCCRnA holds WGMn bits [1:0]
    TCCR3B |= ((1 << WGM33) | (1 << WGM32)); // TCCRnB holds WGMn bits [3:2]
    // set ICR3 and OCR3B registers to generate the desired waveform
    OCR3B = 139;
    ICR3 = 421;
    // enable the OCIE3B interrupt
    TIMSK3 |= (1 << OCIE3B);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2021-01-21 14:13:01 
 * @Desc: starts the IR LED timer to drive the IR LED as per
 * the modified verision of NEC Code - repetitive data so the
 * PL can detect a HEIGHT_MATCHED event
 */
void StartIRLedTimer(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // reset current state and counter variables
    curState = START_BURST_7ms;
    cycleCount = 0;
    StartIRLedBurst(); // set register bits so output compare generates a waveform
    /** set CS3[2:0] for 1 prescaler, therefore need to set bit 0 and leave the
     * others cleared.
     * NOTE: setting CS3 bits starts the counter */
    TCCR3B |= (1 << CS30);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2021-01-21 14:15:41 
 * @Desc: stops the timer. call this function once the height
 * has been matched to reduce the load on the processor
 */
void StopIRLedTimer(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // clear the CS3[2:0] bits to stop the timer from counting
    TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30));
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2021-01-21 14:16:37 
 * @Desc: starts the 37.9 kHz pwm burst. this is a helper function used by
 * the IR LED Timer's interrupt vector and the StartIRLedTimer, and it
 * should not be called anywhere else!
 */
void StartIRLedBurst(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    /** connect OC3B to the port, config COM3B[1:0] for non-inverting mode,
     * therefore need to set bit 1 and leave bit 0 cleared. COM3A/C[1:0] are
     * left as 0, leaving OC3A/C disconnected so those ports operate normally */
    TCCR3A |= (1 << COM3B1);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2021-01-21 14:19:34 
 * @Desc: stops the 37.9 kHz pwm burst. this is a helper function used by
 * the IR LED Timer's interrupt vector, and it should not be called anywhere else!
 */
void StopIRLedBurst(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // clear the COM3B[1:0] bits to disconnect OC3B output from the timer
    TCCR3A &= ~((1 << COM3B1) | (1 << COM3B0));
  }
}