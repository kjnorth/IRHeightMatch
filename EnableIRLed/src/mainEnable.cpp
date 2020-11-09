#include <Arduino.h>
#include <util/atomic.h>

#include "../lib/ArduinoTimerConfig/TimerConfig.h"
#include "../lib/DataLog/DataLog.h"

// TODO: test generating a waveform at 1kHz and determine whether the register needs a +1 or not

#define LED_OUT 2
#define PWM_OUT 5
#define BTN_IN  7

void InitIRLedTimer(void);
void StartIRLedTimer(void);
void StopIRLedTimer(void);
void StartIRLedBurst(void);
void StopIRLedBurst(void);

typedef enum {
  START_BURST_7ms,
  START_DELAY_3_5ms, // 1.5 s
  BURSTING,
  BURST_GAP_TRANSMIT_1,
  BURST_GAP_TRANSMIT_0,
  SIGNAL_GAP,
} ir_pulse_state_t;

static volatile ir_pulse_state_t curState = START_BURST_7ms;
static volatile uint16_t cycleCount = 0;
static volatile bool printFlag = false;
static volatile uint32_t totalCycles = 0;

int main() {
  init(); // inits the Arduino's registers for time-keeping, pwm, and such
  Serial.begin(115200);
  delay(100); // small delay to give the serial port time to boot up
  LogInfo("Enable IR LED project begins\n");

  pinMode(PWM_OUT, OUTPUT); // pin D5 is used for the PWM output
  pinMode(LED_OUT, OUTPUT);
  pinMode(BTN_IN, INPUT_PULLUP);
  InitIRLedTimer();
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
#define NUM_CYCLES_FOR_16ms_AT_37_9_kHz       606
#define NUM_CYCLES_FOR_BURST                  10

// modified version of NEC Code - repetitive data, LSB transmitted first
/** with current config, data rate is at roughly 22Hz
 * PL lifts at roughly 1.33 in/s, let's round up at say 1.5 in/s
 * Receiver sensing area is 0.217 inches in diameter
 * => 0.217in/1.5in/s = 0.145s = 145ms to detect the data stream
 * 22Hz has 45ms period => about 3 tries to detect the data
 */
ISR(TIMER3_COMPA_vect) {
  static uint8_t byteToTransmit = 0b11010010; // 0xD2
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
    case SIGNAL_GAP: // gap for 20ms before sending the next signal
      if (cycleCount >= NUM_CYCLES_FOR_16ms_AT_37_9_kHz) {
        cycleCount = 0;
        StartIRLedBurst();
        curState = START_BURST_7ms;
      }
      break;
  }
}

/** for pwm freq of 37.9 kHz, want prescaler of 1, and TOP = (clkSpeed / (freqHz * prescaler)) - 1
 * -> (16000000 / (37900 * 1)) - 1 = 421.16, round down to 421
 * for 33% duty, OCR3A = TOP * 0.33 = 421 * 0.33 = 138.93, round up to 139
 * OC3A is PortE3 which is pin D5
 * OC3B is PortE4 which is pin D2
 * OC3C is PortE5 which is pin D3 
 * NOTE: using channel A for waveform, must set D5 to output pin */
void InitIRLedTimer(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR3A = 0; TCCR3B = 0; TCCR3C = 0; // set TCCRnx registers to 0
    /** config WGM3[3:0] = 14 for fast PWM mode with ICR3 as the timer's TOP value,
     * therefore need to set WGM3 bits 3, 2, and 1 */
    TCCR3A |= (1 << WGM31); // TCCRnA holds WGMn bits [1:0]
    TCCR3B |= ((1 << WGM33) | (1 << WGM32)); // TCCRnB holds WGMn bits [3:2]
    // set ICR3 and OCR3A registers to generate the desired waveform
    OCR3A = 139;
    ICR3 = 421;
    // enable the OCIE3A interrupt
    TIMSK3 |= (1 << OCIE3A); 
  }
}

void StartIRLedTimer(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // reset current state and counter variables
    curState = START_BURST_7ms;
    cycleCount = 0;
    StartIRLedBurst(); // set register bits so output compare generates a waveform
    /** set CS3[2:0] for 1 prescaler, therefore need to set bit 0.
     * NOTE: setting the CS3 bits starts the counter */
    TCCR3B |= (1 << CS30);
  }
}

void StopIRLedTimer(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // clear the CS3[2:0] bits to stop the timer from counting
    TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30));
  }
}

void StartIRLedBurst(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    /** connect OC3A to the port, config COM3A[1:0] for non-inverting mode,
     * therefore need to set bit 1. COM3B/C[1:0] are left as 0, leaving
     * OC3B/C disconnected so those ports operate normally */
    TCCR3A |= (1 << COM3A1);
  }
}

void StopIRLedBurst(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // clear the COM3A[1:0] bits to disconnect OC3A output from the timer
    TCCR3A &= ~((1 << COM3A1) | (1 << COM3A0));
  }
}