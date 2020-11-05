#include <Arduino.h>
#include <util/atomic.h>
#include "../lib/DataLog/DataLog.h"

#define CAPTURE_PIN 3
#define LED_PIN 5
// #define TEST_PWM_PIN 10

void InitIRCaptureTimer(void);
void StartIRCaptureTimer(void);
void StopIRCaptureTimer(void);

void TestISR(void);

int main() {
  init(); // inits the Arduino's registers for time-keeping, pwm, and such
  Serial.begin(115200);
  delay(100); // small delay to give the serial port time to boot up
  LogInfo("Capture IR Data project begins\n");

  pinMode(CAPTURE_PIN, INPUT);
  // attachInterrupt(CAPTURE_PIN, TestISR, CHANGE);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    InitIRCaptureTimer();
    StartIRCaptureTimer();
  }
  sei(); // enable interrupts

  // pinMode(TEST_PWM_PIN, OUTPUT);
  // analogWrite(TEST_PWM_PIN, 127);

  while(1) {
    unsigned long ct = millis();
    static unsigned long pt = ct;
    if (ct - pt >= 500) {
      LogInfo("i am alive\n");
      pt = ct;
    }
  }
  return 0;
}

typedef enum {
  WAIT_START,
  CAPTURE_DATA,
} ir_captutre_state_t;

// NOTE: 20MHz is a 50us period
#define NUM_CYCLES_FOR_1_125ms_AT_20MHz   22 // technically 22.5, will incorporate a threshold
#define NUM_CYCLES_FOR_2_25ms_AT_20MHz    45
#define NUM_CYCLES_FOR_3ms_AT_20MHz       60
#define CYCLES_THRESHOLD_AT_20MHz         2 // 2 cycles at 50us = to 100us padding
#define EXPECTED_BYTE_RECEIVED            0xD2

void TestISR(void) { // this works but will be jank to implement
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-11-04 08:19:24 
 * @Desc: timer restarts on positive edge on D3, then
 * captures and interrupts on negative edge on D3.
 * It increments at 20MHz i.e. 50us period
 */
ISR(TCB0_INT_vect) {
  // static ir_captutre_state_t curState = WAIT_START;
  // static uint8_t curBit = 0;
  // static uint8_t byteReceived = 0;
  // static uint16_t cycleCount = TCB1_CCMP;

  uint16_t cycleCount = TCB0.CCMP;
  LogInfo("isr, value in ccmp %u\n", cycleCount);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  // clear interrupt flag
  TCB0_INTFLAGS |= (1 << 0);

/*
  switch (curState) {
    case WAIT_START:
      if ((cycleCount > (NUM_CYCLES_FOR_3ms_AT_20MHz - CYCLES_THRESHOLD_AT_20MHz))
          && (cycleCount < (NUM_CYCLES_FOR_3ms_AT_20MHz + CYCLES_THRESHOLD_AT_20MHz))) {
        // start sequence received
        curBit = 0;
        byteReceived = 0;
        curState = CAPTURE_DATA;
        // digitalWrite(LED_PIN, HIGH);
      }
      break;
    case CAPTURE_DATA:
      if ((cycleCount >= (NUM_CYCLES_FOR_1_125ms_AT_20MHz - CYCLES_THRESHOLD_AT_20MHz))
          && (cycleCount <= (NUM_CYCLES_FOR_1_125ms_AT_20MHz + CYCLES_THRESHOLD_AT_20MHz))) {
        // 0 received
        // no bit to set when a 0 is received...
      }
      else if ((cycleCount >= (NUM_CYCLES_FOR_2_25ms_AT_20MHz - CYCLES_THRESHOLD_AT_20MHz))
          && (cycleCount <= (NUM_CYCLES_FOR_2_25ms_AT_20MHz + CYCLES_THRESHOLD_AT_20MHz))) {
        // 1 received
        byteReceived |= (1 << curBit);
      }
      else {
        // corrupt data or false IR source detected
        // digitalWrite(LED_PIN, LOW);
        curState = WAIT_START;
      }
      curBit++;

      if (curBit > 7 && curState == CAPTURE_DATA) { // all bits[7:0] received, no corrupt data, check for correct byte received
        if (byteReceived == EXPECTED_BYTE_RECEIVED) {
          // expected byte has been received, light the LED
          // digitalWrite(LED_PIN, HIGH);
        }
        else {
          // incorrect byte was received, turn LED OFF
          // digitalWrite(LED_PIN, LOW);
        }
        curState = WAIT_START;
      }
      break;
  }*/
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-11-03 15:21:38 
 * @Desc: inits input capture interrupt on TCB1
 * which is on Port F5 once TCBROUTEA is configured
 * properly which makes the IC input on pin D3
 * @Note: see section 15.3.6 in ATmega4809 datasheet
 */
void InitIRCaptureTimer(void) {
  // set TCB1's alternate output select, it is bit 1 in TCBROUTEA register
  // PORTMUX_TCBROUTEA |= (1 << 1); // for TCB1
  // for TCB0 alt location
  PORTMUX_TCBROUTEA |= (1 << 0);
  // keep all default settings in CTRLA, this means prescaler is 1 (CLKDIV1)
  TCB0_CTRLA |= 0x00;
  // config CTRLB's CNTMODE for IC Pulse-Width Measurement mode, CTRLB[2:0] = 0x4
  TCB0_CTRLB |= (1 << 2);
  /** keep all default settings in EVCTRL, this means for ICPWMM, counter is
   * cleared and restarted on POS edge, NEG edge interrupts */
  TCB0_EVCTRL |= 0x00;
  // TCB1_EVCTRL |= (1 << 6); // enable the IC Noise Cancellation Filter
  // set bit to enable interrupt on capture, bit 0 in INTCTRL register
  TCB0_INTCTRL |= (1 << 0);
  // clear interrupt flag
  TCB0_INTFLAGS |= (1 << 0);
}

void StartIRCaptureTimer(void) {
  TCB0_CTRLA |= (1 << 0); // set CTRLA's enable bit which is bit 0
}

void StopIRCaptureTimer(void) {
  TCB0_CTRLA &= ~(1 << 0); // clear CTRLA's enable bit which is bit 0
}