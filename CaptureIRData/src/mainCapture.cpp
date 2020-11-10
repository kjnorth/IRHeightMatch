#include <Arduino.h>
#include <util/atomic.h>
#include "../lib/DataLog/DataLog.h"

#define CAPTURE_PIN 3
#define LED_PIN 5

/** make this either TCB1 or TCB2, TCB3/4 ports are not mapped to N.E. pinout
 * NOTE: modify the PORTMUX, EVSYS, and Interrupt Vector properly in Init
 * function when changing this #define */
#define TCBn_IC_OBJECT TCB1

void InitIRCaptureTimer(void);
void StartIRCaptureTimer(void);
void StopIRCaptureTimer(void);

void TestISR(void);

volatile unsigned long ledStartTime = 0;

int main() {
  init(); // inits the Arduino's registers for time-keeping, pwm, and such
  Serial.begin(115200);
  delay(100); // small delay to give the serial port time to boot up
  Serial.printf("Capture IR Data project begins\n");

  pinMode(CAPTURE_PIN, INPUT_PULLUP);
  // **** example pin config for ISR on an edge change ****
  PORTA.DIRCLR |= (1 << 2); // set PA2 (D18) to input
  PORTA.PIN2CTRL = 0; // set PA2 (D18) CTRL for no inversion, no pullup, and interrupts disabled (see 16.5.12 in ATmega4809 datasheet)
  PORTA.PIN2CTRL |= PORT_ISC_BOTHEDGES_gc;
  // ******************************************************
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    InitIRCaptureTimer();
    StartIRCaptureTimer();
  }
  sei(); // enable interrupts

  while(1) {
    unsigned long ct = millis();
    static unsigned long pt = ct;
    if (ct - pt >= 5000) {
      Serial.printf("alive: CTRLA reg %u, TCBn_CTRLB reg %u, EVCTRL reg %u\n", TCBn_IC_OBJECT.CTRLA, TCBn_IC_OBJECT.CTRLB, TCBn_IC_OBJECT.EVCTRL);
      pt = ct;
    }

    unsigned long temp;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      temp = ledStartTime;
    }
    if (ct - temp >= 100) { // leave the LED on for 100ms once it's activated
      digitalWrite(LED_PIN, 0);
      digitalWrite(LED_BUILTIN, 0);
      // static bool printed = false;
      // if (!printed) {
      //   Serial.printf("LED turns off\n");
      //   printed = true;
      // }
    }
  }
  return 0;
}

typedef enum {
  WAIT_START,
  CAPTURE_DATA,
} ir_captutre_state_t;

// NOTE: running at CLK_DIV2 is 8MHz freq which is a 125ns period
// NOTE: 1ms is 1x10^6ns
#define NUM_CYCLES_FOR_1_125ms_AT_8MHz    9000
#define NUM_CYCLES_FOR_2_25ms_AT_8MHz     18000
#define NUM_CYCLES_FOR_3_5ms_AT_8MHz      28000
#define CYCLES_THRESHOLD                  2000 // 1000 cycles at 125ns = .125ms padding
#define EXPECTED_BYTE_RECEIVED            0xD2

/** 
 * @Author: Kodiak North 
 * @Date: 2020-11-04 08:19:24 
 * @Desc: TCBn is configured for Input Capture
 * Pulse Width Measurement mode to measure
 * the duration between pulses on the VM317 IR
 * receiver. The IR LED uses a modified version
 * of NEC Code to transmits single byte. An event
 * is posted upon reception of the expected byte.
 * Note: TCBn increments at 8MHz i.e. 125ns period
 * Note: TCB1 is currently used, meaning IC pin
 * is D3
 */
ISR(TCB1_INT_vect) {
  static ir_captutre_state_t curState = WAIT_START;
  static uint8_t curBit = 0;
  static uint8_t byteReceived = 0;
  uint16_t cycleCount = TCBn_IC_OBJECT.CCMP;

  // Serial.printf("count %u\n", cycleCount);

  switch (curState) {
    case WAIT_START:
      if ((cycleCount > (NUM_CYCLES_FOR_3_5ms_AT_8MHz - CYCLES_THRESHOLD))
          && (cycleCount < (NUM_CYCLES_FOR_3_5ms_AT_8MHz + CYCLES_THRESHOLD))) {
        // start sequence received
        curBit = 0;
        byteReceived = 0;
        curState = CAPTURE_DATA;
      }
      break;
    case CAPTURE_DATA:
      if ((cycleCount >= (NUM_CYCLES_FOR_1_125ms_AT_8MHz - CYCLES_THRESHOLD))
          && (cycleCount <= (NUM_CYCLES_FOR_1_125ms_AT_8MHz + CYCLES_THRESHOLD))) {
        // 0 received, clear the cur bit
        // byteReceived &= ~(1 << curBit);
        /** unnecessary to call line above since byteReceived is cleared in WAIT_START
         * condition is still necessary so the 'else' is not entered incorrectly */
        curBit++;
      }
      else if ((cycleCount >= (NUM_CYCLES_FOR_2_25ms_AT_8MHz - CYCLES_THRESHOLD))
          && (cycleCount <= (NUM_CYCLES_FOR_2_25ms_AT_8MHz + CYCLES_THRESHOLD))) {
        // 1 received, set the cur bit
        byteReceived |= (1 << curBit);
        curBit++;
      }
      else {
        // corrupt data or false IR source detected
        curState = WAIT_START;
      }

      if (curBit > 7) { // all bits[7:0] received, no corrupt data, check for correct byte received
        if (byteReceived == EXPECTED_BYTE_RECEIVED) {
          // expected byte has been received, light the LED
          digitalWrite(LED_PIN, HIGH);
          digitalWrite(LED_BUILTIN, HIGH);
          ledStartTime = millis();
        }
        curState = WAIT_START;
      }
      break;
  }
}

// **** interrupt handler for any? interrupt on port A ****
ISR(PORTA_PORT_vect) {
  PORTA.INTFLAGS |= (1 << 2); // clear PA2 interrupt flag
  // Serial.printf("port a interrupt\n");
}
// ********************************************************

/** 
 * @Author: Kodiak North 
 * @Date: 2020-11-03 15:21:38 
 * @Desc: inits input capture interrupt on TCB1
 * which is on Port F5 once TCBROUTEA is configured
 * properly which makes the IC input on pin D3
 * @Note: see section 15.3.6 in ATmega4809 datasheet
 */
void InitIRCaptureTimer(void) {
  // disable the peripheral first
  TCBn_IC_OBJECT.CTRLA &= ~(1 << TCB_ENABLE_bp); // clear TCBn_CTRLA's enable bit which is bit 0
  TCBn_IC_OBJECT.CTRLA = 0; // reset TCBn_CTRLA register
  TCBn_IC_OBJECT.CTRLB = 0; // reset TCBn_CTRLB register
  TCBn_IC_OBJECT.EVCTRL = 0; // reset TCBn_EVCTRL register
  TCBn_IC_OBJECT.INTCTRL = 0; // reset TCBn_INTCTRL register
  TCBn_IC_OBJECT.CNT = 0; // reset the peripheral's count
  TCBn_IC_OBJECT.CCMP = 0; // reset the peripheral's Compare/Capture register
  PORTMUX_TCBROUTEA = 0; // reset the TCB PORTMUX register
  // set TCB1's alternate output select (PF5), it is bit 1 in TCBROUTEA register
  PORTMUX_TCBROUTEA |= (1 << PORTMUX_TCB1_bp); // for TCB1
  // set CTRLA's CLKSEL for CLKDIV2 (2 prescaler)
  TCBn_IC_OBJECT.CTRLA |= (1 << TCB_CLKSEL0_bp);
  // config CTRLB's CNTMODE for IC Pulse-Width Measurement mode, CTRLB[2:0] = 0x4
  TCBn_IC_OBJECT.CTRLB |= (1 << TCB_CNTMODE2_bp);
  /** enable input capture event and noise cancellation filter in EVCTRL, leave
   * other bits cleared, this means for ICPWMM, counter is cleared and restarted
   * on POS edge, NEG edge interrupts */
  TCBn_IC_OBJECT.EVCTRL |= (1 << TCB_CAPTEI_bp) | (1 << TCB_FILTER_bp);
  // set bit to enable interrupt on capture, bit 0 in INTCTRL register
  TCBn_IC_OBJECT.INTCTRL |= (1 << TCB_CAPT_bp);
  // clear any interrupt flag
  TCBn_IC_OBJECT.INTFLAGS |= (1 << TCB_CAPT_bp);
  // congifure the event system
  EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT1_PIN5_gc; // map PF5 (D3) to event generator, refer to 14.5.2 in ATmega4809 datasheet when changing
  EVSYS.USERTCB1 = EVSYS_CHANNEL_CHANNEL4_gc; // map event system channel 4 to TCB1 event user
}

void StartIRCaptureTimer(void) {
  TCBn_IC_OBJECT.CTRLA |= (1 << TCB_ENABLE_bp); // set CTRLA's enable bit which is bit 0
}

void StopIRCaptureTimer(void) {
  TCBn_IC_OBJECT.CTRLA &= ~(1 << TCB_ENABLE_bp); // clear CTRLA's enable bit which is bit 0
}