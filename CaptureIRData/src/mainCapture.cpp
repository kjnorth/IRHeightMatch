#include <Arduino.h>
#include "../lib/DataLog/DataLog.h"

#define CAPTURE_PIN 3
#define LED_PIN 5

void InitIRCaptureTimer(void);
void StartIRCaptureTimer(void);
void StopIRCaptureTimer(void);

int main() {
  init(); // inits the Arduino's registers for time-keeping, pwm, and such
  Serial.begin(115200);
  delay(100); // small delay to give the serial port time to boot up
  LogInfo("Capture IR Data project begins\n");

  pinMode(CAPTURE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  InitIRCaptureTimer();
  StartIRCaptureTimer();

  while(1) {

  }
  return 0;
}

ISR(TCB1_INT_vect) {
  digitalWrite(LED_PIN, !digitalRead(CAPTURE_PIN));
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
  PORTMUX_TCBROUTEA |= (1 << 1);
  // keep all default settings in CTRLA, this means prescaler is 1 (CLKDIV1)
  TCB1_CTRLA = 0;
  // config CTRLB's CNTMODE for IC Pulse-Width Measurement mode, CTRLB[2:0] = 0x4
  TCB1_CTRLB |= (1 << 2);
  /** keep all default settings in EVCTRL, this means for ICPWMM, counter is
   * cleared and restarted on POS edge, NEG edge interrupts */
  TCB1_EVCTRL = 0;
  // set bit to enable interrupt on capture, bit 0 in INTCTRL register
  TCB1_INTCTRL |= (1 << 0);
}

void StartIRCaptureTimer(void) {
  TCB1_CTRLA |= (1 << 0); // set CTRLA's enable bit which is bit 0
}

void StopIRCaptureTimer(void) {
  TCB1_CTRLA &= ~(1 << 0); // clear CTRLA's enable bit which is bit 0
}