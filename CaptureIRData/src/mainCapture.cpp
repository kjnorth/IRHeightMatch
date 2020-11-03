#include <Arduino.h>
#include "../lib/DataLog/DataLog.h"

#define LED_PIN 5

int main() {
  init(); // inits the Arduino's registers for time-keeping, pwm, and such
  Serial.begin(115200);
  delay(100); // small delay to give the serial port time to boot up
  LogInfo("Capture IR Data project begins\n");

  pinMode(LED_PIN, OUTPUT);

  while(1) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
  return 0;
}