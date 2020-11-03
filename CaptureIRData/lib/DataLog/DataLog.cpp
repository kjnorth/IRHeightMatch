/** 
 * @Author: Kodiak 
 * @Date: 2020-02-12 10:35:10 
 * @Desc: Used for data logging similar to printf() in C.
 * Typical use is as follows
 * * * * * * * * normal message * * * * * * * * * *
 * LogInfo("Hello world\n");
 * * * * * * * * * * * * * * * * * * * * * * * * * *
 * * * * * * * ints, longs, bools * * * * * * * * *
 * LogInfo(F("my int is %d\n"), myInt);
 * * * * * * * * * * * * * * * * * * * * * * * * * *
 * LogInfo("my float is ", myFloat, 4, true); // true for newline, flase or empty for no newline
 * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

#include <stdio.h>
#include <stdarg.h>

#include "DataLog.h"

static char buf[256];

void LogInfo(const char *msg, ...) {
  va_list arg;

  // Write Message
  va_start(arg, msg);
  vsprintf(buf, msg, arg);
  va_end(arg);
  Serial.write(buf); 
}

void LogInfo(const __FlashStringHelper *msg, ...) {
  va_list arg;

  // Write Message
  va_start(arg, msg);

#ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), (const char *)msg, arg); // program for AVR
#else
  vsnprintf(buf, sizeof(buf), (const char *)msg, arg); // for the rest of the world
#endif
  va_end(arg);
    
  Serial.write(buf);
}

void LogInfo(const char *msg, float data, int digits, bool newline) {
  Serial.print(msg);
  Serial.print(data, digits);
  if (newline)
    Serial.write("\n");
}

void LogInfo(float data, int digits) {
	Serial.print(data, digits);
}