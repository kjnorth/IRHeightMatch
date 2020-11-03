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
 * * * * * * * * * * floats * * * * * * * * * * * * 
 * LogInfo("my float is ", myFloat, 4, true); // true for newline, flase or empty for no newline
 * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

#ifndef DATA_LOG_H_
#define DATA_LOG_H_

#include <Arduino.h>

void LogInfo(const char *msg, ...);
void LogInfo(const __FlashStringHelper* msg, ...);
// @param - newline: true -> adds \n to message
void LogInfo(const char *msg, float data, int digits, bool newline=false);
void LogInfo(float data, int digits);

#endif /* DATA_LOG_H_ */