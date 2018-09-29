#include "channel_active.h"

void boot_setup(int* bootPin, const int mpuNum){
	for (int i = 0; i < mpuNum; ++i)
	  {
	    pinMode(bootPin[i], OUTPUT);
	  }
}


void boot(int* bootPin, const int mpuNum, int pin) {
  for (int i = 0; i < mpuNum; ++i)
  {
    digitalWrite(bootPin[i], HIGH);
  }
  digitalWrite(bootPin[pin], LOW);
}