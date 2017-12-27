#include <iostream>

#include <pigpio.h>

#include "rotary_encoder.hpp"

#include <stdint.h>


/*

             +---------+         +---------+      0
             |         |         |         |
   A         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 1

*/

void re_decoder::_pulse(int gpio, int level, uint32_t tick)
{
    struct timespec timeStruct;
    clock_gettime(CLOCK_MONOTONIC, &timeStruct);
    uint64_t timeNowUs = timeStruct.tv_sec * 1000000 + timeStruct.tv_nsec / 1000;
  
    if (timeNowUs > timeOldUs){
      if (level) (mycallback)(1);
      // delta 15 ms because peak to peak is min 20 ms, therefore we debounce
      // change this, if you have a faster motor
      timeOldUs = timeNowUs + 15000; 
    }
 
    //std::cout << "timeNowUs=" << timeNowUs;
    //std::cout << "timeOldUs=" << timeOldUs << std::endl;
}

void re_decoder::_pulseEx(int gpio, int level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   re_decoder *mySelf = (re_decoder *) user;

   mySelf->_pulse(gpio, level, tick); /* Call the instance callback. */
}

re_decoder::re_decoder(int gpioA, re_decoderCB_t callback)
{
   mygpioA = gpioA;

   mycallback = callback;

   timeOldUs = 0;

   gpioSetMode(gpioA, PI_INPUT);

   /* pull up is needed as encoder common is grounded */

   gpioSetPullUpDown(gpioA, PI_PUD_UP);

   /* monitor encoder level changes */
   gpioSetAlertFuncEx(gpioA, _pulseEx, this);
}

void re_decoder::re_cancel(void)
{
   gpioSetAlertFuncEx(mygpioA, 0, this);
}

