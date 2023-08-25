#ifndef NATIVE_RASPITIMEINTERNAL_H

#include "RaspiInternal.h"
#include <stdint.h>

#define NATIVE_RASPITIMEINTERNAL_H

void Raspi_restartTiming();

int64_t Raspi_getTime();

double Raspi_getTimestamp();

void Raspi_getProgramStarted();


#endif //NATIVE_RASPITIMEINTERNAL_H
