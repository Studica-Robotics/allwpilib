/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

#include "RaspiTime.h"
#include "RaspiClockInternal.h"

static std::atomic<bool> programStarted{false};

static std::atomic<uint64_t> programStartTime{0};

namespace hal {
    namespace init {
        void InitializeRaspiClock() {}
    }
}

void Raspi_restartTiming() {
    //programStartTime = raspi::vmxGetTime();
}

int64_t Raspi_getTime() {
    auto now = raspi::vmxGetTime();
    auto currentTime = now - programStartTime;
    return currentTime;
}

double Raspi_getTimestamp() { 
    return Raspi_getTime() * 1.0e-6; 
}

void Raspi_getProgramStarted() { 
    programStarted = true; 
}
