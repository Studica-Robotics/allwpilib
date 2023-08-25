/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

namespace hal {

constexpr int32_t kSystemClockTicksPerMicrosecond = 1;
constexpr int32_t kDutyCycleScaleFactor = 4e7 - 1;

}  // namespace hal
