#ifndef NATIVE_RASPIENUMSINTERNAL_H

#include "hal/handles/HandlesInternal.h"
#include <VMXChannel.h>
#include <map>
#include <math.h>
#include <string>
#include <vector>
#include<utility>

#define NATIVE_RASPIENUMSINTERNAL_H

class Raspi_EnumConverter {
    std::vector<std::pair<std::string, hal::HAL_HandleEnum>> halHandles;
public:
    void setHandlePair(std::string label, hal::HAL_HandleEnum handle);
    std::string getHandleLabel(hal::HAL_HandleEnum handle);
    hal::HAL_HandleEnum getHandleValue(std::string label);
};

#endif //NATIVE_RASPIENUMSINTERNAL_H