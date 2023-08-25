#ifndef NATIVE_RASPITIME_H

#include <VMXTime.h>

#define NATIVE_RASPITIME_H

namespace raspi {
    extern VMXTime* vmxTime;

    inline uint64_t vmxGetTime() {
        return vmxTime->GetCurrentTotalMicroseconds();
    }
}

#endif //NATIVE_RASPITIME_H
