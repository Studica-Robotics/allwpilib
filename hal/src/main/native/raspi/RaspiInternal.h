#pragma once

#include <string>
#include <AHRS.h>
#include <VMXIO.h>
#include <VMXCAN.h>
#include <VMXTime.h>
#include <VMXPower.h>
#include <VMXThread.h>
#include <VMXErrors.h>
#include <VMXChannel.h>
#include "RaspiMap.h"
#include "Translator/include/RaspiEnumConverter.h"
#include "RaspiErrors.h"
#include "hal/handles/HandlesInternal.h"

namespace raspi {
    extern vmx::AHRS* vmxIMU;
    extern VMXIO* vmxIO;
    extern VMXCAN* vmxCAN;
    extern VMXPower* vmxPower;
    extern VMXThread* vmxThread;

    extern VMXErrorCode* vmxError;

    extern Raspi_ChannelMap* channelMap;
    extern Raspi_EnumConverter* enumConverter;

    extern bool vmxIOActive;

    inline VMXChannelInfo GetChannelInfo(hal::HAL_HandleEnum hal_handle_type, int channel_index) {
    	VMXChannelInfo chan_info =
    			channelMap->getChannelInfo(enumConverter->getHandleLabel(hal_handle_type), channel_index);
    	// Update capabilities provided by underlying VMX-pi HAL
    	VMXChannelType chan_type;
    	vmxIO->GetChannelCapabilities(chan_info.index, chan_type, chan_info.capabilities);
    	return chan_info;
    }
    
    // Caller invokes VMX read function, returning status
    // int32_t newValue;
    // raspi::vmx->ReadSomething(int32_t &newValue, int32_t *status);
    // newValue = VerifiedRead(status, currValue, obj->lastValue);
    // return newValue; // *status should be 0, even if communication error occurred    
    
    template <typename T>
    T DefaultOnBoardCommError(int32_t *status, T readValue, T& defaultValue) {
        if (!status) return readValue;
        if (VMXERR_IO_BOARD_COMM_ERROR == *status) {
            *status = 0;
            readValue = defaultValue;
        } else if (0 == *status) {
            defaultValue = readValue; // No error - update deafultValue
        } else {
            readValue = defaultValue; // Other error - return defaultValue;
        }
        return readValue;
    }
 
    // Caller invokes VMX write function, returning status.
    // Caller also provides number of remaining retries, which is managed by this function
    // uint8_t retry_count = 3;
    // do {
    //   raspi::vmx->WriteSomething(int what, status);
    // } while ( VerifiedWrite(status, retry_count);
    // Return:  true:  a retry due to board comm error is required.
    //          false:  no board comm error occurred, so no retry is required.
    
    inline bool RetryWriteOnBoardCommError(int32_t *status, uint8_t& remaining_retry_count) {
        if (remaining_retry_count == 0) {
            return false;
        }
        if (!status) return false;
        if (VMXERR_IO_BOARD_COMM_ERROR == *status) {
            // Signal need for a retry following a board comm error, as long as there are remaining retry attempts.
            *status = 0;
            remaining_retry_count--;
            return (remaining_retry_count > 0);		
        } else if (0 == *status) {
            return false;
        } else {
            // In case of a non-board comm error, no retries should occur
            return false;
        }   
    }
    
    inline void ClearBoardCommErrorStatus(int32_t *status) {
        if ((status) && (VMXERR_IO_BOARD_COMM_ERROR == *status)) {
            *status = 0;
        }
    }
}

