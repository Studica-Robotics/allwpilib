/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/AnalogTrigger.h"

#include "AnalogInternal.h"
#include "hal/AnalogInput.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedHandleResource.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "RaspiInternal.h"
#include "AnalogTriggerInternal.h"

using namespace hal;

namespace hal {

	LimitedHandleResource<HAL_AnalogTriggerHandle, AnalogTriggerResource,
        kNumAnalogTriggers, HAL_HandleEnum::AnalogTrigger>*
        analogTriggerHandles;

	namespace init {
        void InitializeAnalogTrigger() {
            static LimitedHandleResource<HAL_AnalogTriggerHandle, AnalogTriggerResource,
                    kNumAnalogTriggers,
                    HAL_HandleEnum::AnalogTrigger>
                    atH;
            analogTriggerHandles = &atH;
        }
    }

    int32_t GetAnalogTriggerInputIndex(HAL_AnalogTriggerHandle handle, int32_t* status) {
        auto trigger = analogTriggerHandles->Get(handle);
        if (trigger == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return -1;
        }

        auto analog_port = analogInputHandles->Get(trigger->analogInputHandle);
        if (analog_port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return -1;
        }

        return analog_port->channel;
    }
}

static bool AllocateVMXAnalogTrigger(HAL_AnalogInputHandle anInHandle, std::shared_ptr<AnalogTriggerResource> port, int32_t* status) {
	VMXResourceHandle vmx_res_handle;
	VMXResourceIndex vmx_res_index = static_cast<VMXResourceIndex>(port->index);
	if (!raspi::vmxIO->GetResourceHandle(VMXResourceType::AnalogTrigger, vmx_res_index, vmx_res_handle, status)) {
		return false;
	}

	if (!raspi::vmxIO->AllocateResource(vmx_res_handle, status)) {
		return false;
	}

	if (!raspi::vmxIO->ActivateResource(vmx_res_handle, status)) {
		return false;
	}

	port->vmx_res_handle = vmx_res_handle;

	return true;
}

/* Can be used to deallocate either a DigitalIO or a PWMGenerator VMXPi Resource. */
static void DeallocateVMXAnalogTrigger(std::shared_ptr<AnalogTriggerResource> port, bool& isActive) {
	isActive = false;
	raspi::vmxIO->IsResourceActive(port->vmx_res_handle, isActive, raspi::vmxError);
	if (isActive) {
		raspi::vmxIO->DeallocateResource(port->vmx_res_handle, raspi::vmxError);
	}
	port->vmx_res_handle = 0;
}

extern "C" {
    HAL_AnalogTriggerHandle HAL_InitializeAnalogTrigger(HAL_AnalogInputHandle anInHandle,
                                                        int32_t* status) {
        hal::init::CheckInit();
        // ensure we are given a valid and active AnalogInput handle
        auto analog_port = analogInputHandles->Get(anInHandle);
        if (analog_port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return HAL_kInvalidHandle;
        }
        HAL_AnalogTriggerHandle handle = analogTriggerHandles->Allocate();
        if (handle == HAL_kInvalidHandle) {
            *status = NO_AVAILABLE_RESOURCES;
            return HAL_kInvalidHandle;
        }
        auto trigger = analogTriggerHandles->Get(handle);
        if (trigger == nullptr) {  // would only occur on thread issue
            *status = HAL_HANDLE_ERROR;
            return HAL_kInvalidHandle;
        }
        // TODO:  Review whether the default VMXPi analog trigger thresholds are appropriate here
        *trigger.get() = AnalogTriggerResource();
        trigger->analogInputHandle = anInHandle;
        trigger->index = getHandleIndex(anInHandle);

        if (AllocateVMXAnalogTrigger(anInHandle, trigger, status)) {
        	return handle;
        } else {
            *trigger.get() = AnalogTriggerResource();
            analogTriggerHandles->Free(handle);
            return HAL_kInvalidHandle;
        }
    }

    HAL_AnalogTriggerHandle HAL_InitializeAnalogTriggerDutyCycle(
        HAL_DutyCycleHandle dutyCycleHandle, int32_t* status) {
      hal::init::CheckInit();
#if 0
      // ensure we are given a valid and active DutyCycle handle
      auto dutyCycle = dutyCycleHandles->Get(dutyCycleHandle);
      if (dutyCycle == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return HAL_kInvalidHandle;
      }
      HAL_AnalogTriggerHandle handle = analogTriggerHandles->Allocate();
      if (handle == HAL_kInvalidHandle) {
        *status = NO_AVAILABLE_RESOURCES;
        return HAL_kInvalidHandle;
      }
      auto trigger = analogTriggerHandles->Get(handle);
      if (trigger == nullptr) {  // would only occur on thread issue
        *status = HAL_HANDLE_ERROR;
        return HAL_kInvalidHandle;
      }
      trigger->handle = dutyCycleHandle;
      trigger->index = static_cast<uint8_t>(getHandleIndex(handle));


      trigger->trigger.reset(tAnalogTrigger::create(trigger->index, status));
      trigger->trigger->writeSourceSelect_Channel(dutyCycle->index, status);
      trigger->trigger->writeSourceSelect_DutyCycle(true, status);
      return handle;
#endif
        *status = HAL_HANDLE_ERROR;
        return HAL_kInvalidHandle;
    }

    // Note:  caller owns the analog input handle, even after the trigger is freed.
    void HAL_CleanAnalogTrigger(HAL_AnalogTriggerHandle analogTriggerHandle, int32_t* status) {
        auto trigger = analogTriggerHandles->Get(analogTriggerHandle);
        if (trigger == nullptr) return;
        bool active;
        DeallocateVMXAnalogTrigger(trigger, active);
        *trigger.get() = AnalogTriggerResource();
        analogTriggerHandles->Free(analogTriggerHandle);
    }

    /**
     * Set the upper and lower limits of the analog trigger.
     *
     * The limits are given in ADC codes.  If oversampling is used, the units must
     * be scaled appropriately.
     *
     * @param lower The lower limit of the trigger in ADC codes (12-bit values).
     * @param upper The upper limit of the trigger in ADC codes (12-bit values).
     */
    void HAL_SetAnalogTriggerLimitsRaw(HAL_AnalogTriggerHandle analogTriggerHandle, int32_t lower, int32_t upper,
                                       int32_t* status) {
        auto trigger = analogTriggerHandles->Get(analogTriggerHandle);
        if (trigger == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }
        if (lower > upper) {
            *status = ANALOG_TRIGGER_LIMIT_ORDER_ERROR;
        }

        bool active;
        DeallocateVMXAnalogTrigger(trigger, active);
        trigger->vmx_config.SetThresholdLow(static_cast<uint16_t>(lower));
        trigger->vmx_config.SetThresholdHigh(static_cast<uint16_t>(upper));
        AllocateVMXAnalogTrigger(trigger->analogInputHandle, trigger, status);
    }

    void HAL_SetAnalogTriggerLimitsDutyCycle(
        HAL_AnalogTriggerHandle analogTriggerHandle, double lower, double upper,
        int32_t* status) {
      auto trigger = analogTriggerHandles->Get(analogTriggerHandle);
      if (trigger == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
      }
#if 0
      if (getHandleType(trigger->handle) != HAL_HandleEnum::DutyCycle) {
        *status = HAL_HANDLE_ERROR;
        return;
      }
      if (lower > upper) {
        *status = ANALOG_TRIGGER_LIMIT_ORDER_ERROR;
        return;
      }

      if (lower < 0.0 || upper > 1.0) {
        *status = PARAMETER_OUT_OF_RANGE;
        return;
      }

      int32_t scaleFactor =
          HAL_GetDutyCycleOutputScaleFactor(trigger->handle, status);
      if (*status != 0) {
        return;
      }

      trigger->trigger->writeLowerLimit(static_cast<int32_t>(scaleFactor * lower),
                                        status);
      trigger->trigger->writeUpperLimit(static_cast<int32_t>(scaleFactor * upper),
                                        status);
#endif
    }

    void HAL_SetAnalogTriggerLimitsVoltage(HAL_AnalogTriggerHandle analogTriggerHandle, double lower_volts, double upper_volts,
                                           int32_t* status) {
        auto trigger = analogTriggerHandles->Get(analogTriggerHandle);
        if (trigger == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }
        if (lower_volts > upper_volts) {
            *status = ANALOG_TRIGGER_LIMIT_ORDER_ERROR;
        }
        int32_t lower = HAL_GetAnalogVoltsToValue(trigger->analogInputHandle,lower_volts,status);
        int32_t upper = HAL_GetAnalogVoltsToValue(trigger->analogInputHandle,upper_volts,status);

        HAL_SetAnalogTriggerLimitsRaw(analogTriggerHandle, lower, upper, status);
    }

    // TODO:  Implement AnalogTrigger Averaging (using output from Oversample/Avg engine).
    void HAL_SetAnalogTriggerAveraged(HAL_AnalogTriggerHandle analogTriggerHandle, HAL_Bool useAveragedValue,
                                      int32_t* status) {
        auto trigger = analogTriggerHandles->Get(analogTriggerHandle);
        if (trigger == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

    //    AnalogTriggerData* triggerData = &SimAnalogTriggerData[trigger->wpiIndex];
    //
    //    if (triggerData->GetTriggerMode() == HALSIM_AnalogTriggerFiltered) {
    //        *status = INCOMPATIBLE_STATE;
    //        return;
    //    }
    //
    //    auto setVal = useAveragedValue ? HALSIM_AnalogTriggerAveraged
    //                                   : HALSIM_AnalogTriggerUnassigned;
    //    triggerData->SetTriggerMode(setVal);
    }

    // TODO:  Implement AnalogTrigger Filtering.
    // The analog trigger will operate with a 3 point average rejection filter. This
    // is designed to help with 360 degree pot applications for the period where
    // the pot crosses through zero.
    void HAL_SetAnalogTriggerFiltered(HAL_AnalogTriggerHandle analogTriggerHandle, HAL_Bool useFilteredValue,
                                      int32_t* status) {
        auto trigger = analogTriggerHandles->Get(analogTriggerHandle);
        if (trigger == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

    //    AnalogTriggerData* triggerData = &SimAnalogTriggerData[trigger->wpiIndex];
    //
    //    if (triggerData->GetTriggerMode() == HALSIM_AnalogTriggerAveraged) {
    //        *status = INCOMPATIBLE_STATE;
    //        return;
    //    }
    //
    //    auto setVal = useFilteredValue ? HALSIM_AnalogTriggerAveraged
    //                                   : HALSIM_AnalogTriggerUnassigned;
    //    triggerData->SetTriggerMode(setVal);
    }

    HAL_Bool HAL_GetAnalogTriggerInWindow(HAL_AnalogTriggerHandle analogTriggerHandle, int32_t* status) {
        auto trigger = analogTriggerHandles->Get(analogTriggerHandle);
        if (trigger == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }

        VMXIO::AnalogTriggerState analog_trigger_state;
        if (!raspi::vmxIO->AnalogTrigger_GetState(trigger->vmx_res_handle, analog_trigger_state, status)) {
        	return false;
        }

        return (analog_trigger_state == VMXIO::AnalogTriggerState::InWindow);
    }

    HAL_Bool HAL_GetAnalogTriggerTriggerState(HAL_AnalogTriggerHandle analogTriggerHandle, int32_t* status) {
        auto trigger = analogTriggerHandles->Get(analogTriggerHandle);
        if (trigger == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }

        VMXIO::AnalogTriggerState analog_trigger_state;
        if (!raspi::vmxIO->AnalogTrigger_GetState(trigger->vmx_res_handle, analog_trigger_state, status)) {
        	return false;
        }

        return (analog_trigger_state == VMXIO::AnalogTriggerState::AboveThreshold);
    }

    HAL_Bool HAL_GetAnalogTriggerOutput(HAL_AnalogTriggerHandle analogTriggerHandle, HAL_AnalogTriggerType type,
                                        int32_t* status) {
        if (type == HAL_Trigger_kInWindow) {
            return HAL_GetAnalogTriggerInWindow(analogTriggerHandle, status);
        } else if (type == HAL_Trigger_kState) {
            return HAL_GetAnalogTriggerTriggerState(analogTriggerHandle, status);
        } else {
            *status = ANALOG_TRIGGER_PULSE_OUTPUT_ERROR;
            return false;
        }
    }

    // Newly added in 2020 WPI Library HAL
    // Returns the trigger's "index" value, which is used by Raspi

    int32_t HAL_GetAnalogTriggerFPGAIndex(
        HAL_AnalogTriggerHandle analogTriggerHandle, int32_t* status) {
        auto trigger = analogTriggerHandles->Get(analogTriggerHandle);
        if (trigger == nullptr) {
          *status = HAL_HANDLE_ERROR;
          return -1;
        }
        return trigger->index;
    }

}
