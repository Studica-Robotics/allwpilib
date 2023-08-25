/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


// TODO: Cleanup/Rewrite whole file 

#include "hal/Interrupts.h"

#include <memory>

#include <wpi/SafeThread.h>

#include "DigitalInternal.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedHandleResource.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "InterruptsInternal.h"
#include <VMXChannel.h>
#include <VMXResource.h>
#include <VMXIO.h>
#include <atomic>

using namespace hal;

static constexpr int64_t kHalWaitResultInterruptTimeout = 0x0;

class InterruptSafeThread : public wpi::SafeThread {
	void Main() {}
};

class BlockingInterruptManager: public wpi::detail::SafeThreadProxyBase {
public:
	BlockingInterruptManager(wpi::SafeThread *p_thread) :
			SafeThreadProxyBase(std::shared_ptr<wpi::SafeThread>(p_thread)) {
	}

	std::atomic<uint32_t> m_mask;

	// Invoked from interrupt handler
	// void Notify(uint32_t mask) {
	// 	m_mask = mask;
	// 	if (m_thread != nullptr) {
	// 		m_thread->m_cond.notify_one();
	// 	}
	// }

	// int64_t Wait(uint64_t timeout_us) {
	// 	std::unique_lock<wpi::mutex>& lock = GetLock();
	// 	if (m_thread->m_cond.wait_for(lock, std::chrono::microseconds(timeout_us))
	// 			== std::cv_status::timeout) {
	// 		return kHalWaitResultInterruptTimeout;
	// 	} else {
	// 		return m_mask;
	// 	}
	// }
};

// static void blockingInterruptHandler(uint32_t mask, void* param) {
// 	static_cast<BlockingInterruptManager*>(param)->Notify(mask);
// }

struct InterruptResource {
	VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
			VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
	VMXChannelInfo vmx_chan_info;
	InterruptConfig interrupt_config = InterruptConfig(false);
	HAL_InterruptHandle p_userHandler;
	// std::atomic<HAL_InterruptHandlerFunction> p_userHandler;
	void* param = nullptr;
	void Init() {
		vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
				VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
		vmx_chan_info = VMXChannelInfo();
		interrupt_config = InterruptConfig(false);
		p_userHandler = 0; 
		param = nullptr;
	}
};

static LimitedHandleResource<HAL_InterruptHandle, InterruptResource, kNumInterrupts,
		HAL_HandleEnum::Interrupt>* interruptHandles = 0;

static std::atomic<InterruptResource *> handler_lookup_table[kNumInterrupts] = {};
static std::atomic<uint8_t> hal_int_index_lookup_table[kNumVMXPiInterrupts] = {};

static constexpr uint8_t kInvalidHALInterruptIndex = 255;

namespace hal {
namespace init {
void InitializeInterrupts() {
	static LimitedHandleResource<HAL_InterruptHandle, InterruptResource, kNumInterrupts,
			HAL_HandleEnum::Interrupt> iH;
	interruptHandles = &iH;
	for (int i = 0; i < kNumInterrupts; i++) {
		handler_lookup_table[i].store(nullptr);
	}
	for (int i = 0; i < kNumVMXPiInterrupts; i++) {
		hal_int_index_lookup_table[i] = kInvalidHALInterruptIndex;
	}
}
}  // namespace init
}  // namespace hal

/* This single function handles and dispatches all VMX-pi Interrupts */
static void VMXPi_InterruptHandler(uint32_t vmx_channel_index,
		InterruptEdgeType edge, void* param, uint64_t timestamp_us) {

	// Convert vmx_channel_index to wpi hal resource handle
	VMXResourceIndex interrupt_resource_index =
			static_cast<VMXResourceIndex>(vmx_channel_index);

	// get corresponding HAL_Interrupt object
	if (interrupt_resource_index >= kNumVMXPiInterrupts)
		return;

	uint8_t hal_int_index = hal_int_index_lookup_table[interrupt_resource_index];
	if (hal_int_index != kInvalidHALInterruptIndex) {
		InterruptResource *interrupt = handler_lookup_table[hal_int_index].load();
		if (interrupt != nullptr) {
			// The interrupt mask implemented in the WPI simulator HAL uses this mask format:
			// Bits 0-7:  If set, is rising edge interrupt.  Bit position indicates interrupt number (0-7)
			// Bits 8-15: If set, is falling edge interrupt.  Bit position indicates interrupt number (0-7)
			// NOTE:  This behavior limits the number of interrupts to 8
			// HAL_InterruptHandle* p_handler = interrupt->p_userHandler;
			// if (p_handler) {
			// 	uint32_t mask =
			// 			(edge == InterruptEdgeType::RISING_EDGE_INTERRUPT) ?
			// 					(1 << hal_int_index) :
			// 					(1 << (hal_int_index + 8));
			// 	p_handler(mask, interrupt->param);
			// } else {
			// 	// Interrupt handler invoked for interrupt w/no registered handlers
			// }
		}
	}
}

extern "C" {

/**
 * API Function
 *
 * Allocates one of the interrupt handles (in the reference implementation, they are limited,
 * but on VMX-pi there is one for each VMXChannelIndex (both digital and analog).
 *
 * Note that at this point, the input channel is not yet known.
 *
 **/

HAL_InterruptHandle HAL_InitializeInterrupts(int32_t* status) {
	hal::init::CheckInit();
	HAL_InterruptHandle handle = interruptHandles->Allocate();
	if (handle == HAL_kInvalidHandle) {
		*status = NO_AVAILABLE_RESOURCES;
		return HAL_kInvalidHandle;
	}

	auto anInterrupt = interruptHandles->Get(handle);
	if (anInterrupt == nullptr) {
		*status = NO_AVAILABLE_RESOURCES;
		return HAL_kInvalidHandle;
	}

	/* Since the input source channel is not yet known, */
	/* Defer VMX Resource allocation until later.       */
	anInterrupt->interrupt_config = InterruptConfig();
	anInterrupt->p_userHandler = 0;
	anInterrupt->param = nullptr;

	return handle;
}

/**
 * API Function
 *
 * Cancel interrupts on this device.
 *
 * This deallocates all structures and disables any interrupts.
 **/
void HAL_CleanInterrupts(HAL_InterruptHandle interruptHandle) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	int32_t* status = NULL;
	if (anInterrupt == nullptr) {
		return;
	}

	int16_t hal_interrupt_handle_index = getHandleIndex(interruptHandle);
	if (hal_interrupt_handle_index >= kNumInterrupts) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (INVALID_VMX_RESOURCE_HANDLE(anInterrupt->vmx_res_handle)) {
		*status = VMXERR_IO_INVALID_RESOURCE_HANDLE;
		return;
	}

	VMXResourceIndex vmx_interrupt_res_index = EXTRACT_VMX_RESOURCE_INDEX(anInterrupt->vmx_res_handle);
	if (vmx_interrupt_res_index >= kNumVMXPiInterrupts) {
		*status = VMXERR_IO_INVALID_RESOURCE_INDEX;
		return;
	}

	raspi::vmxIO->Interrupt_SetEnabled(anInterrupt->vmx_res_handle, false,
			status);

	raspi::vmxIO->DeallocateResource(anInterrupt->vmx_res_handle, status);
	anInterrupt->Init();
	anInterrupt->p_userHandler = 0;
	anInterrupt->param = nullptr;

	handler_lookup_table[hal_interrupt_handle_index].store(nullptr);
	hal_int_index_lookup_table[vmx_interrupt_res_index] =
			kInvalidHALInterruptIndex;

	// void* param = anInterrupt->param;
	interruptHandles->Free(interruptHandle);

	return;
}

/**
 * In synchronous mode, wait for the defined interrupt to occur.
 * @param timeout Timeout in seconds
 * @param ignorePrevious If true, ignore interrupts that happened before
 * waitForInterrupt was called.
 * @return The mask of interrupts that fired:
 * 0x000:  Timeout
 * 0x001:  Rising Edge
 * 0x100:  Falling Edge
 * 0x101:  Both Rising and Falling Edge
 *
 * NOTE:  HAL_RequestInterrupts will have already been invoked before this.
 *
 */

int64_t HAL_WaitForInterrupt(HAL_InterruptHandle interruptHandle,
		double timeout, HAL_Bool ignorePrevious, int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	InterruptSafeThread ist; // encapsulates condition var, mutex and atomic "quit" boolean
	BlockingInterruptManager istproxy(&ist); // encapsulates mutex lock and SafeThread *

	anInterrupt->param = &istproxy;
	anInterrupt->p_userHandler = 0;
	// anInterrupt->p_userHandler = static_cast<int>(blockingInterruptHandler(0, nullptr));

	// uint64_t timeout_us = static_cast<uint64_t>(timeout * 1e+6);
	// int64_t result = istproxy.Wait(timeout_us);

	// return result;
	return 0;
}

int64_t HAL_WaitForMultipleInterrupts(HAL_InterruptHandle interruptHandle,
		int64_t mask, double timeout, HAL_Bool ignorePrevious, int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	InterruptSafeThread ist;
	BlockingInterruptManager istproxy(&ist);

	anInterrupt->param = &istproxy;
	anInterrupt->p_userHandler = 0;
	// anInterrupt->p_userHandler = static_cast<int>(blockingInterruptHandler(mask, nullptr));

	// uint64_t timeout_us = static_cast<uint64_t>(timeout * 1e+6);
	// int64_t result = istproxy.Wait(timeout_us);

	// return result;
	return 0;
}

/**
 * Return the timestamp for the rising interrupt that occurred most recently.
 * This is in the same time domain as GetClock().
 * @return Timestamp in seconds since boot.
 */
int64_t HAL_ReadInterruptRisingTimestamp(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	uint64_t timestamp_microseconds;
	if (!raspi::vmxIO->Interrupt_GetLastRisingEdgeTimestampMicroseconds(
			anInterrupt->vmx_res_handle, timestamp_microseconds, status)) {
		return 0;
	}

	return timestamp_microseconds;
}

/**
 * Return the timestamp for the falling interrupt that occurred most recently.
 * This is in the same time domain as GetClock().
 * @return Timestamp in seconds since boot.
 */
int64_t HAL_ReadInterruptFallingTimestamp(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	uint64_t timestamp_microseconds;
	if (!raspi::vmxIO->Interrupt_GetLastFallingEdgeTimestampMicroseconds(
			anInterrupt->vmx_res_handle, timestamp_microseconds, status)) {
		return 0;
	}

	return timestamp_microseconds;
}

/**
 * API Function
 *
 * Attach an asynchronous interrupt handler to the interrupt handle
 * previously returned from HAL_RequestInterrupts.  This handler will
 * be invoked asynchronously when the interrupt fires.
 *
 * This function is also invoked for synchronous interrupt handling.
 *
 * wpiblic invokes this before HAL_EnableInterrupts is invoked.
 *
 * TODO:  Determine what the purpose of the analogTriggerType in this case is.
 */

void HAL_RequestInterrupts(HAL_InterruptHandle interruptHandle,
		HAL_Handle digitalSourceHandle, HAL_AnalogTriggerType analogTriggerType,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	VMXChannelInfo vmx_src_channel_info;
	VMXResourceIndex vmx_interrupt_res_index;
	if (!GetVMXInterruptResourceIndexForDigitalSourceHandle(digitalSourceHandle,
			vmx_interrupt_res_index, vmx_src_channel_info, status)) {
		return;
	}

	if (vmx_interrupt_res_index >= kNumVMXPiInterrupts) {
		*status = VMXERR_IO_INVALID_RESOURCE_INDEX;
		return;
	}

	int16_t hal_interrupt_handle_index = getHandleIndex(interruptHandle);
	if (hal_interrupt_handle_index >= kNumInterrupts) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	hal_int_index_lookup_table[vmx_interrupt_res_index] =
			hal_interrupt_handle_index;
	handler_lookup_table[hal_interrupt_handle_index].store(anInterrupt.get());

	anInterrupt->interrupt_config.SetHandler(VMXPi_InterruptHandler);

	if (!raspi::vmxIO->ActivateSinglechannelResource(vmx_src_channel_info,
			&anInterrupt->interrupt_config, anInterrupt->vmx_res_handle,
			status)) {
		handler_lookup_table[hal_interrupt_handle_index].store(nullptr);
		hal_int_index_lookup_table[vmx_interrupt_res_index] =
				kInvalidHALInterruptIndex;
		return;
	}
}

/**
 * API Function
 *
 * Configures the previously-allocated interrupt handle to trigger on
 * rising edge, falling edge, or both edges.
 *
 * Per wpilibc, this API function must be set *before* HAL_RequestInterrupts.
 */

void HAL_SetInterruptUpSourceEdge(HAL_InterruptHandle interruptHandle,
		HAL_Bool risingEdge, HAL_Bool fallingEdge, int32_t* status) {
	auto interrupt = interruptHandles->Get(interruptHandle);
	if (interrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (risingEdge) {
		if (fallingEdge) {
			interrupt->interrupt_config.SetEdge(
					InterruptConfig::InterruptEdge::BOTH);
		} else {
			interrupt->interrupt_config.SetEdge(
					InterruptConfig::InterruptEdge::RISING);
		}
	} else {
		interrupt->interrupt_config.SetEdge(
				InterruptConfig::InterruptEdge::FALLING);
	}
}

void HAL_ReleaseWaitingInterrupt(HAL_InterruptHandle interruptHandle,
                                 int32_t* status) {
  auto anInterrupt = interruptHandles->Get(interruptHandle);
  if (anInterrupt == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  int16_t hal_interrupt_handle_index = getHandleIndex(interruptHandle);
  if (hal_interrupt_handle_index >= kNumInterrupts) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  if (INVALID_VMX_RESOURCE_HANDLE(anInterrupt->vmx_res_handle)) {
    *status = VMXERR_IO_INVALID_RESOURCE_HANDLE;
    return;
  }

  VMXResourceIndex vmx_interrupt_res_index = EXTRACT_VMX_RESOURCE_INDEX(anInterrupt->vmx_res_handle);
  if (vmx_interrupt_res_index >= kNumVMXPiInterrupts) {
    *status = VMXERR_IO_INVALID_RESOURCE_INDEX;
    return;
  }


#if 0

  raspi::vmxIO->DeallocateResource(anInterrupt->vmx_res_handle, status);

  HAL_EnableInterrupts(interruptHandle, status);
#endif

  std::printf("HAL_ReleaseWaitingInterrupt() - TODO:  Implementatation goes here.");

}


}  // extern "C"
