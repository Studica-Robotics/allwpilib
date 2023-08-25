/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/CANAPI.h"

#include <atomic>
#include <ctime>

#include <wpi/DenseMap.h>

#include "HALInitializer.h"
#include "hal/CAN.h"
#include "hal/Errors.h"
#include "hal/HAL.h"
#include "hal/handles/UnlimitedHandleResource.h"
#include "RaspiTime.h"
#include <chrono>
#include <thread>

using namespace hal;

namespace {
struct Receives {
  uint32_t lastTimeStamp;
  uint8_t data[8];
  uint8_t length;
};

struct CANStorage {
  HAL_CANManufacturer manufacturer;
  HAL_CANDeviceType deviceType;
  uint8_t deviceId;
  wpi::mutex mapMutex;
  wpi::SmallDenseMap<int32_t, int32_t> periodicSends;
  wpi::SmallDenseMap<int32_t, Receives> receives;
};
}  // namespace

static UnlimitedHandleResource<HAL_CANHandle, CANStorage, HAL_HandleEnum::CAN>*
    canHandles;

namespace hal {
namespace init {
void InitializeCANAPI() {
  static UnlimitedHandleResource<HAL_CANHandle, CANStorage, HAL_HandleEnum::CAN>
      cH;
  canHandles = &cH;
}
}  // namespace init
}  // namespace hal

static int32_t CreateCANId(CANStorage* storage, int32_t apiId) {
  int32_t createdId = 0;
  createdId |= (static_cast<int32_t>(storage->deviceType) & 0x1F) << 24;
  createdId |= (static_cast<int32_t>(storage->manufacturer) & 0xFF) << 16;
  createdId |= (apiId & 0x3FF) << 6;
  createdId |= (storage->deviceId & 0x3F);
  return createdId;
}

uint32_t HAL_GetCANPacketBaseTime(void) {
	uint64_t microseconds_now = raspi::vmxGetTime();
	uint32_t milliseconds_now = static_cast<uint32_t>(microseconds_now / 1000);
	return milliseconds_now;
}

HAL_CANHandle HAL_InitializeCAN(HAL_CANManufacturer manufacturer,
                                int32_t deviceId, HAL_CANDeviceType deviceType,
                                int32_t* status) {
  hal::init::CheckInit();
  auto can = std::make_shared<CANStorage>();

  auto handle = canHandles->Allocate(can);

  if (handle == HAL_kInvalidHandle) {
    *status = NO_AVAILABLE_RESOURCES;
    return HAL_kInvalidHandle;
  }

  can->deviceId = deviceId;
  can->deviceType = deviceType;
  can->manufacturer = manufacturer;

  return handle;
}

void HAL_CleanCAN(HAL_CANHandle handle) {
  auto data = canHandles->Free(handle);
  if (data == nullptr) {
    return;
  }

  std::lock_guard<wpi::mutex> lock(data->mapMutex);

  for (auto&& i : data->periodicSends) {
    int32_t s = 0;
    HAL_CAN_SendMessage(i.first, nullptr, 0, HAL_CAN_SEND_PERIOD_STOP_REPEATING,
                        &s);
    i.second = -1;
  }
}

void HAL_WriteCANPacket(HAL_CANHandle handle, const uint8_t* data,
                        int32_t length, int32_t apiId, int32_t* status) {
  auto can = canHandles->Get(handle);
  if (!can) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  auto id = CreateCANId(can.get(), apiId);

  HAL_CAN_SendMessage(id, data, length, HAL_CAN_SEND_PERIOD_NO_REPEAT, status);

  if (*status != 0) {
    return;
  }
  std::lock_guard<wpi::mutex> lock(can->mapMutex);
  can->periodicSends[apiId] = -1;
}

void HAL_WriteCANPacketRepeating(HAL_CANHandle handle, const uint8_t* data,
                                 int32_t length, int32_t apiId,
                                 int32_t repeatMs, int32_t* status) {
  auto can = canHandles->Get(handle);
  if (!can) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  auto id = CreateCANId(can.get(), apiId);

  HAL_CAN_SendMessage(id, data, length, repeatMs, status);

  if (*status != 0) {
    return;
  }
  std::lock_guard<wpi::mutex> lock(can->mapMutex);
  can->periodicSends[apiId] = repeatMs;
}

void HAL_WriteCANRTRFrame(HAL_CANHandle handle, int32_t length, int32_t apiId,
                          int32_t* status) {
  auto can = canHandles->Get(handle);
  if (!can) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  auto id = CreateCANId(can.get(), apiId);
  id |= HAL_CAN_IS_FRAME_REMOTE;
  uint8_t data[8];
  std::memset(data, 0, sizeof(data));

  HAL_CAN_SendMessage(id, data, length, HAL_CAN_SEND_PERIOD_NO_REPEAT, status);

  if (*status != 0) {
    return;
  }
  std::scoped_lock lock(can->mapMutex);
  can->periodicSends[apiId] = -1;
}

void HAL_StopCANPacketRepeating(HAL_CANHandle handle, int32_t apiId,
                                int32_t* status) {
  auto can = canHandles->Get(handle);
  if (!can) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  auto id = CreateCANId(can.get(), apiId);

  HAL_CAN_SendMessage(id, nullptr, 0, HAL_CAN_SEND_PERIOD_STOP_REPEATING,
                      status);

  if (*status != 0) {
    return;
  }
  std::lock_guard<wpi::mutex> lock(can->mapMutex);
  can->periodicSends[apiId] = -1;
}

void HAL_ReadCANPacketNew(HAL_CANHandle handle, int32_t apiId, uint8_t* data,
                          int32_t* length, uint64_t* receivedTimestamp,
                          int32_t* status) {
  auto can = canHandles->Get(handle);
  if (!can) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  uint32_t messageId = CreateCANId(can.get(), apiId);
  uint8_t dataSize = 0;
  uint32_t ts = 0;
  HAL_CAN_ReceiveMessage(&messageId, 0x1FFFFFFF, data, &dataSize, &ts, status);

  if (*status == 0) {
    std::lock_guard<wpi::mutex> lock(can->mapMutex);
    auto& msg = can->receives[messageId];
    msg.length = dataSize;
    msg.lastTimeStamp = ts;
    // The NetComm call placed in data, copy into the msg
    std::memcpy(msg.data, data, dataSize);
  }
  *length = dataSize;
  *receivedTimestamp = ts;
}

void HAL_ReadCANPacketLatest(HAL_CANHandle handle, int32_t apiId, uint8_t* data,
                             int32_t* length, uint64_t* receivedTimestamp,
                             int32_t* status) {
  auto can = canHandles->Get(handle);
  if (!can) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  uint32_t messageId = CreateCANId(can.get(), apiId);
  uint8_t dataSize = 0;
  uint32_t ts = 0;
  HAL_CAN_ReceiveMessage(&messageId, 0x1FFFFFFF, data, &dataSize, &ts, status);

  std::lock_guard<wpi::mutex> lock(can->mapMutex);
  if (*status == 0) {
    // fresh update
    auto& msg = can->receives[messageId];
    msg.length = dataSize;
    *length = dataSize;
    msg.lastTimeStamp = ts;
    *receivedTimestamp = ts;
    // The NetComm call placed in data, copy into the msg
    std::memcpy(msg.data, data, dataSize);
  } else {
    auto i = can->receives.find(messageId);
    if (i != can->receives.end()) {
      // Read the data from the stored message into the output
      std::memcpy(data, i->second.data, i->second.length);
      *length = i->second.length;
      *receivedTimestamp = i->second.lastTimeStamp;
      *status = 0;
    }
  }
}

void HAL_ReadCANPacketTimeout(HAL_CANHandle handle, int32_t apiId,
                              uint8_t* data, int32_t* length,
                              uint64_t* receivedTimestamp, int32_t timeoutMs,
                              int32_t* status) {
  auto can = canHandles->Get(handle);
  if (!can) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  uint32_t messageId = CreateCANId(can.get(), apiId);
  uint8_t dataSize = 0;
  uint32_t ts = 0;

  HAL_CAN_ReceiveMessage(&messageId, 0x1FFFFFFF, data, &dataSize, &ts, status);

  std::lock_guard<wpi::mutex> lock(can->mapMutex);
  if (*status == 0) {
    // fresh update
    auto& msg = can->receives[messageId];
    msg.length = dataSize;
    *length = dataSize;
    msg.lastTimeStamp = ts;
    *receivedTimestamp = ts;
    // The NetComm call placed in data, copy into the msg
    std::memcpy(msg.data, data, dataSize);
  } else {
    auto i = can->receives.find(messageId);
    if (i != can->receives.end()) {
      // Found, check if new enough
      uint32_t now = HAL_GetCANPacketBaseTime();
      if (now - i->second.lastTimeStamp > static_cast<uint32_t>(timeoutMs)) {
        // Timeout, return bad status
        *status = HAL_CAN_TIMEOUT;
        return;
      }
      // Read the data from the stored message into the output
      std::memcpy(data, i->second.data, i->second.length);
      *length = i->second.length;
      *receivedTimestamp = i->second.lastTimeStamp;
      *status = 0;
    }
  }
  if (*status != 0) {
    /* HACK:  temporary workaround while this issue is discussed w/REV. */
    /* Currently, responses to parameter (e.g., Firmware Version)       */
    /* requests are received after this function is called repeatedly.  */
    /* The current assumption is that this works on the reference       */
    /* platform due to differences in timing; so this workaround adds   */
    /* delay (equal to the requested timeout period), an approach that  */
    /* has been shown to provide sufficient delay for the response to   */
    /* be received as expected.                                         */
    if ((can->manufacturer == 5 /*REV Robotics*/) &&
        (can->deviceType == HAL_CAN_Dev_kMotorController)) {
	std::this_thread::sleep_for(std::chrono::milliseconds(timeoutMs));	
    }
  }
#if 0
  uint64_t currentTime = HAL_GetFPGATime(status);
  if( (messageId == 0x2052601) && (*status == 0)) {
	printf("HAL_ReadCanPacketTimeout (%u ms):  MessageID:  0x%x, status:  %d, length:  %d %02X %02X %02X %02X %02X %02X %02X %02X - timestamp:  %llu, curr fpga time:  %llu.\n", 
		timeoutMs, messageId, *status, *length, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], *receivedTimestamp, currentTime);
  } else if (messageId == 0x2052601) {
	printf("HAL_ReadCanPacketTimeout (%u ms):  NO RESPONSE for MessageID:  0x%x, status:  %d\n", timeoutMs, messageId, *status);
  }
#endif
}

#if 0

// This functon has been removed from the 2020 version of the WPI Library HAL

void HAL_ReadCANPeriodicPacket(HAL_CANHandle handle, int32_t apiId,
                               uint8_t* data, int32_t* length,
                               uint64_t* receivedTimestamp, int32_t timeoutMs,
                               int32_t periodMs, int32_t* status) {
  auto can = canHandles->Get(handle);
  if (!can) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  uint32_t messageId = CreateCANId(can.get(), apiId);

  {
    std::lock_guard<wpi::mutex> lock(can->mapMutex);
    auto i = can->receives.find(messageId);
    if (i != can->receives.end()) {
      // Found, check if new enough
      uint32_t now = GetPacketBaseTime();
      if (now - i->second.lastTimeStamp < static_cast<uint32_t>(periodMs)) {
        // Read the data from the stored message into the output
        std::memcpy(data, i->second.data, i->second.length);
        *length = i->second.length;
        *receivedTimestamp = i->second.lastTimeStamp;
        *status = 0;
        return;
      }
    }
  }

  uint8_t dataSize = 0;
  uint32_t ts = 0;
  HAL_CAN_ReceiveMessage(&messageId, 0x1FFFFFFF, data, &dataSize, &ts, status);

  std::lock_guard<wpi::mutex> lock(can->mapMutex);
  if (*status == 0) {
    // fresh update
    auto& msg = can->receives[messageId];
    msg.length = dataSize;
    *length = dataSize;
    msg.lastTimeStamp = ts;
    *receivedTimestamp = ts;
    // The NetComm call placed in data, copy into the msg
    std::memcpy(msg.data, data, dataSize);
  } else {
    auto i = can->receives.find(messageId);
    if (i != can->receives.end()) {
      // Found, check if new enough
      uint32_t now = GetPacketBaseTime();
      if (now - i->second.lastTimeStamp > static_cast<uint32_t>(timeoutMs)) {
        // Timeout, return bad status
        *status = HAL_CAN_TIMEOUT;
        return;
      }
      // Read the data from the stored message into the output
      std::memcpy(data, i->second.data, i->second.length);
      *length = i->second.length;
      *receivedTimestamp = i->second.lastTimeStamp;
      *status = 0;
    }
  }
}

#endif