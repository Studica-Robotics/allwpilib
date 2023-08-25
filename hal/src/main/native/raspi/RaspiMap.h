#ifndef NATIVE_CHANNELMAP_H
#define NATIVE_CHANNELMAP_H

#include "hal/handles/HandlesInternal.h"
#include "Translator/include/RaspiEnumConverter.h"
#include <VMXChannel.h>
#include <VMXResourceConfig.h>
#include <vector>
#include <string>
#include <map>

struct Raspi_Channel {
    int wpiIndex;
    bool unsupported;
    bool isMXP;

    VMXChannelIndex vmxIndex;
    VMXChannelCapability vmxAbility;

    VMXResourceHandle vmxResHandle;
    VMXResourceConfig* vmxResConfig;

    VMXChannelInfo getInfo();
    VMXResourceIndex getResourceIndex();
};

class Raspi_ChannelGroup {
    Raspi_Channel* channels;
public:
    Raspi_ChannelGroup(Raspi_Channel* newChannels);
    Raspi_Channel* getChannel(int index);
};

class Raspi_HandledGroup : public Raspi_ChannelGroup {
    hal::HAL_HandleEnum WPI_Handle;

public:
    Raspi_HandledGroup(Raspi_Channel* newChannels, int handle);
    hal::HAL_HandleEnum getHandle();
};

class Raspi_ChannelMap {
    std::map<std::string, Raspi_ChannelGroup*> groups;

    friend class Raspi_FileHandler;
    void setChannelAsReference(std::string target, std::string ref);
    void setGroup(std::string target, Raspi_ChannelGroup* group);
public:
    Raspi_ChannelMap();
    Raspi_Channel* getChannel(std::string label, int index);
    VMXChannelInfo getChannelInfo(std::string label, int index);

    void initializeChannel(std::string label, int index, VMXChannelCapability ability, VMXResourceConfig* config);
};

#endif //NATIVE_CHANNELMAP_H
