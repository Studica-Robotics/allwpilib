#include "RaspiMap.h"

// ----- Raspi Channel ----- //

VMXChannelInfo Raspi_Channel::getInfo() {
    return VMXChannelInfo(vmxIndex, vmxAbility);
}

VMXResourceIndex Raspi_Channel::getResourceIndex() {
    return EXTRACT_VMX_RESOURCE_INDEX(vmxResHandle);
}

// ----- Hal Channel Group ----- //

Raspi_ChannelGroup::Raspi_ChannelGroup(Raspi_Channel* newChannels) {
    channels = newChannels;
}

Raspi_Channel* Raspi_ChannelGroup::getChannel(int index) {
    return &channels[index];
}

// ----- Hal Channel Group: w/WPI_Handle ----- //

Raspi_HandledGroup::Raspi_HandledGroup(Raspi_Channel* newChannels, int handle) : Raspi_ChannelGroup(newChannels) {
    WPI_Handle = (hal::HAL_HandleEnum) handle;
}

hal::HAL_HandleEnum Raspi_HandledGroup::Raspi_HandledGroup::getHandle() {
    return WPI_Handle;
}

// ----- Raspi Channel Map ----- //

Raspi_ChannelMap::Raspi_ChannelMap() {
    groups["DIO"] = NULL;
    groups["Interrupt"] = NULL;
    groups["AnalogOutput"] = NULL;
    groups["AnalogInput"] = NULL;
    groups["AnalogTrigger"] = NULL;
    groups["Relay"] = NULL;
    groups["PWM"] = NULL;
    groups["DigitalPWM"] = NULL;
    groups["Counter"] = NULL;
    groups["FPGAEncoder"] = NULL;
    groups["Encoder"] = NULL;
    groups["SPI"] = NULL;
    groups["I2C"] = NULL;
    groups["Serial"] = NULL;
}

// ----- Raspi Map: Friend Creators ----- //

void Raspi_ChannelMap::setChannelAsReference(std::string target, std::string ref) {
    groups[target] = groups[ref];
}

void Raspi_ChannelMap::setGroup(std::string target, Raspi_ChannelGroup* group) {
    groups[target] = group;
}

// ----- Raspi Map: Getters ----- //

Raspi_Channel* Raspi_ChannelMap::getChannel(std::string label, int index) {
    return groups[label]->getChannel(index);
}

VMXChannelInfo Raspi_ChannelMap::getChannelInfo(std::string label, int index) {
    Raspi_Channel* channel = getChannel(label, index);
    if (channel == nullptr) {
    	return VMXChannelInfo();
    } else {
    	return VMXChannelInfo(channel->vmxIndex, channel->vmxAbility);
    }
}

// ----- Raspi Map: Setters ----- //

void Raspi_ChannelMap::initializeChannel(std::string label, int index, VMXChannelCapability ability, VMXResourceConfig* config) {
    Raspi_Channel* channel = getChannel(label, index);
    channel->vmxResConfig = config;
    channel->vmxAbility = ability;
}
