#ifndef NATIVE_FILEHANDLER_H

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"
#include "FileHandlerInternal.h"
#include "RaspiMap.h"
#include "RaspiEnumConverter.h"
#include <VMXChannel.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <cmath>

#define NATIVE_FILEHANDLER_H

class Raspi_FileHandler {
    Raspi_EnumConverter* enums;

    static Raspi_Channel* genGroupChannels(Raspi_ChannelMap* raspiMap, const char* labelVal, rapidjson::Document* doc);
    static void genHandleGroup(Raspi_ChannelMap* raspiMap, std::string label, rapidjson::Document* doc);
    static void genEncoderChannel(Raspi_ChannelMap* raspiMap, std::string label, rapidjson::Document* doc);
    static void genSPIChannel(Raspi_ChannelMap* raspiMap, std::string label, rapidjson::Document* doc);
    static void genI2CChannel(Raspi_ChannelMap* raspiMap, std::string label, rapidjson::Document* doc);
    static void genSerialChannel(Raspi_ChannelMap* raspiMap, std::string label, rapidjson::Document* doc);

    Raspi_EnumConverter* readEnums();

    typedef void(* genFuncs)(Raspi_ChannelMap* raspiMap, std::string label, rapidjson::Document* doc);
    const std::map<std::string, genFuncs> allGenerators = {
        {"DIO", &genHandleGroup},
        {"Interrupt", &genHandleGroup},
        {"AnalogOutput", &genHandleGroup},
        {"AnalogInput", &genHandleGroup},
        {"AnalogTrigger", &genHandleGroup},
        {"Relay", &genHandleGroup},
        {"PWM", &genHandleGroup},
        {"DigitalPWM", &genHandleGroup},
        {"Counter", &genHandleGroup},
        {"FPGAEncoder", &genEncoderChannel},
        {"Encoder", &genEncoderChannel},
        {"SPI", &genSPIChannel},
        {"I2C", &genI2CChannel},
        {"Serial", &genSerialChannel}
    };

public:
    Raspi_ChannelMap* readChannelMap();
    Raspi_EnumConverter* getEnumConverter();
    Raspi_FileHandler();
    ~Raspi_FileHandler();
};


#endif //NATIVE_FILEHANDLER_H
