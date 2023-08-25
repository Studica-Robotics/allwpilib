#include "RaspiDriveData.h"
#include <cstring>

// Mutual Exclusion & Semaphor for Cached Driver Station data
wpi::mutex Raspi_DriveData::memLock;
wpi::condition_variable Raspi_DriveData::memSignal;

// Cached Driver Station data
HAL_AllianceStationID Raspi_DriveData::allianceID;
HAL_MatchInfo Raspi_DriveData::matchInfo = {};
HAL_ControlWord Raspi_DriveData::controlWord;
Raspi_SharedJoystick Raspi_DriveData::joysticks[HAL_kMaxJoysticks] = {};

volatile uint32_t Raspi_DriveData::newDSDataAvailableCounter = 0;
volatile float Raspi_DriveData::matchTime = 0.0;

void Raspi_DriveData::unlockAndSignal() {
    memSignal.notify_all();
    memLock.unlock();
}

void Raspi_DriveData::initializeDriveData() {

	// Initialize all drive station data to reasonable defaults
	Raspi_DriveData::allianceID = HAL_AllianceStationID_kRed1;

	Raspi_DriveData::controlWord.enabled = 0;
	Raspi_DriveData::controlWord.autonomous = 0;
	Raspi_DriveData::controlWord.test = 0;
	Raspi_DriveData::controlWord.eStop= 0;
	Raspi_DriveData::controlWord.fmsAttached = 0;
	Raspi_DriveData::controlWord.dsAttached = 0;
	Raspi_DriveData::controlWord.control_reserved = 0;

	Raspi_DriveData::matchInfo.eventName[0] = '\0';
	Raspi_DriveData::matchInfo.gameSpecificMessage[0] = '\0';

	// All joysticks should default to having zero axes, povs and buttons, so
	// uninitialized memory doesn't get sent to speed controllers.
	for (unsigned int i = 0; i < HAL_kMaxJoysticks; i++) {
		Raspi_DriveData::joysticks[i].joyAxes.count = 0;
		Raspi_DriveData::joysticks[i].joyPOVs.count = 0;
		Raspi_DriveData::joysticks[i].joyButtons.count = 0;
		Raspi_DriveData::joysticks[i].joyDescriptor.axisCount = 0;
		Raspi_DriveData::joysticks[i].joyDescriptor.povCount = 0;
		Raspi_DriveData::joysticks[i].joyDescriptor.buttonCount = 0;

		Raspi_DriveData::joysticks[i].joyDescriptor.isXbox = 0;
		Raspi_DriveData::joysticks[i].joyDescriptor.type = -1;
		Raspi_DriveData::joysticks[i].joyDescriptor.name[0] = '\0';

		Raspi_DriveData::joysticks[i].outputs = 0;
		Raspi_DriveData::joysticks[i].leftRumble = 0;
		Raspi_DriveData::joysticks[i].rightRumble = 0;
	}
}

void Raspi_DriveData::updateMatchIdentifyInfo(char *event_name, uint8_t match_type, uint16_t match_number, uint8_t replay_number)
{
    memLock.lock();
    strcpy((char *)matchInfo.eventName, event_name);
    matchInfo.matchType = (HAL_MatchType)match_type;
    matchInfo.matchNumber = match_number;
    matchInfo.replayNumber = replay_number;
    unlockAndSignal();
}

void Raspi_DriveData::updateMatchTimeUnsafe(float currMatchTime) {
	matchTime = currMatchTime;
}

void Raspi_DriveData::updateMatchGameSpecificMessage(uint8_t msg_len, uint8_t *msg_data)
{
    if (msg_len > sizeof(matchInfo.gameSpecificMessage)) {
    	msg_len = sizeof(matchInfo.gameSpecificMessage);
    }

    memLock.lock();
    matchInfo.gameSpecificMessageSize = msg_len;
    memcpy(matchInfo.gameSpecificMessage, msg_data, msg_len);
    unlockAndSignal();
}

// --- Update: ControlWord --- //

void Raspi_DriveData::updateControlWordAndAllianceIDUnsafe(bool enabled, bool auton, bool test, bool eStop, bool fms, bool ds, HAL_AllianceStationID id) {
    controlWord.enabled = enabled;
    controlWord.autonomous = auton;
    controlWord.test = test;
    controlWord.eStop = eStop;
    controlWord.fmsAttached = fms;
    controlWord.dsAttached = ds;

    allianceID = id;
    newDSDataAvailableCounter = newDSDataAvailableCounter + 1; // '++'expression of 'volatile' type is deprecated
}

void Raspi_DriveData::updateControlWordAndAllianceID(bool enabled, bool auton, bool test, bool eStop, bool fms, bool ds, HAL_AllianceStationID id) {
    memLock.lock();
    updateControlWordAndAllianceIDUnsafe(enabled, auton, test, eStop, fms, ds, id);
    unlockAndSignal();
}

void Raspi_DriveData::updateDSAttached(bool attached)
{
    memLock.lock();
    controlWord.dsAttached = attached;
    newDSDataAvailableCounter = newDSDataAvailableCounter + 1;
    unlockAndSignal();
}

uint32_t Raspi_DriveData::getNewDSDataAvailableCounter() {
	return newDSDataAvailableCounter;
}

// --- Update: Joystick --- //

void Raspi_DriveData::updateJoyAxis(int joyNumber, int16_t axisCount, int8_t* axes) {
    memLock.lock();
    updateJoyAxisUnsafe(joyNumber, axisCount, axes);
    unlockAndSignal();
}

void Raspi_DriveData::updateJoyPOV(int joyNumber, int povsCount, uint16_t* povs) {
    memLock.lock();
    updateJoyPOVUnsafe(joyNumber, povsCount, povs);
    unlockAndSignal();
}

void Raspi_DriveData::updateJoyButtons(int joyNumber, uint8_t buttonCount, uint32_t buttons) {
    memLock.lock();
    updateJoyButtonsUnsafe(joyNumber, buttonCount, buttons);
    unlockAndSignal();
}

void Raspi_DriveData::updateLock()
{
    memLock.lock();
}

void Raspi_DriveData::updateUnlockAndSignal()
{
    unlockAndSignal();
}

void Raspi_DriveData::updateJoyAxisUnsafe(int joyNumber, int16_t axisCount, int8_t* axes)
{
    // If change in axis count, reinitialize all axes to 0 before updating
    if (joysticks[joyNumber].joyAxes.count != axisCount) {
        joysticks[joyNumber].joyAxes.count = axisCount;
        for (int index = 0; index < HAL_kMaxJoystickAxes; index++) {
            joysticks[joyNumber].joyAxes.axes[index] = 0.0f;
        }
    }
    for (int index = 0; index < axisCount; index++) {
    	float joystickValue = ((float)axes[index]) / 127;
    	if (joystickValue < -1.0f) {
    		joystickValue = -1.0f;
    	} else if (joystickValue > 1.0f) {
    		joystickValue = 1.0f;
    	}
        joysticks[joyNumber].joyAxes.axes[index] = joystickValue;
    }
}
void Raspi_DriveData::updateJoyPOVUnsafe(int joyNumber, int povsCount, uint16_t* povs)
{
    // If change in pov count, reinitialize all povs to 0 before updating
    if (joysticks[joyNumber].joyPOVs.count != povsCount) {
    	joysticks[joyNumber].joyPOVs.count = povsCount;
        for (int index = 0; index < HAL_kMaxJoystickPOVs; index++) {
            joysticks[joyNumber].joyPOVs.povs[index] = 0;
        }
    }
    for (int index = 0; index < povsCount; index++) {
        joysticks[joyNumber].joyPOVs.povs[index] = povs[index];
        index++;
    }
}
void Raspi_DriveData::updateJoyButtonsUnsafe(int joyNumber, uint8_t buttonCount, uint32_t buttons)
{
    joysticks[joyNumber].joyButtons.count = buttonCount;
    joysticks[joyNumber].joyButtons.buttons = buttons;
}

void Raspi_DriveData::updateJoyDescriptor(int joyNumber, HAL_JoystickDescriptor* desc) {
    memLock.lock();
    joysticks[joyNumber].joyDescriptor = *desc;
    unlockAndSignal();
}

void Raspi_DriveData::updateJoyOutputs(int32_t joyNumber, int64_t outputs, int32_t leftRumble, int32_t rightRumble) {
	if (joyNumber < HAL_kMaxJoysticks) {
		memLock.lock();
		 Raspi_SharedJoystick* joy = &joysticks[joyNumber];
		 joy->outputs = outputs;
		 joy->leftRumble = leftRumble;
		 joy->rightRumble = rightRumble;
		 memLock.lock();
		 // TODO:  Signal DriverComms that new joystick output values are available.
	}
}

//// ----- HAL Data: Scribe ----- ////

void Raspi_DriveData::scribeMatchInfo(HAL_MatchInfo* info) {
    memLock.lock();
    std::strcpy((char *)info->eventName, matchInfo.eventName);
    info->matchType = matchInfo.matchType;
    info->matchNumber = matchInfo.matchNumber;
    info->replayNumber = matchInfo.replayNumber;
    std::strcpy((char *)info->gameSpecificMessage, (char *)matchInfo.gameSpecificMessage);
    memLock.unlock();
}

void Raspi_DriveData::scribeJoyAxes(int joyNumber, HAL_JoystickAxes* axes) {
    memLock.lock();
    HAL_JoystickAxes* dataAxes = &joysticks[joyNumber].joyAxes;

    axes->count = dataAxes->count;
    std::memcpy(axes->axes, dataAxes->axes, sizeof(axes->axes));
    memLock.unlock();
}

void Raspi_DriveData::scribeJoyPOVs(int joyNumber, HAL_JoystickPOVs* povs) {
    memLock.lock();
    HAL_JoystickPOVs* dataPOVs = &joysticks[joyNumber].joyPOVs;

    povs->count = dataPOVs->count;
    std::memcpy(povs->povs, dataPOVs->povs, sizeof(povs->povs));
    memLock.unlock();
}

void Raspi_DriveData::scribeJoyButtons(int joyNumber, HAL_JoystickButtons* buttons) {
    memLock.lock();
    HAL_JoystickButtons* dataButtons = &joysticks[joyNumber].joyButtons;

    buttons->count = dataButtons->count;
    buttons->buttons = dataButtons->buttons;
    memLock.unlock();
}

void Raspi_DriveData::scribeJoyDescriptor(int joyNumber, HAL_JoystickDescriptor* desc) {
    memLock.lock();
    HAL_JoystickDescriptor* dataDesc = &joysticks[joyNumber].joyDescriptor;

    desc->isXbox = dataDesc->isXbox;
    desc->type = dataDesc->type;
    std::strcpy(desc->name, dataDesc->name);
    desc->axisCount = dataDesc->axisCount;
    std::memcpy(desc->axisTypes, dataDesc->axisTypes, sizeof(desc->axisTypes));
    desc->buttonCount = dataDesc->buttonCount;
    desc->povCount = dataDesc->povCount;
    memLock.unlock();
}

void Raspi_DriveData::scribeJoyName(int joyNumber, char* name) {
    memLock.lock();
    std::strcpy(name, joysticks[joyNumber].joyDescriptor.name);
    memLock.unlock();
}

//// ----- HAL Data: Read ----- ////

HAL_ControlWord Raspi_DriveData::readControlWord() {
    std::lock_guard<wpi::mutex> lock(memLock);
    return controlWord;
}

HAL_AllianceStationID Raspi_DriveData::readAllianceID() {
    std::lock_guard<wpi::mutex> lock(memLock);
    return allianceID;
}

HAL_MatchType Raspi_DriveData::readMatchType() {
    std::lock_guard<wpi::mutex> lock(memLock);
    return matchInfo.matchType;
}

HAL_Bool Raspi_DriveData::readJoyIsXbox(int joyNumber) {
    std::lock_guard<wpi::mutex> lock(memLock);
    return joysticks[joyNumber].joyDescriptor.isXbox;
}

int32_t Raspi_DriveData::readJoyType(int joyNumber) {
    std::lock_guard<wpi::mutex> lock(memLock);
    return joysticks[joyNumber].joyDescriptor.type;
}

int32_t Raspi_DriveData::readJoyAxisType(int joyNumber, int axisNumber) {
    std::lock_guard<wpi::mutex> lock(memLock);
    return joysticks[joyNumber].joyDescriptor.axisTypes[axisNumber];
}

float Raspi_DriveData::readMatchTime() {
	return matchTime;
}

//// ----- HAL Data: Get ----- ////

wpi::mutex* Raspi_DriveData::getMutex() {
    return &memLock;
}

wpi::condition_variable* Raspi_DriveData::getDataSignal() {
    return &memSignal;
}
