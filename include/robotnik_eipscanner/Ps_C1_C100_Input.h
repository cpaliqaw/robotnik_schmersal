#ifndef ROBOTNIK_EIPSCANNER_PSC1C100OUTPUT_H_
#define ROBOTNIK_EIPSCANNER_PSC1C100OUTPUT_H_

#include <bitset>
#include <ParameterObject.h>

using eipScanner::cip::CipUint;

namespace robotnik_eipscanner {

const CipUint kPsc1C100_ClassId{0x04};
const eipScanner::cip::CipUint kExplicitMessagingDefaultPort{0xAF12};
const float kPollHz{5.0f};
const int kBitsPerByte{8};
const unsigned kPs_C1_C100_InputSize{192 * kBitsPerByte};

const unsigned kPs_C1_C100_OutputSize{4 * kBitsPerByte};

enum class RunState
{
    /* Byte 0, first 4 bits (decimal values):
    (1-3 will be invisible, since the bus isn't initialized)
    1 init 2 self check 3 initializing bus
    4 running
    5 stopped (probably can't get data either)
    6 fatal error
    7 alarm
    */
    kInit = 1,
    kSelfCheck = 2,
    kInitializingBus = 3,
    kRunning = 4,
    kStopped = 5,
    kFatalError = 6,
    kAlarm = 7
};

enum class Instance : CipUint
{
    kWrite = 0x64,
    kRead = 0x65
};

enum class Attribute : CipUint
{
    kData = 0x3,
    kDataLength = 0x4
};

class Ps_C1_C100_Input
{
public:
    RunState GetRunState();
private:
    RunState run_state_;
    std::bitset<kPs_C1_C100_InputSize> data_;
};
} // namespace robotnik_eipscanner
#endif // ROBOTNIK_EIPSCANNER_PSC1C100OUTPUT_H_