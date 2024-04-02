#include "ADCSample.h"

using namespace MARTe;

namespace MFI {

ADCSample::ADCSample() :
        adc1_data(0u),
        adc2_data(0u),
        timestamp(0u),
        valid(false) {}

ADCSample::ADCSample(uint16 adc1, uint16 adc2,
                     uint32 secs, uint32 usecs) :
        adc1_data(adc1),
        adc2_data(adc2),
        timestamp(static_cast<MARTe::uint64>(secs) * USECS_PER_SEC + usecs),
        valid(true) {}

uint32 ADCSample::seconds() const { 
    return timestamp / USECS_PER_SEC; 
}

uint32 ADCSample::microseconds() const {
    return timestamp % USECS_PER_SEC; 
}

bool time_is_between(uint64 time_to_check, const ADCSample& first_sample, const ADCSample& second_sample) {
    return (time_to_check >= first_sample.timestamp) && (time_to_check <= second_sample.timestamp);
}

} // namespace MFI
