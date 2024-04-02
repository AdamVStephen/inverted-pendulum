#ifndef MFI_ADC_SAMPLE_H_
#define MFI_ADC_SAMPLE_H_

#include "CompilerTypes.h"

namespace MFI {

const MARTe::uint32 USECS_PER_SEC = 1000000;

/**
 * @brief A single ADC sample at a given time
 */
struct ADCSample {

    ADCSample();
    ADCSample(MARTe::uint16 adc1, MARTe::uint16 adc2,
              MARTe::uint32 secs, MARTe::uint32 usecs);

    /**
     * @brief Get the seconds portion of the timestamp
     */
    MARTe::uint32  seconds() const;

    /**
     * @brief Get the microseconds portion of the timestamp
     */
    MARTe::uint32 microseconds() const;

    /**
     * @brief The value of the first ADC
     */
    MARTe::uint16 adc1_data;
    
    /**
     * @brief The value of the second ADC
     */
    MARTe::uint16 adc2_data;
    
    /**
     * @brief The time in microseconds
     */
    MARTe::uint64 timestamp;
    
    /**
     * @brief Whether or not the sample is valid
     */
    bool valid;
};

/**
 * @brief Check if a time lies between two samples
 */
bool time_is_between(MARTe::uint64 time_to_check, const ADCSample& first_sample, const ADCSample& second_sample);

} // namespace MFI

#endif // MFI_ADC_SAMPLE_H_
