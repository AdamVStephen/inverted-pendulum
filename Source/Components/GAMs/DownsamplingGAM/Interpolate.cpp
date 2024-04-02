#include "Interpolate.h"

using namespace MARTe;

namespace MFI {

uint64 get_last_resampling_time(const ADCSample& sample, uint32 out_rate) {
    float64 timestamp_usecs = sample.microseconds();
    float64 delta_t_usecs = static_cast<float64>(USECS_PER_SEC) / out_rate;

    uint64 n = static_cast<uint64>(timestamp_usecs / delta_t_usecs);
    float64 last_resampling_time = n * delta_t_usecs;

    return static_cast<uint64>(last_resampling_time) + 
           static_cast<uint64>(sample.seconds()) * USECS_PER_SEC;
}

ADCSample interpolate(const ADCSample& earlier, const ADCSample& later, MARTe::uint64 t) {  
    ADCSample sample;
    if (t <= earlier.timestamp) {
        sample = earlier;
    } else if (t >= later.timestamp) {
        sample = later;
    } else {
        float64 t1 = earlier.timestamp, t2 = later.timestamp;
        
        float64 x1 = earlier.adc1_data, x2 = later.adc1_data;
        float64 m = (x2 - x1) / (t2 - t1);
        float64 x = x1 + m * (t - t1);
        sample.adc1_data = static_cast<uint16>(x);

        x1 = earlier.adc2_data, x2 = later.adc2_data;
        m = (x2 - x1) / (t2 - t1);
        x = x1 + m * (t - t1);
        sample.adc2_data = static_cast<uint16>(x);
        
        sample.timestamp = static_cast<uint64>(t);
        sample.valid = true;
    }

    return sample;
}

} // namespace MFI
