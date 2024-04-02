#ifndef MFI_INTERPOLATE_H_
#define MFI_INTERPOLATE_H_

#include "ADCSample.h"

namespace MFI {

/**
 * @brief Get the most recent resampling time preceding the given sample
 */
MARTe::uint64 get_last_resampling_time(const ADCSample& sample, MARTe::uint32 out_rate);

/**
 * @brief Interpolate between two ADC samples
 */
ADCSample interpolate(const ADCSample& earlier, const ADCSample& later, MARTe::uint64 t);

} // namespace MFI

#endif // MFI_INTERPOLATE_H_

