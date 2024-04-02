#ifndef MFI_DOWNSAMPLING_GAM_H_
#define MFI_DOWNSAMPLING_GAM_H_

#include "GAM.h"
#include "ADCSample.h"

namespace MFI {

/**
 * @brief A GAM for synchronous downsampling of an input data stream
 * 
 * @details The GAM downsamples the time-stamped input data stream to a lower rate, defined by its
 * configuration
 * 
 * The GAM shall have the following configuration (the object name `DownsamplingGAM` is an example - 
 * it is arbitrary):
 * <pre>
 * +DownsamplingGAM = {
 *     Class = DownsamplingGAM
 *     DownsampledDataRate = 6400
 *     InputSignals = {
 *         ADC1Data = {
 *             DataSource = DDB
 *             Type = uint16
 *         }
 *         ADC2Data = {
 *             DataSource = DDB
 *             Type = uint16
 *         }
 *         ADCUTCUnixSeconds = {
 *             DataSource = DDB
 *             Type = uint32
 *         }
 *         ADCUTCMicroseconds = {
 *             DataSource = DDB
 *             Type = uint32
 *         }
 *         Validity = {
 *             DataSource = DDB
 *             Type = uint8
 *         }
 *    }
 *    OutputSignals = {
 *         Count = {
 *             DataSource = DDB
 *             Type = uint32
 *         }
 *         ADC1Data = {
 *             DataSource = DDB
 *             Type = uint16
 *         }
 *         ADC2Data = {
 *             DataSource = DDB
 *             Type = uint16
 *         }
 *         ADCUTCUnixSeconds = {
 *             DataSource = DDB
 *             Type = uint32
 *         }
 *         ADCUTCMicroseconds = {
 *             DataSource = DDB
 *             Type = uint32
 *         }
 *         Validity = {
 *             DataSource = DDB
 *             Type = uint8
 *         }
 *    }
 * }
 * </pre>
 * 
 * The downsampling is performed aligned to the UTC zero-second rollover. The configuration of the 
 * GAM is:
 * 
 * DownsampledDataRate - The data rate of the downsampled data stream, in samples per second
 * 
 * The input signals are:
 * 
 * ADC1Data - The value of ADC 1 at the original data rate
 * ADC2Data - The value of ADC2 at the original data rate
 * ADCUTCUnixSeconds - The seconds portion of the original data's timestamp
 * ADCUTCMicroseconds - The microseconds portion of the original data's timestamp
 * Validity - The validity of the original data
 * 
 * The output signals are:
 * 
 * Count - The number of downsampled data points generated. Always increments by 1 when Validity is 1
 * ADC1Data - The downsampled value of ADC 1
 * ADC2Data - The dowsnsampled value of ADC 2
 * ADCUTCUnixSeconds - The seconds portion of the downsampled data's timestamp
 * ADCUTCMicroseconds - The microseconds portion of the downsampled data's timestamp
 * Validity - The validity of the downsampled data: 1 is valid, 0 is invalid
 */
class DownsamplingGAM : public MARTe::GAM {
 public:
    CLASS_REGISTER_DECLARATION();
    
    DownsamplingGAM();
    virtual ~DownsamplingGAM();

    virtual bool Initialise(MARTe::StructuredDataI & data);

    virtual bool Setup();

    virtual bool Execute();    

 private:
   
    ADCSample prev_sample;

    MARTe::uint32 count;
    
    MARTe::uint32 input_data_rate;

    MARTe::uint32 output_data_rate;
    
    MARTe::uint16* in_adc1_data;

    MARTe::uint16* in_adc2_data;

    MARTe::uint32* in_adc_time_seconds;

    MARTe::uint32* in_adc_time_microseconds;

    MARTe::uint8* in_validity;

    MARTe::uint32* out_count;
    
    MARTe::uint16* out_adc1_data;

    MARTe::uint16* out_adc2_data;

    MARTe::uint32* out_adc_time_seconds;

    MARTe::uint32* out_adc_time_microseconds;

    MARTe::uint8* out_validity;
};

} // namespace MFI

#endif // MFI_DOWNSAMPLING_GAM_H_
