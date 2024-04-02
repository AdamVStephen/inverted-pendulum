#ifndef MFI_INVERSION_GAM_H_
#define MFI_INVERSION_GAM_H_

#include "GAM.h"

namespace MFI {

/**
 * @brief A GAM for applying an inversion operation to ADC input signals
 * 
 * @details The GAM shall have the following configuration (the object name `InversionGAM` is an 
 * example - it is arbitrary):
 * <pre>
 * +InversionGAM = {
 *     Class = InversionGAM
 *     InputSignals = {
 *         ADC1Data = {
 *             DataSource = DDB
 *             Alias = TimestampedADC1Data
 *             Type = uint16
 *         }
 *         ADC2Data = {
 *             DataSource = DDB
 *             Alias = TimestampedADC2Data
 *             Type = uint16
 *         }
 *    }
 *    OutputSignals = {
 *         DAC1Data = {
 *             DataSource = STM32
 *             Type = uint16
 *         }
 *         DAC2Data = {
 *             DataSource = STM32
 *             Type = uint16
 *         }
 *    }
 * }
 * </pre>
 * 
 * The inversion is performed by simply subtracting the ADC signal from the maximum 12-bit value 
 * of 4095
 */
class InversionGAM : public MARTe::GAM {
 public:
    CLASS_REGISTER_DECLARATION();

    InversionGAM();
    virtual ~InversionGAM();

    virtual bool Initialise(MARTe::StructuredDataI & data);

    virtual bool Setup();

    virtual bool Execute();

 private:

    MARTe::uint16* in_adc1_data;

    MARTe::uint16* in_adc2_data;

    MARTe::uint16* out_dac1_data;

    MARTe::uint16* out_dac2_data;
};

} // namespace MFI

#endif // MFI_INVERSION_GAM_H_
