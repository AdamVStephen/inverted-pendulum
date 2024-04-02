#include "InversionGAM.h"
#include "AdvancedErrorManagement.h"
#include "GAMSignalChecker.h"

using namespace MARTe;

namespace MFI {

InversionGAM::InversionGAM() {
    in_adc1_data = NULL_PTR(uint16*);
    in_adc2_data = NULL_PTR(uint16*);
    out_dac1_data = NULL_PTR(uint16*);
    out_dac2_data = NULL_PTR(uint16*);
}

InversionGAM::~InversionGAM() {

}

bool InversionGAM::Initialise(StructuredDataI & data) {
    return GAM::Initialise(data);
}

bool InversionGAM::Setup() {
    StreamString gam_name;
    
    bool ok = GetQualifiedName(gam_name);
    if (!ok) {
        REPORT_ERROR(ErrorManagement::ParametersError, "Cannot get the qualified name");
    }

    if (ok) {
        uint32 nOfInputSignals = GetNumberOfInputSignals();
        ok = (nOfInputSignals == 2u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "%s::Number of input signals must be 2", gam_name.Buffer());
        }
    }
    uint32 signalIdx;
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADC1Data", InputSignals, UnsignedInteger16Bit, 0u, 1u, signalIdx);
        if (ok) {
            in_adc1_data = (uint16*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal ADC1Data");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADC2Data", InputSignals, UnsignedInteger16Bit, 0u, 1u, signalIdx);
        if (ok) {
            in_adc2_data = (uint16*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal ADC2Data");
        }
    }

    if (ok) {
        uint32 nOfOutputSignals = GetNumberOfOutputSignals();
        ok = (nOfOutputSignals == 2u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "%s::Number of output signals must be 2", gam_name.Buffer());
        }
    }

    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "DAC1Data", OutputSignals, UnsignedInteger16Bit, 0u, 1u, signalIdx);
        if (ok) {
            out_dac1_data = (uint16*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal DAC1Data");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "DAC2Data", OutputSignals, UnsignedInteger16Bit, 0u, 1u, signalIdx);
        if (ok) {
            out_dac2_data = (uint16*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal DAC2Data");
        }
    }

    return ok;
}

bool InversionGAM::Execute() {
    *out_dac1_data = 0x0FFF - *in_adc1_data;
    *out_dac2_data = 0x0FFF - *in_adc2_data;

    return true;
}

CLASS_REGISTER(InversionGAM, "1.0");

} // namespace MFI
