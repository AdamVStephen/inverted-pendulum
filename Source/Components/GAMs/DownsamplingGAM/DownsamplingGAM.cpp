#include "DownsamplingGAM.h"
#include "Interpolate.h"
#include "AdvancedErrorManagement.h"
#include "GAMSignalChecker.h"

using namespace MARTe;

namespace MFI {

DownsamplingGAM::DownsamplingGAM() : GAM(), prev_sample() {
    count = 0u;
    input_data_rate = 0u;
    output_data_rate = 0u;
    in_adc1_data = NULL_PTR(uint16*);
    in_adc2_data = NULL_PTR(uint16*);
    in_adc_time_seconds = NULL_PTR(uint32*);
    in_adc_time_microseconds = NULL_PTR(uint32*);
    in_validity = NULL_PTR(uint8*);
    out_count = NULL_PTR(uint32*);
    out_adc1_data = NULL_PTR(uint16*);
    out_adc2_data = NULL_PTR(uint16*);
    out_adc_time_seconds = NULL_PTR(uint32*);
    out_adc_time_microseconds = NULL_PTR(uint32*);
    out_validity = NULL_PTR(uint8*);
}

DownsamplingGAM::~DownsamplingGAM() {

}

bool DownsamplingGAM::Initialise(MARTe::StructuredDataI & data) {
    bool ok = GAM::Initialise(data);

    if (ok) {
        ok = data.Read("InputDataRate", input_data_rate);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "No input data rate has been specified");
        }
    }
    
    if (ok) {
        ok = data.Read("OutputDataRate", output_data_rate);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "No output data rate has been specified");
        }
    }
    
    return ok;
}

bool DownsamplingGAM::Setup() {
    StreamString gam_name;
    
    bool ok = GetQualifiedName(gam_name);
    if (!ok) {
        REPORT_ERROR(ErrorManagement::ParametersError, "Cannot get the qualified name");
    }

    if (ok) {
        uint32 nOfInputSignals = GetNumberOfInputSignals();
        ok = (nOfInputSignals == 5u); // Will need to be changed if any input signals are added or removed
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "%s::Number of input signals must be 5", gam_name.Buffer());
        }
    } 
    uint32 signalIdx;
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADC1Data", InputSignals, UnsignedInteger16Bit, 0u, 1u, signalIdx);
        if (ok) {
            in_adc1_data = (uint16*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal ADCTime");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADC2Data", InputSignals, UnsignedInteger16Bit, 0u, 1u, signalIdx);
        if (ok) {
            in_adc2_data = (uint16*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal ADCTime");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADCUTCUnixSeconds", InputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            in_adc_time_seconds = (uint32*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal ADCUTCUnixSeconds");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADCUTCMicroseconds", InputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            in_adc_time_microseconds = (uint32*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal ADCUTCMicroseconds");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "Validity", InputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
        if (ok) {
            in_validity = (uint8*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal Validity");
        }
    }

    if (ok) {
        uint32 nOfOutputSignals = GetNumberOfOutputSignals();
        ok = (nOfOutputSignals == 6u); // Will need to be changed if any output signals are added or removed
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "%s::Number of output signals must be 6", gam_name.Buffer());
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "Count", OutputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            out_count = (uint32*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal Count");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADC1Data", OutputSignals, UnsignedInteger16Bit, 0u, 1u, signalIdx);
        if (ok) {
            out_adc1_data = (uint16*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal ADC1Data");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADC2Data", OutputSignals, UnsignedInteger16Bit, 0u, 1u, signalIdx);
        if (ok) {
            out_adc2_data = (uint16*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal ADC2Data");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADCUTCUnixSeconds", OutputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            out_adc_time_seconds = (uint32*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal ADCUTCUnixSeconds");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "ADCUTCMicroseconds", OutputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            out_adc_time_microseconds = (uint32*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal ADCUTCMicroseconds");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "Validity", OutputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
        if (ok) {
            out_validity = (uint8*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal Validity");
        }
    }

    return ok;
}

bool DownsamplingGAM::Execute() {
    *out_validity = 0u;
    
    if (*in_validity == 1u) {
        ADCSample sample(*in_adc1_data, *in_adc2_data, 
                         *in_adc_time_seconds, *in_adc_time_microseconds);

        uint64 t = get_last_resampling_time(sample, output_data_rate);
        if (prev_sample.valid && time_is_between(t, prev_sample, sample)) {
            ADCSample interpolated_sample = interpolate(prev_sample, sample, t);

            count++;
            *out_adc1_data = interpolated_sample.adc1_data;
            *out_adc2_data = interpolated_sample.adc2_data;
            *out_adc_time_seconds = interpolated_sample.seconds();
            *out_adc_time_microseconds = interpolated_sample.microseconds();
            *out_validity = 1u;
            *out_count = count;
        }       
        
        prev_sample = sample;
    }

    return true;
}

CLASS_REGISTER(DownsamplingGAM, "1.0");

} // namespace MFI
