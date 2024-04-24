#include "DataFrame.h"

using namespace MARTe;

namespace MFI {
namespace DataFrame {

const uint8 SYNC_BYTES[4] = {0x00, 0xAC, 0xDD, 0xDD};

RxDataFrame::RxDataFrame() : 
                        positionRotor(0u),
                        positionEncoder(0u),
                        Pwm1Counter(0u),
                        CYCCNT(0u)
                        // adc_time(0u),
                        // pps1_time(0u),
                        // pps2_time(0u),
                        // adc1_data(0u),
                        // adc2_data(0u) 
{
                        
}

uint32 SanitiseRxBuffer(SerialBuffer& buffer) {
    uint32 discarded = 0u;
    while (1) {
        uint32 nbytes_to_check = sizeof(SYNC_BYTES);
        if (nbytes_to_check > buffer.count()) {
            nbytes_to_check = buffer.count();
        }

        bool sanitised = true;
        for (uint32 i = 0; i < nbytes_to_check; i++) {
            uint8 value;
            buffer.at(i, value);
            if (value != SYNC_BYTES[i]) {
                if (i == 0) {
                    discarded += 1;
                    buffer.empty(1);
                } else {
                    discarded += i;
                    buffer.empty(i);
                }
                sanitised = false;                
                break;
            }
        }

        if (sanitised) {
            break;
        }
    }
    
    return discarded;
}

bool GetNextRxDataFrame(SerialBuffer& buffer, RxDataFrame& dataframe) {
    if (buffer.count() < RX_FRAME_SIZE) {
        return false;
    }

    uint8 temp[RX_FRAME_SIZE];
    buffer.dequeue(temp, RX_FRAME_SIZE);
    //ParseRxDataFramePayload(temp + sizeof(SYNC_BYTES), dataframe);
    ParseRxDataFramePayload(temp, dataframe);

    return true;
}

float bytesToFloat(uint8 b0, uint8 b1, uint8 b2, uint8 b3)
{
    float output;

    *((uint8*)(&output) + 3) = b0;
    *((uint8*)(&output) + 2) = b1;
    *((uint8*)(&output) + 1) = b2;
    *((uint8*)(&output) + 0) = b3;

    return output;
}

void ParseRxDataFramePayload(uint8* raw, RxDataFrame& dataframe) {  
    dataframe.positionRotor = static_cast<uint32>(raw[0]) +
                        (static_cast<uint32>(raw[1]) << 8) +
                        (static_cast<uint32>(raw[2]) << 16) +
                        (static_cast<uint32>(raw[3]) << 24);

    dataframe.positionEncoder = static_cast<uint32>(raw[4]) +
                         (static_cast<uint32>(raw[5]) << 8) +
                         (static_cast<uint32>(raw[6]) << 16) +
                         (static_cast<uint32>(raw[7]) << 24);

    dataframe.Pwm1Counter = static_cast<uint32>(raw[8]) +
                         (static_cast<uint32>(raw[9]) << 8) +
                         (static_cast<uint32>(raw[10]) << 16) +
                         (static_cast<uint32>(raw[11]) << 24);

    dataframe.CYCCNT =   static_cast<uint32>(raw[12]) +
                         (static_cast<uint32>(raw[13]) << 8)+
                         (static_cast<uint32>(raw[14]) << 16) +
                         (static_cast<uint32>(raw[15]) << 24);

    dataframe.OUTPUT_rotor_control_target_steps = bytesToFloat(raw[16], raw[17], raw[18], raw[19]);

    dataframe.OUTPUT_gpioState =   static_cast<uint8>(raw[20]);

    dataframe.OUTPUT_rotor_control_target_steps =   
                         static_cast<uint32>(raw[21]) +
                         (static_cast<uint32>(raw[22]) << 8)+
                         (static_cast<uint32>(raw[23]) << 16) +
                         (static_cast<uint32>(raw[24]) << 24);

}   

// void ParseRxDataFramePayloadOld(uint8* raw, RxDataFrame& dataframe) {  
//     dataframe.adc_time = static_cast<uint32>(raw[0]) +
//                         (static_cast<uint32>(raw[1]) << 8) +
//                         (static_cast<uint32>(raw[2]) << 16) +
//                         (static_cast<uint32>(raw[3]) << 24);

//     dataframe.pps1_time = static_cast<uint32>(raw[4]) +
//                          (static_cast<uint32>(raw[5]) << 8) +
//                          (static_cast<uint32>(raw[6]) << 16) +
//                          (static_cast<uint32>(raw[7]) << 24);

//     dataframe.pps2_time = static_cast<uint32>(raw[8]) +
//                          (static_cast<uint32>(raw[9]) << 8) +
//                          (static_cast<uint32>(raw[10]) << 16) +
//                          (static_cast<uint32>(raw[11]) << 24);

//     dataframe.adc1_data = static_cast<uint16>(raw[12]) +
//                          (static_cast<uint16>(raw[13]) << 8);

//     dataframe.adc2_data = static_cast<uint16>(raw[14]) +
//                          (static_cast<uint16>(raw[15]) << 8);
// }

} // namespace DataFrame
} // namespace MFI
