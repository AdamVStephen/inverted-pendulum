#include "DataFrame.h"

using namespace MARTe;

namespace MFI {
    namespace DataFrame {

        const uint8 SYNC_BYTES[4] = {0x00, 0xAC, 0xDD, 0xDD};

         RxDataFrame::RxDataFrame():
            positionRotor(0), 
            encoder_counter(0u)
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

            uint8 temp[RX_FRAME_SIZE] = {'\0'};
            buffer.dequeue(temp, RX_FRAME_SIZE);
            ParseRxDataFramePayload(temp + sizeof(SYNC_BYTES), dataframe);
            //ParseRxDataFramePayload(temp, dataframe);

            // Data d = (Data)temp;
            
            // dataframe.positionRotor = d->positionRotor; 
            // dataframe.encoder_counter =  d->encoder_counter;
            // dataframe.Pwm1Counter =  d->Pwm1Counter;
            // dataframe.CYCCNT =  d->CYCCNT;
            // dataframe.OUTPUT_rotor_control_target_steps =  d->OUTPUT_rotor_control_target_steps;
            // dataframe.OUTPUT_L6474_Board_Pwm1Period =  d->OUTPUT_L6474_Board_Pwm1Period;
            // dataframe.OUTPUT_gpioState =  d->OUTPUT_gpioState;
            // dataframe.OUTPUT_break_Control_Loop = d->OUTPUT_break_Control_Loop;
            // dataframe.OUTPUT_state = d->OUTPUT_state;

            return true;
        }

        // float bytesToFloat(uint8 b0, uint8 b1, uint8 b2, uint8 b3)
        // {
        //     float output;

        //     *((uint8*)(&output) + 3) = b0;
        //     *((uint8*)(&output) + 2) = b1;
        //     *((uint8*)(&output) + 1) = b2;
        //     *((uint8*)(&output) + 0) = b3;

        //     return output;
        // }

        void ParseRxDataFramePayload(uint8* raw, RxDataFrame& dataframe) {  


            dataframe.positionRotor = static_cast<int32>(raw[0]) +
                                (static_cast<int32>(raw[1]) << 8) +
                                (static_cast<int32>(raw[2]) << 16) +
                                (static_cast<int32>(raw[3]) << 24);

            //dataframe.positionEncoder = bytesToFloat(raw[4], raw[5], raw[6], raw[7]);
            dataframe.encoder_counter = static_cast<uint32>(raw[4]) +
                                 (static_cast<uint32>(raw[5]) << 8) +
                                 (static_cast<uint32>(raw[6]) << 16) +
                                 (static_cast<uint32>(raw[7]) << 24);

           
        }   
    } // namespace DataFrame
} // namespace MFI
