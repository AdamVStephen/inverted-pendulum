#include "DataFrame.h"

using namespace MARTe;

namespace MFI {
    namespace DataFrame {

        const uint8 SYNC_BYTES[4] = {0x00, 0xAC, 0xDD, 0xDD};

         RxDataFrame::RxDataFrame():
            positionRotor(0), 
            encoder_counter(0u), 
            Pwm1Counter(0u), 
            CYCCNT(0u), 
            OUTPUT_rotor_control_target_steps(0), 
            OUTPUT_L6474_Board_Pwm1Period(0u), 
            OUTPUT_gpioState(0u), 
            OUTPUT_break_Control_Loop(0u),
            OUTPUT_state(0u)
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


        /*
        MARTe::int32 positionRotor;
            MARTe::int32 positionEncoder;
            MARTe::uint32  Pwm1Counter;
            MARTe::uint32 CYCCNT;

            MARTe::int32 OUTPUT_rotor_control_target_steps;
            MARTe::uint32 OUTPUT_L6474_Board_Pwm1Period;
            MARTe::uint8 OUTPUT_gpioState;
        */
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

            dataframe.Pwm1Counter = static_cast<uint32>(raw[8]) +
                                 (static_cast<uint32>(raw[9]) << 8) +
                                 (static_cast<uint32>(raw[10]) << 16) +
                                 (static_cast<uint32>(raw[11]) << 24);

            dataframe.CYCCNT =   static_cast<uint32>(raw[12]) +
                                 (static_cast<uint32>(raw[13]) << 8)+
                                 (static_cast<uint32>(raw[14]) << 16) +
                                 (static_cast<uint32>(raw[15]) << 24);

            //dataframe.OUTPUT_rotor_control_target_steps = bytesToFloat(raw[16], raw[17], raw[18], raw[19]);
            dataframe.OUTPUT_rotor_control_target_steps =   
                                 static_cast<int32>(raw[16]) +
                                 (static_cast<int32>(raw[17]) << 8)+
                                 (static_cast<int32>(raw[18]) << 16) +
                                 (static_cast<int32>(raw[19]) << 24);

            dataframe.OUTPUT_L6474_Board_Pwm1Period =   
                                 static_cast<uint32>(raw[20]) +
                                 (static_cast<uint32>(raw[21]) << 8)+
                                 (static_cast<uint32>(raw[22]) << 16) +
                                 (static_cast<uint32>(raw[23]) << 24);
                                
            dataframe.OUTPUT_gpioState =   static_cast<uint8>(raw[24]);
            dataframe.OUTPUT_break_Control_Loop =   static_cast<uint8>(raw[25]);
            dataframe.OUTPUT_state =   static_cast<uint8>(raw[26]);

            //int ch  = 1;

        }   

        // void ParseRxDataFramePayload(uint8* raw, RxDataFrame& dataframe) {  
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
