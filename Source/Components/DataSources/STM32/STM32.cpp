#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <sched.h>
#include <pthread.h>

#include "STM32.h"
#include "STM32Reader.h"
#include "STM32Writer.h"
#include "SerialBuffer.h"
#include "DataFrame.h"

#include "DataSourceSignalChecker.h"
#include "AdvancedErrorManagement.h"
#include "MemoryMapSynchronisedInputBroker.h"
#include "MemoryMapSynchronisedOutputBroker.h"
#include "Threads.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <sys/ioctl.h>
#include <linux/serial.h>


using namespace MARTe;

namespace MFI {

STM32OutSignals::STM32OutSignals(): 
                          received_byte_count(0u),
                          discarded_byte_count(0u),
                          read_error_count(0u),
                          message_count(0u),
                          message_rx_time(0u),
                          message_tx_time(0u),
                          rx_buffer_occupancy(0u),
                          dataframe() {

}

STM32InSignals::STM32InSignals():
                        control_target_steps(0u),
                         Pwm1Period(0u),
                         gpioState(0u)
                        //   dac1_data(0u),
                        //   dac2_data(0u) 
{

}

STM32::STM32() : DataSourceI(), 
                 serial_fd(0),
                 port(""),
                 baud_rate(0),
                 rx_signals(),
                 tx_signals(),
                 rx_buffer(STM32_BUFSIZE) {

    struct sched_param sp;
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &sp);

    cpu_set_t useCPUs;
    CPU_ZERO(&useCPUs);
    CPU_SET(1, &useCPUs);

    if (pthread_setaffinity_np(pthread_self(), sizeof (useCPUs), &useCPUs)) {
        perror("main() pthread_setaffinity_np");
        exit(1);
    }
}

STM32::~STM32() {

}

bool STM32::Initialise(MARTe::StructuredDataI& data) {
    bool ok = DataSourceI::Initialise(data);

    if (ok) {
        ok = data.Read("Port", port);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "No serial port has been specified");
        }
    }

    if (ok) {
        ok = data.Read("BaudRate", baud_rate);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "No baud rate has been specified");
        }
    }

    // Open and configure the serial port
    if (ok) {
        serial_fd = open(port.Buffer(), O_RDWR);
        ok = !(serial_fd < 0);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Failed to open serial port");
        }

        struct termios tty;
        ok = (tcgetattr(serial_fd, &tty) == 0);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Failed to get serial port settings");
        }
        if (ok) {
            int baud_rate_code = B9600;
            switch (baud_rate) {
                case 9600: baud_rate_code = B9600; break;
                case 115200: baud_rate_code = B115200; break;
                case 230400: baud_rate_code = B230400; break;
                case 460800: baud_rate_code = B460800; break;
                case 921600: baud_rate_code = B921600; break;
                case 3000000: baud_rate_code = B3000000; break;
                default:
                    REPORT_ERROR(ErrorManagement::InitialisationError, "Baud rate %u not supported", baud_rate);
                    ok = false;
            }
            
            cfmakeraw(&tty);
            cfsetispeed(&tty, baud_rate_code);
            cfsetospeed(&tty, baud_rate_code);

            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;
            tty.c_cflag |= CLOCAL | CREAD;
            ok = (tcsetattr(serial_fd, TCSANOW, &tty) == 0);
            if (!ok) {
                REPORT_ERROR(ErrorManagement::InitialisationError, "Failed to set serial port settings");
            }
        }
      
        // Configure the serial port for low latency operation
        if (ok) {
            struct serial_struct srl;
            ioctl(serial_fd, TIOCGSERIAL, &srl);
            srl.flags |= ASYNC_LOW_LATENCY;
            ioctl(serial_fd, TIOCSSERIAL, &srl);
        }
    }
    
    return ok;
}

bool STM32::Synchronise() {   
    return true;
}

bool STM32::AllocateMemory() { 
    return true;
}

bool STM32::GetSignalMemoryBuffer(const uint32 signalIdx, const uint32 bufferIdx,
                                  void *&signalAddress) {
    signalAddress = NULL;
    
    if (signalIdx == 0u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.received_byte_count);
    } else if (signalIdx == 1u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.discarded_byte_count);
    } else if (signalIdx == 2u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.read_error_count);
    } else if (signalIdx == 3u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.message_count);
    } else if (signalIdx == 4u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.message_rx_time);
    } else if (signalIdx == 5u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.message_tx_time);
    } else if (signalIdx == 6u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.rx_buffer_occupancy);
    } 

//INPUT
    // positionRotor(0u),
    // positionEncoder(0u),
    // Pwm1Counter(0u),
    // CYCCNT(0u)
    else if (signalIdx == 7u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.positionRotor);
    } else if (signalIdx == 8u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.positionEncoder);
    } else if (signalIdx == 9u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.Pwm1Counter);
    } else if (signalIdx == 10u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.CYCCNT);
    } 
    //OUTPUT
    // control_target_steps,
    // gpioState,
    // Pwm1Period
    else if (signalIdx == 11u) {
        signalAddress = reinterpret_cast<void *>(&tx_signals.control_target_steps);
    } else if (signalIdx == 12u) {
        signalAddress = reinterpret_cast<void *>(&tx_signals.gpioState);
    } else if (signalIdx == 13u) {
        signalAddress = reinterpret_cast<void *>(&tx_signals.Pwm1Period);
    } 
    else if (signalIdx == 14u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.OUTPUT_rotor_control_target_steps);
    } else if (signalIdx == 15u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.OUTPUT_gpioState);
    } else if (signalIdx == 16u) {
        signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.OUTPUT_L6474_Board_Pwm1Period);
    } 
    // else if (signalIdx == 7u) {
    //     signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.adc_time);
    // } else if (signalIdx == 8u) {
    //     signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.pps1_time);
    // } else if (signalIdx == 9u) {
    //     signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.pps2_time);
    // } else if (signalIdx == 10u) {
    //     signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.adc1_data);
    // } else if (signalIdx == 11u) {
    //     signalAddress = reinterpret_cast<void *>(&rx_signals.dataframe.adc2_data);
    // } else if (signalIdx == 12u) {
    //     signalAddress = reinterpret_cast<void *>(&tx_signals.dac1_data);
    // } else if (signalIdx == 13u) {
    //     signalAddress = reinterpret_cast<void *>(&tx_signals.dac2_data);
    // } else {
    //     ;
    // }
    
    if (signalAddress == NULL) {
        return false;
    } else {
        return true;
    }
}

const MARTe::char8 *STM32::GetBrokerName(StructuredDataI &data,
                                         const SignalDirection direction) {
    if (direction == InputSignals) {
        return "STM32Reader";
    } else {
        return "STM32Writer";
    }
}

bool STM32::GetInputBrokers(ReferenceContainer &inputBrokers,
                            const char8* const functionName,
                            void * const gamMemPtr) {
    ReferenceT<STM32Reader> broker("STM32Reader");
    bool ret = broker.IsValid();
    if (ret) {
        ret = broker->Init(InputSignals, *this, functionName, gamMemPtr);
    }
    if (ret) {
        ret = inputBrokers.Insert(broker);
    }

    return ret;
}

bool STM32::GetOutputBrokers(ReferenceContainer &outputBrokers,
                             const char8* const functionName,
                             void * const gamMemPtr) {
    ReferenceT<STM32Writer> broker("STM32Writer");
    bool ret = broker.IsValid();
    if (ret) {
        ret = broker->Init(OutputSignals, *this, functionName, gamMemPtr);
    }
    if (ret) {
        ret = outputBrokers.Insert(broker);
    }

    return ret;
}

bool STM32::SetConfiguredDatabase(StructuredDataI & data) {
    bool ok = DataSourceI::SetConfiguredDatabase(data);
    
    if (ok) {
        ok = (GetNumberOfSignals() == 17u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Exactly 17 signals should be specified");
        }
    }
    
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 0u, UnsignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal ReceivedByteCount");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 1u, UnsignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal DiscardedByteCount");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 2u, UnsignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal ReadErrorCount");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 3u, UnsignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal MessageCount");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 4u, UnsignedInteger64Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal MessageRxTime");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 5u, UnsignedInteger64Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal MessageTxTime");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 6u, UnsignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal RxBufferOccupancy");
        }
    }


//INPUT
    // positionRotor(0u),
    // positionEncoder(0u),
    // Pwm1Counter(0u),
    // CYCCNT(0u)


    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 7u, SignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal positionRotor");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 8u, SignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal positionEncoder");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 9u, UnsignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal Pwm1Counter");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 10u, UnsignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal CYCCNT");
        }
    }

//OUTPUT
    // control_target_steps,
    // gpioState,
    // Pwm1Period
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 11u, SignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal control_target_steps");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 12u, UnsignedInteger8Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal gpioState");
        }
    }
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 13u, UnsignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal Pwm1Period");
        }
    }

    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 14u, SignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal OUTPUT_rotor_control_target_steps");
        }
    }
    
    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 15u, UnsignedInteger8Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal OUTPUT_gpioState");
        }
    }

    if (ok) {
        ok = DataSourceCheckSignalProperties(*this, 16u, UnsignedInteger32Bit, 0u, 1u);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal OUTPUT_L6474_Board_Pwm1Period");
        }
    }

    

    

    // if (ok) {
    //     ok = DataSourceCheckSignalProperties(*this, 7u, UnsignedInteger32Bit, 0u, 1u);
    //     if (!ok) {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal ADCTime");
    //     }
    // }
    // if (ok) {
    //     ok = DataSourceCheckSignalProperties(*this, 8u, UnsignedInteger32Bit, 0u, 1u);
    //     if (!ok) {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal PPS1Time");
    //     }
    // }
    // if (ok) {
    //     ok = DataSourceCheckSignalProperties(*this, 9u, UnsignedInteger32Bit, 0u, 1u);
    //     if (!ok) {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal PPS2Time");
    //     }
    // }
    // if (ok) {
    //     ok = DataSourceCheckSignalProperties(*this, 10u, UnsignedInteger16Bit, 0u, 1u);
    //     if (!ok) {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal ADC1Data");
    //     }
    // }
    // if (ok) {
    //     ok = DataSourceCheckSignalProperties(*this, 11u, UnsignedInteger16Bit, 0u, 1u);
    //     if (!ok) {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal ADC2Data");
    //     }
    // }
    // if (ok) {
    //     ok = DataSourceCheckSignalProperties(*this, 12u, UnsignedInteger16Bit, 0u, 1u);
    //     if (!ok) {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal DAC1Data");
    //     }
    // }
    // if (ok) {
    //     ok = DataSourceCheckSignalProperties(*this, 13u, UnsignedInteger16Bit, 0u, 1u);
    //     if (!ok) {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for signal DAC2Data");
    //     }
    // }

    return ok;
}

bool STM32::PrepareNextState(const char8 * const currentStateName,
                                    const char8 * const nextStateName) {
    return true;
}

bool STM32::RxSynchronise() {
    static uint32 counter = 0;
    
    // Hacky solution to deal with the fact that there may be a large amount of data in the serial 
    // port soft buffer when the application starts running
    if (counter == 0) {
        tcflush(serial_fd, TCIOFLUSH);
    }
    
    while (1) {           
        uint8 temp_rx_buffer[STM32_BUFSIZE];
        
        // Blocking read
        int ret = read(serial_fd, temp_rx_buffer, sizeof(temp_rx_buffer));

        if (ret < 0) {
            // To do: fix problem where the transfer of the updated rx_signals is skipped by this continue
            rx_signals.read_error_count++;
            continue;
        }
        
        uint32 nbytes = static_cast<uint32>(ret);
        rx_signals.received_byte_count += nbytes;
        
        uint32 nbytes_queued = rx_buffer.queue(temp_rx_buffer, nbytes);
        rx_signals.discarded_byte_count += (nbytes - nbytes_queued);

        // The rx_buffer could contain at most STM32_BUFSIZE / DataFrame::RX_FRAME_SIZE messages for 
        // processing. In principle there should be no more than one message available, since we 
        // process received bytes immediately. However, if this thread were starved of CPU time, 
        // this would not be the case, and we would need to process multiple buffered messages to
        // catch up - hence this loop
        bool new_frames_received = false;
        for (uint32 frame = 0; frame < STM32_BUFSIZE / DataFrame::RX_FRAME_SIZE; frame++) {
            // Sanitise the rx_buffer - delete any leading rogue bytes that cannot be part of an STM32
            // Rx data frame
           // rx_signals.discarded_byte_count += DataFrame::SanitiseRxBuffer(rx_buffer);

            DataFrame::RxDataFrame dataframe;
            if (DataFrame::GetNextRxDataFrame(rx_buffer, rx_signals.dataframe)) {
                rx_signals.message_count++;
                rx_signals.message_rx_time = HighResolutionTimer::Counter();
                new_frames_received = true;
            } else {
                break;
            }
        }

        if (new_frames_received) {
            rx_signals.rx_buffer_occupancy = rx_buffer.count();
            counter++;
            return true;
        }
    }
}

bool STM32::TxSynchronise() {
    //uint16 tx_buffer;
    
    //tx_buffer = tx_signals.control_target_steps;
    tx_signals.control_target_steps=1;
    tx_signals.gpioState=1;
    tx_signals.Pwm1Period=1;
    int ret = write(serial_fd, &tx_signals, sizeof(tx_signals));

    rx_signals.message_tx_time = HighResolutionTimer::Counter();

    return true;
}

CLASS_REGISTER(STM32, "1.0");

} // namespace MFI
