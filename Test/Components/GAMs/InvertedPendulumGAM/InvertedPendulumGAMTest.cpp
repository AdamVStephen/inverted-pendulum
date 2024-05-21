/**
 * @file InvertedPendulumGAMTest.h
 * @brief Header file for class InvertedPendulumGAMTest
 * @date 21/05/2024
 * @author Jawad Muhammad
 *
 */

#define DLL_API

/*---------------------------------------------------------------------------*/
/*                         Standard header includes                          */
/*---------------------------------------------------------------------------*/
#include "stdio.h"
/*---------------------------------------------------------------------------*/
/*                         Project header includes                           */
/*---------------------------------------------------------------------------*/

#include "InvertedPendulumGAMTest.h"

/*---------------------------------------------------------------------------*/
/*                           Static definitions                              */
/*---------------------------------------------------------------------------*/

namespace {

}

/*---------------------------------------------------------------------------*/
/*                           Method definitions                              */
/*---------------------------------------------------------------------------*/

namespace MARTe {
class InvertedPendulumGAMTestHelper: public InvertedPendulumGAM {
public:
    CLASS_REGISTER_DECLARATION()InvertedPendulumGAMTestHelper(float64 kpDef = 1.0,
            float64 kiDef = 1.2,
            float64 kdDef = 1.3,
            float64 sampleTimeDef = 0.001,
            float64 maxOutputDef = 0x1.FFFFFFFFFFFFFp1023,
            float64 minOutputDef = -0x1.FFFFFFFFFFFFFp1023,
            uint32 noe = 1) {
        kp = kpDef;
        ki = kiDef;
        kd = kdDef;
        maxOutput = maxOutputDef;
        minOutput = minOutputDef;
        sampleTime = sampleTimeDef;
        numberOfElements = noe;
    }

    void *GetInputSignalsMemory() {
        return GAM::GetInputSignalsMemory();
    }
    void *GetOutputSignalsMemory() {
        return GAM::GetOutputSignalsMemory();
    }
    void *GetInputSignalsMemory(uint32 idx) {
        return GAM::GetInputSignalMemory(idx);
    }

    void *GetOutputSignalsMemory(uint32 idx) {
        return GAM::GetOutputSignalMemory(idx);
    }

    bool HelperInitialise() {
        bool ok;
        ok = config.Write("Kp", kp);
        ok = config.Write("Ki", ki);
        ok = config.Write("Kd", kd);
        ok &= config.Write("SampleTime", sampleTime);
        ok &= config.Write("MaxOutput", maxOutput);
        ok &= config.Write("MinOutput", minOutput);
        return ok;
    }
    bool HelperSetup1() {
        bool ok;
        uint32 numberOfInputSignals = 1;
        uint32 byteSizePerSignal = numberOfElements * sizeof(float64);
        uint32 totalInputBytes = byteSizePerSignal*numberOfInputSignals;
        uint32 totalOutputBytes = byteSizePerSignal;

        ok = configSignals.CreateAbsolute("Signals.InputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("NumberOfElements", 1);
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.Write("NumberOfDimensions", 1);
        ok &= configSignals.Write("Type", "float64");
        ok &= configSignals.Write("ByteSize", byteSizePerSignal);
        ok &= configSignals.MoveAbsolute("Signals.InputSignals");
        ok &= configSignals.Write("ByteSize", totalInputBytes);

        ok &= configSignals.MoveToRoot();
        ok &= configSignals.CreateAbsolute("Memory.InputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.CreateRelative("Signals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("Samples", 1);

        ok &= configSignals.MoveToRoot();
        ok &= configSignals.CreateAbsolute("Signals.OutputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("NumberOfElements", 1);
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.Write("NumberOfDimensions", 1);
        ok &= configSignals.Write("Type", "float64");
        ok &= configSignals.Write("ByteSize", byteSizePerSignal);
        ok &= configSignals.MoveAbsolute("Signals.OutputSignals");
        ok &= configSignals.Write("ByteSize", totalOutputBytes);

        ok &= configSignals.MoveToRoot();
        ok &= configSignals.CreateAbsolute("Memory.OutputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.CreateRelative("Signals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("Samples", 1);

        ok &= configSignals.MoveToRoot();
        return ok;
    }

    bool HelperSetup2() {
        bool ok;
        uint32 numberOfInputSignals = 2;
        uint32 byteSizePerSignal = numberOfElements * sizeof(float64);
        uint32 totalInputBytes = byteSizePerSignal*numberOfInputSignals;
        uint32 totalOutputBytes = byteSizePerSignal;

        ok = configSignals.CreateAbsolute("Signals.InputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("NumberOfElements", 1);
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.Write("NumberOfDimensions", 1);
        ok &= configSignals.Write("Type", "float64");
        ok &= configSignals.Write("ByteSize", byteSizePerSignal);
        ok &= configSignals.MoveAbsolute("Signals.InputSignals");
        ok &= configSignals.CreateRelative("1");
        ok &= configSignals.Write("NumberOfElements", 1);
        ok &= configSignals.Write("DataSource", "Measurement");
        ok &= configSignals.Write("NumberOfDimensions", 1);
        ok &= configSignals.Write("Type", "float64");
        ok &= configSignals.Write("ByteSize", byteSizePerSignal);
        ok &= configSignals.MoveToAncestor(1u);
        ok &= configSignals.Write("ByteSize", totalInputBytes);

        ok &= configSignals.MoveToRoot();
        ok &= configSignals.CreateAbsolute("Memory.InputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.CreateRelative("Signals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("Samples", 1);
        ok &= configSignals.MoveToAncestor(3u);
        ok &= configSignals.CreateRelative("1");
        ok &= configSignals.Write("DataSource", "Measurement");
        ok &= configSignals.CreateRelative("Signals");
        ok &= configSignals.CreateRelative("1");
        ok &= configSignals.Write("Samples", 1);

        ok &= configSignals.MoveToRoot();
        ok &= configSignals.CreateAbsolute("Signals.OutputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("NumberOfElements", 1);
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.Write("NumberOfDimensions", 1);
        ok &= configSignals.Write("Type", "float64");
        ok &= configSignals.Write("ByteSize", byteSizePerSignal);
        ok &= configSignals.MoveAbsolute("Signals.OutputSignals");
        ok &= configSignals.Write("ByteSize", totalOutputBytes);

        ok &= configSignals.MoveToRoot();
        ok &= configSignals.CreateAbsolute("Memory.OutputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.CreateRelative("Signals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("Samples", 1);

        ok &= configSignals.MoveToRoot();
        return ok;
    }

    bool IsEqualLargerMargins(float64 f1,
                              float64 f2) {
        float64 *min = reinterpret_cast<float64*>(const_cast<uint64*>(&EPSILON_FLOAT64));
        float64 minLarger = *min * 2;
        return ((f1 - f2) < (minLarger)) && ((f1 - f2) > -(minLarger));
    }

    ConfigurationDatabase config;
    ConfigurationDatabase configSignals;
private :
    float64 kp;
    float64 ki;
    float64 kd;
    float64 sampleTime;
    float64 maxOutput;
    float64 minOutput;
    uint32 numberOfElements;

};
CLASS_REGISTER(InvertedPendulumGAMTestHelper, "1.0")

InvertedPendulumGAMTest::InvertedPendulumGAMTest() {
//Auto-generated constructor stub for InvertedPendulumGAMTest

//TODO Verify if manual additions are needed here
}

InvertedPendulumGAMTest::~InvertedPendulumGAMTest() {
//Auto-generated destructor stub for InvertedPendulumGAMTest

//TODO Verify if manual additions are needed here
}

bool InvertedPendulumGAMTest::TestInitialiseMissingKpKiKd() {
    InvertedPendulumGAM gam;
    bool ret;
    ConfigurationDatabase config;
    ret = !gam.Initialise(config);
    return ret;
}

bool InvertedPendulumGAMTest::TestInitialiseMissingSampleTime() {
    InvertedPendulumGAM gam;
    bool ret;
    ConfigurationDatabase config;
    float64 kp = 1.1;
    config.Write("Kp", kp);
    ret = !gam.Initialise(config);
    return ret;
}




}

