/**
 * @file InvertedPendulumGAMTest.h
 * @brief Header file for class InvertedPendulumGAMTest
 * @date 21/05/2024
 * @author Jawad Muhammad
 *.
 */

#define DLL_API

/*---------------------------------------------------------------------------*/
/*                         Standard header includes                          */
/*---------------------------------------------------------------------------*/
#include <limits.h>
#include "gtest/gtest.h"
/*---------------------------------------------------------------------------*/
/*                         Project header includes                           */
/*---------------------------------------------------------------------------*/

#include "InvertedPendulumGAM.h"
#include "InvertedPendulumGAMTest.h"
/*---------------------------------------------------------------------------*/
/*                           Static definitions                              */
/*---------------------------------------------------------------------------*/
using namespace MARTe;

TEST(PIDGAMGTest,TestInitialiseMissingKpKiKd) {
    PIDGAMTest test;
    ASSERT_TRUE(test.TestInitialiseMissingKpKiKd());
}

TEST(PIDGAMGTest,TestInitialiseMissingSampleTime) {
    PIDGAMTest test;
    ASSERT_TRUE(test.TestInitialiseMissingSampleTime());
}

TEST(PIDGAMGTest,TestInitialiseWrongSampleTime) {
    PIDGAMTest test;
    ASSERT_TRUE(test.TestInitialiseWrongSampleTime());
}


