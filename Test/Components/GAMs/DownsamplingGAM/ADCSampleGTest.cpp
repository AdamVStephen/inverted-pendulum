#include "ADCSample.h"
#include "CompilerTypes.h"

#include "gtest/gtest.h"
#include <limits>

using namespace MARTe;
using namespace MFI;

class ADCSampleSeconds : public ::testing::TestWithParam<uint32> {
 public:
    ADCSampleSeconds() :
        sample(0u, 0u, GetParam(), 0u) {}
 protected:
    ADCSample sample;   
};

TEST_P(ADCSampleSeconds, ReturnsExpectedValue) {
    ASSERT_EQ(GetParam(), sample.seconds());
}

INSTANTIATE_TEST_CASE_P(RangeOfSecondsValues,
                        ADCSampleSeconds,
                        ::testing::Values(0u, 
                                          100u,
                                          std::numeric_limits<uint32>::max() >> 1,
                                          std::numeric_limits<uint32>::max()));

class ADCSampleMicroseconds : public ::testing::TestWithParam<uint32> {
 public:
    ADCSampleMicroseconds() :
        sample(0u, 0u, 0u, GetParam()) {}
 protected:
    ADCSample sample;   
};

TEST_P(ADCSampleMicroseconds, ReturnsExpectedValue) {
    ASSERT_EQ(GetParam(), sample.microseconds());
}

INSTANTIATE_TEST_CASE_P(RangeOfMicrosecondsValues,
                        ADCSampleMicroseconds,
                        ::testing::Values(0u, 
                                          100u,
                                          999999u));