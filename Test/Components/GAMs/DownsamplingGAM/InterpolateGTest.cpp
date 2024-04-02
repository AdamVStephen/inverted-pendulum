#include "Interpolate.h"

#include "gtest/gtest.h"

#include <utility>

using namespace MARTe;
using namespace MFI;

class InterpolatingSamplesSlope1 : public ::testing::TestWithParam<uint64> {
 public:
    InterpolatingSamplesSlope1() {
        earlier.adc1_data = earlier.adc2_data = 0u;
        earlier.timestamp = 0u;

        later.adc1_data = later.adc2_data = 10u;
        later.timestamp = 10u;
    }

 protected:
    ADCSample earlier, later;
};

TEST_P(InterpolatingSamplesSlope1, GivesTheExpectedValueForADC1) {
    ADCSample interpolated = interpolate(earlier, later, GetParam());

    ASSERT_EQ(interpolated.adc1_data, GetParam());
} 

TEST_P(InterpolatingSamplesSlope1, GivesTheExpectedValueForADC2) {
    ADCSample interpolated = interpolate(earlier, later, GetParam());

    ASSERT_EQ(interpolated.adc2_data, GetParam());
}

INSTANTIATE_TEST_CASE_P(RangeOfInterpolationTimes, 
                        InterpolatingSamplesSlope1,
                        ::testing::Values(0u,
                                          1u,
                                          3u,
                                          7u,
                                          10u));

class InterpolatingOutsideSampleRange : public ::testing::Test {
 public:
    InterpolatingOutsideSampleRange() {
        earlier.adc1_data = 31415;
        earlier.adc2_data = 1729;
        earlier.timestamp = 10u;

        later.adc1_data = 0xABCD;
        later.adc2_data = 0xFFFF;
        later.timestamp = 20u;
    }

 protected:
    ADCSample earlier, later;
};

TEST_F(InterpolatingOutsideSampleRange, ReturnsEarlierSample) {
   ADCSample interpolated = interpolate(earlier, later, 9u);

   ASSERT_TRUE(interpolated.timestamp == earlier.timestamp &&
               interpolated.adc1_data == earlier.adc1_data &&
               interpolated.adc2_data == earlier.adc2_data);
}

TEST_F(InterpolatingOutsideSampleRange, ReturnsLaterSample) {
   ADCSample interpolated = interpolate(earlier, later, 21u);

   ASSERT_TRUE(interpolated.timestamp == later.timestamp &&
               interpolated.adc1_data == later.adc1_data &&
               interpolated.adc2_data == later.adc2_data);
}

typedef std::pair<uint64, uint16> time_and_value;

class InterpolatingSamplesSlopeLessThan1 : public ::testing::TestWithParam<time_and_value> {
 public:
    InterpolatingSamplesSlopeLessThan1() {
        earlier.adc1_data = earlier.adc2_data = 0u;
        earlier.timestamp = 5u;

        later.adc1_data = later.adc2_data = 5u;
        later.timestamp = 15u;
    }

 protected:
    ADCSample earlier, later;
};

TEST_P(InterpolatingSamplesSlopeLessThan1, GivesTheExpectedValueForADC1) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc1_data);
} 

TEST_P(InterpolatingSamplesSlopeLessThan1, GivesTheExpectedValueForADC2) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc2_data);
}

INSTANTIATE_TEST_CASE_P(RangeOfInterpolationTimes, 
                        InterpolatingSamplesSlopeLessThan1,
                        ::testing::Values(time_and_value(5u, 0u),
                                          time_and_value(6u, 0u),
                                          time_and_value(7u, 1u),
                                          time_and_value(8u, 1u),
                                          time_and_value(11u, 3u),
                                          time_and_value(12u, 3u),
                                          time_and_value(15u, 5u)));

class InterpolatingSamplesSlopeGreaterThan1 : public ::testing::TestWithParam<time_and_value> {
 public:
    InterpolatingSamplesSlopeGreaterThan1() {
        earlier.adc1_data = earlier.adc2_data = 0u;
        earlier.timestamp = 0u;

        later.adc1_data = later.adc2_data = 50u;
        later.timestamp = 10u;
    }

 protected:
    ADCSample earlier, later;
};

TEST_P(InterpolatingSamplesSlopeGreaterThan1, GivesTheExpectedValueForADC1) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc1_data);
} 

TEST_P(InterpolatingSamplesSlopeGreaterThan1, GivesTheExpectedValueForADC2) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc2_data);
} 

INSTANTIATE_TEST_CASE_P(RangeOfInterpolationTimes, 
                        InterpolatingSamplesSlopeGreaterThan1,
                        ::testing::Values(time_and_value(0u, 0u),
                                          time_and_value(1u, 5u),
                                          time_and_value(2u, 10u),
                                          time_and_value(3u, 15u),
                                          time_and_value(6u, 30u),
                                          time_and_value(7u, 35u),
                                          time_and_value(10u, 50u)));

class InterpolatingSamplesSlopeZero : public ::testing::TestWithParam<time_and_value> {
 public:
    InterpolatingSamplesSlopeZero() {
        earlier.adc1_data = earlier.adc2_data = 5u;
        earlier.timestamp = 1u;

        later.adc1_data = later.adc2_data = 5u;
        later.timestamp = 11u;
    }

 protected:
    ADCSample earlier, later;
};

TEST_P(InterpolatingSamplesSlopeZero, GivesTheExpectedValueForADC1) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc1_data);
} 

TEST_P(InterpolatingSamplesSlopeZero, GivesTheExpectedValueForADC2) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc2_data);
} 

INSTANTIATE_TEST_CASE_P(RangeOfInterpolationTimes, 
                        InterpolatingSamplesSlopeZero,
                        ::testing::Values(time_and_value(1u, 5u),
                                          time_and_value(2u, 5u),
                                          time_and_value(3u, 5u),
                                          time_and_value(4u, 5u),
                                          time_and_value(7u, 5u),
                                          time_and_value(8u, 5u),
                                          time_and_value(11u, 5u)));

class InterpolatingSamplesSlopeNegative : public ::testing::TestWithParam<time_and_value> {
 public:
    InterpolatingSamplesSlopeNegative() {
        earlier.adc1_data = earlier.adc2_data = 10u;
        earlier.timestamp = 1u;

        later.adc1_data = later.adc2_data = 0u;
        later.timestamp = 11u;
    }

 protected:
    ADCSample earlier, later;
};

TEST_P(InterpolatingSamplesSlopeNegative, GivesTheExpectedValueForADC1) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc1_data);
} 

TEST_P(InterpolatingSamplesSlopeNegative, GivesTheExpectedValueForADC2) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc2_data);
}

INSTANTIATE_TEST_CASE_P(RangeOfInterpolationTimes, 
                        InterpolatingSamplesSlopeNegative,
                        ::testing::Values(time_and_value(1u, 10u),
                                          time_and_value(2u, 9u),
                                          time_and_value(3u, 8u),
                                          time_and_value(4u, 7u),
                                          time_and_value(7u, 4u),
                                          time_and_value(8u, 3u),
                                          time_and_value(10u, 1u)));

class InterpolatingSamplesAcrossSecondBoundary : public ::testing::TestWithParam<time_and_value> {
 public:
    InterpolatingSamplesAcrossSecondBoundary() {
        earlier.adc1_data = earlier.adc2_data = 0u;
        earlier.timestamp = 1000000u - 5u;

        later.adc1_data = later.adc2_data = 10u;
        later.timestamp = 1000000u + 4u;
    }

 protected:
    ADCSample earlier, later;
};

TEST_P(InterpolatingSamplesAcrossSecondBoundary, GivesTheExpectedValueForADC1) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc1_data);
} 

TEST_P(InterpolatingSamplesAcrossSecondBoundary, GivesTheExpectedValueForADC2) {
    uint64 t = GetParam().first;
    uint16 expected_value = GetParam().second;

    ADCSample interpolated = interpolate(earlier, later, t);

    ASSERT_EQ(expected_value, interpolated.adc2_data);
}

INSTANTIATE_TEST_CASE_P(RangeOfInterpolationTimes, 
                        InterpolatingSamplesAcrossSecondBoundary,
                        ::testing::Values(time_and_value(1000000u - 5u, 0u),
                                          time_and_value(1000000u - 4u, 1u),
                                          time_and_value(1000000u - 0u, 5u),
                                          time_and_value(1000000u + 1u, 6u),
                                          time_and_value(1000000u + 3u, 8u),
                                          time_and_value(1000000u + 4u, 10u)));

struct last_resample_time_test_data {
    last_resample_time_test_data(uint32 out_data_rate_,
                                 uint64 sample_timestamp_,
                                 uint64 expected_resample_time_) :
                                 out_data_rate(out_data_rate_),
                                 sample_timestamp(sample_timestamp_),
                                 expected_resample_time(expected_resample_time_) {}
    uint32 out_data_rate;
    uint64 sample_timestamp;
    uint64 expected_resample_time;
};

class GetLastResamplingTime : public ::testing::TestWithParam<last_resample_time_test_data> {
 public:
    GetLastResamplingTime() : sample() {
        sample.valid = true;
        sample.adc1_data = 0u;
        sample.adc2_data = 0u;

        out_data_rate = GetParam().out_data_rate;
        sample.timestamp = GetParam().sample_timestamp;
        expected_resample_time = GetParam().expected_resample_time;
    }

 protected:
    uint32 in_data_rate;
    uint32 out_data_rate;
    uint64 expected_resample_time;
    ADCSample sample;
};

TEST_P(GetLastResamplingTime, ReturnsExpectedValue) {
    uint64 actual_resample_time = get_last_resampling_time(sample, out_data_rate);

    ASSERT_EQ(expected_resample_time, actual_resample_time);
}

INSTANTIATE_TEST_CASE_P(RangeOfResamplingConfigurations,
                        GetLastResamplingTime,
                        ::testing::Values(last_resample_time_test_data(6400u, 0u, 0u),
                                          last_resample_time_test_data(6400u, 1u, 0u),
                                          last_resample_time_test_data(6400u, 156u, 0u),
                                          last_resample_time_test_data(6400u, 157u, 156u),
                                          last_resample_time_test_data(6400u, 312u, 156u),
                                          last_resample_time_test_data(6400u, 313u, 312u),
                                          last_resample_time_test_data(6400u, 999843u, 999687u),
                                          last_resample_time_test_data(6400u, 999844u, 999843u),
                                          last_resample_time_test_data(6400u, 1000000 + 0u, 1000000 + 0u),
                                          last_resample_time_test_data(6400u, 1000000 + 1u, 1000000 + 0u),
                                          last_resample_time_test_data(6400u, 1000000 + 156u, 1000000 + 0u),
                                          last_resample_time_test_data(6400u, 1000000 + 157u, 1000000 + 156u),
                                          last_resample_time_test_data(6400u, 1000000 + 312u, 1000000 + 156u),
                                          last_resample_time_test_data(6400u, 1000000 + 313u, 1000000 + 312u),
                                          last_resample_time_test_data(6400u, 1000000 + 999843u, 1000000 + 999687u),
                                          last_resample_time_test_data(6400u, 1000000 + 999844u, 1000000 + 999843u)));
