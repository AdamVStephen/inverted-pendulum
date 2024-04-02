#include "SerialBuffer.h"
#include "CompilerTypes.h"

#include "gtest/gtest.h"

using namespace MFI;
using namespace MARTe;

namespace gps_test {

bool buffers_equal(uint8* buffer1, uint8* buffer2, uint32 size) {
    for (uint32 i = 0; i < size; i++) {
        if (buffer1[i] != buffer2[i]) {
            return false;
        }
    }

    return true;
}

}

TEST(GPSBufferSize, IsZero) {
    SerialBuffer buffer(0u);

    ASSERT_EQ(buffer.size(), 0u);
}

TEST(GPSBufferSize, IsNonZero) {
    SerialBuffer buffer(1u);

    ASSERT_EQ(buffer.size(), 1u);
}

TEST(GPSBufferCount, IsInitiallyZero) {
    SerialBuffer buffer(1u);

    ASSERT_EQ(buffer.count(), 0u);
}

TEST(GPSBufferCount, IncreasesWhenCharsAppendedFromEmpty) {
    SerialBuffer buffer(10u);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};

    buffer.queue(&message[0], 5u);

    ASSERT_EQ(buffer.count(), 5u);
}

TEST(GPSBufferCount, IncreasesWhenCharsAppendedFromNonEmpty) {
    SerialBuffer buffer(10u);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    buffer.queue(&message[0], 3u);

    ASSERT_EQ(buffer.count(), 8u);
}

TEST(GPSBufferCount, DoesNotExceedSize) {
    SerialBuffer buffer(4);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    
    buffer.queue(&message[0], 5u);

    ASSERT_EQ(buffer.count(), 4u);
}

TEST(GPSBufferCount, ResetsToZeroOnEmpty) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    buffer.empty();

    ASSERT_EQ(buffer.count(), 0u);
}

TEST(GPSBufferAvailable, IsInitiallyEqualToSize) {
    SerialBuffer buffer(5u);

    ASSERT_EQ(buffer.size(), buffer.available());
}

TEST(GPSBufferAvailable, DecreasesByOne) {
    SerialBuffer buffer(5u);
    uint8 message = 0x00;

    buffer.queue(&message, 1u);

    ASSERT_EQ(buffer.size() - 1, buffer.available());
}

TEST(GPSBufferAvailable, DecreasesToZero) {
    SerialBuffer buffer(5u);
    uint8 message[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    buffer.queue(&message[0], 5u);

    ASSERT_EQ(0u, buffer.available());
}

TEST(GPSBufferQueue, AppendsBytesWhenEmpty) {
    SerialBuffer buffer(5);
    uint8 bytes_in[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    uint8 bytes_out[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    buffer.queue(&bytes_in[0], 5u);
    buffer.dequeue(&bytes_out[0], 5u);

    ASSERT_TRUE(gps_test::buffers_equal(bytes_in, bytes_out, 5u));
}

TEST(GPSBufferQueue, AppendsBytesWhenNonEmpty) {
    SerialBuffer buffer(10);
    uint8 bytes_in[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&bytes_in[0], 5u);

    buffer.queue(&bytes_in[0], 5u);

    uint8 bytes_out[10];
    buffer.dequeue(&bytes_out[0], 10u);
    ASSERT_TRUE(gps_test::buffers_equal(&bytes_out[0], bytes_in, 5u) && gps_test::buffers_equal(&bytes_out[5], bytes_in, 5u));
}

TEST(GPSBufferQueue, AppendsSomeOfMessageOnAvailability) {
    SerialBuffer buffer(7);
    uint8 bytes_in[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&bytes_in[0], 5u);

    uint32 n = buffer.queue(&bytes_in[0], 5u);

    ASSERT_EQ(n, 2u);
}

TEST(GPSBufferQueue, CanDequeueAPartialAppend) {
    SerialBuffer buffer(7);
    uint8 bytes_in[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&bytes_in[0], 5u);

    buffer.queue(&bytes_in[0], 5u);
    uint8 bytes_out[7u];
    buffer.dequeue(&bytes_out[0], 7u);

    ASSERT_TRUE(gps_test::buffers_equal(&bytes_out[0], bytes_in, 5u) && gps_test::buffers_equal(&bytes_out[5], bytes_in, 2u));
}

TEST(GPSBufferQueue, ReturnsNumberBytesAppended) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    
    uint32 n = buffer.queue(&message[0], 5u);

    ASSERT_EQ(n, 5u);
}

TEST(GPSBufferQueue, AppendedBytesMatchesAvailableSpace) {
    SerialBuffer buffer(4);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    
    uint32 n = buffer.queue(&message[0], 5u);

    ASSERT_EQ(n, 4u);
}

TEST(GPSBufferFind, ReturnsFalseIfValueNotFound) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint32 index;
    bool found = buffer.find(0x05, index);

    ASSERT_FALSE(found);
}

TEST(GPSBufferFind, ReturnsTrueIfValueFound) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint32 index;
    bool found = buffer.find(0x02, index);

    ASSERT_TRUE(found);
}

TEST(GPSBufferFind, ReturnsIndexIfValueFound) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint32 index;
    bool found = buffer.find(0x02, index);

    ASSERT_EQ(index, 2u);
}

TEST(GPSBufferFind, ReturnsFalseIfStartInvalid) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint32 index;
    bool found = buffer.find(0x02, index, 5);

    ASSERT_FALSE(found);
}

TEST(GPSBufferFind, SearchesOnlyAfterStart) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint32 index;
    bool found = buffer.find(0x01, index, 2);

    ASSERT_FALSE(found);
}

TEST(GPSBufferEmpty, LeavesAllCharactersInBuffer) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    buffer.empty(0);
    uint8 message_out[5];
    buffer.dequeue(message_out, 5);

    ASSERT_TRUE(gps_test::buffers_equal(&message_out[0], &message[0], 5u));
}

TEST(GPSBufferEmpty, LeavesSomeCharactersInBuffer) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    buffer.empty(2);
    uint8 message_out[5];
    buffer.dequeue(message_out, 5);

    ASSERT_TRUE(gps_test::buffers_equal(&message_out[0], &message[2], 3u));
}

TEST(GPSBufferEmpty, LeavesNoCharactersInBuffer) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    buffer.empty(5);
    uint8 message_out[5];
    ASSERT_EQ(buffer.dequeue(message_out, 5), 0u);
}

TEST(GPSBufferAt, ReturnsTrueIfIndexValid) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint8 value;
    bool ret = buffer.at(0, value);

    ASSERT_TRUE(ret);
}

TEST(GPSBufferAt, ReturnsValueIfIndexValidStart) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint8 value = 0x01;
    bool ret = buffer.at(0, value);

    ASSERT_EQ(0x00, value);
}

TEST(GPSBufferAt, ReturnsValueIfIndexValidEnd) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint8 value = 0x01;
    bool ret = buffer.at(4, value);

    ASSERT_EQ(0x04, value);
}

TEST(GPSBufferAt, ReturnsFalseIfIndexInvalid) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint8 value;
    bool ret = buffer.at(5, value);

    ASSERT_FALSE(ret);
}

TEST(GPSBufferAt, WrapsRoundCorrectly) {
    SerialBuffer buffer(3);
    uint8 message[3] = {0x00, 0x01, 0x02};
    buffer.queue(&message[0], 3u);
    buffer.empty(3u);

    buffer.queue(&message[1], 2u);
    uint8 value;
    bool ret = buffer.at(1, value);

    ASSERT_EQ(value, 0x02);
}

TEST(GPSBufferDequeue, ReducesBufferCount) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint8 bytes_out[3u];
    buffer.dequeue(&bytes_out[0], 3u);

    ASSERT_EQ(2u, buffer.count());
}

TEST(GPSBufferDequeue, ReducesBufferCountToZero) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint8 bytes_out[5u];
    buffer.dequeue(&bytes_out[0], 5u);

    ASSERT_EQ(0u, buffer.count());
}

TEST(GPSBufferDequeue, DoesNotReduceBufferCountIfZero) {
    SerialBuffer buffer(5);
    uint8 message[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    buffer.queue(&message[0], 5u);

    uint8 bytes_out[5u];
    buffer.dequeue(&bytes_out[0], 0u);

    ASSERT_EQ(5u, buffer.count());
}
