///////////////////////////////////////////////////////////
//                                                       //
//                        .                 .:-:         //
//                       :-:              :-::           //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                ---------------------                  //
//               .-------:. :---------:                  //
//              :-----:.     .-------.                   //
//             .:---:         .-----.                    //
//            .:-:.             :-:                      //
//          .-:.                 .                       //
//         .:                                            //
//                                                       //
//    ███╗   ██╗███████╗██╗  ██╗████████╗    ███████╗    //
//    ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝    ██╔════╝    //
//    ██╔██╗ ██║█████╗   ╚███╔╝    ██║       █████╗      //
//    ██║╚██╗██║██╔══╝   ██╔██╗    ██║       ██╔══╝      //
//    ██║ ╚████║███████╗██╔╝ ██╗   ██║       ███████╗    //
//    ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝       ╚══════╝    //
//                                                       //
///////////////////////////////////////////////////////////
//                                                       //
// Copyright (c) 2026 XAUT NEXT-E. All Rights Reserved.  //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Description:
// Unit test 都TM AI写的，太懒了
// Unit test for channel and task.

#include "gtest/gtest.h"
#include "ne_vision/utils/ne_channel.hpp"
#include "ne_vision/utils/ne_task.hpp"
#include <thread>
#include <atomic>
#include <vector>

using namespace ne_vision;
using namespace std::chrono_literals;

// -----------------------------------------------------------------------------
// Channel Tests
// -----------------------------------------------------------------------------

TEST(NeChannelTest, PopOnRead)
{
  NeChannel<int> channel("pop_channel", NeChannelType_e::POP_ON_READ, 3);

  EXPECT_TRUE(channel.Empty());

  channel.Transmit(1);
  channel.Transmit(2);

  EXPECT_EQ(channel.Size(), 2);

  int val;
  EXPECT_TRUE(channel.Receive(val));
  EXPECT_EQ(val, 2);
  EXPECT_EQ(channel.Size(), 1);

  EXPECT_TRUE(channel.Receive(val));
  EXPECT_EQ(val, 1);
  EXPECT_TRUE(channel.Empty());

  EXPECT_FALSE(channel.Receive(val));
}

TEST(NeChannelTest, KeepOnRead)
{
  NeChannel<int> channel("keep_channel", NeChannelType_e::KEEP_ON_READ, 3);

  channel.Transmit(100);
  EXPECT_EQ(channel.Size(), 1);

  int val;
  // First read
  EXPECT_TRUE(channel.Receive(val));
  EXPECT_EQ(val, 100);
  EXPECT_EQ(channel.Size(), 1); // Should not decrease

  // Second read
  EXPECT_TRUE(channel.Receive(val));
  EXPECT_EQ(val, 100);
  EXPECT_EQ(channel.Size(), 1);
}

TEST(NeChannelTest, BufferOverflow)
{
  NeChannel<int> channel("overflow_channel", NeChannelType_e::POP_ON_READ, 2);

  channel.Transmit(1);
  channel.Transmit(2);
  EXPECT_EQ(channel.Size(), 2);

  // Transmit 3, should push out 1
  channel.Transmit(3);
  EXPECT_EQ(channel.Size(), 2);

  int val;
  EXPECT_TRUE(channel.Receive(val));
  EXPECT_EQ(val, 3); // Receive defaults to the newest element.

  EXPECT_TRUE(channel.Receive(val));
  EXPECT_EQ(val, 2);
}

TEST(NeChannelTest, ThreadSafety)
{
  NeChannel<int>   channel("ts_channel", NeChannelType_e::POP_ON_READ, 1000);
  std::atomic<int> received_count{0};
  const int        num_producers = 4;
  const int        num_items_per_producer = 100;

  std::vector<std::jthread> producers;
  for (int i = 0; i < num_producers; ++i)
  {
    producers.emplace_back([&channel, num_items_per_producer]() {
      for (int j = 0; j < num_items_per_producer; ++j)
      {
        channel.Transmit(1);
        std::this_thread::yield();
      }
    });
  }

  std::vector<std::jthread> consumers;
  std::atomic<bool>         stop_consumers{false};
  for (int i = 0; i < 4; ++i)
  {
    consumers.emplace_back([&channel, &received_count, &stop_consumers]() {
      int val;
      while (!stop_consumers || !channel.Empty())
      {
        if (channel.Receive(val))
        {
          received_count++;
        }
        else
        {
          std::this_thread::yield();
        }
      }
    });
  }

  // Join producers
  producers.clear();

  // Wait for channel to verify all data is consumed
  int retry = 0;
  while (received_count < num_producers * num_items_per_producer &&
         retry++ < 100)
  {
    std::this_thread::sleep_for(10ms);
  }

  stop_consumers = true;
  consumers.clear(); // Join consumers

  EXPECT_EQ(received_count, num_producers * num_items_per_producer);
}

TEST(NeChannelTest, FindOnKeepChannel)
{
  NeChannel<int> channel("find_channel", NeChannelType_e::KEEP_ON_READ, 5);

  channel.Transmit(10);
  channel.Transmit(20);
  channel.Transmit(30);

  int found_val;
  // Find an existing element
  bool result =
      channel.Find(found_val, [](const int& val) { return val == 20; });
  EXPECT_TRUE(result);
  EXPECT_EQ(found_val, 20);
  EXPECT_EQ(channel.Size(), 3); // Size should not change

  // Try to find a non-existing element
  result = channel.Find(found_val, [](const int& val) { return val == 99; });
  EXPECT_FALSE(result);

  // Test on a POP_ON_READ channel - should trigger an assertion
  NeChannel<int> pop_channel(
      "pop_find_channel", NeChannelType_e::POP_ON_READ, 5);
  pop_channel.Transmit(10);
  int unused_val;
  // Using ASSERT_DEATH to verify that the assertion inside Find() is triggered.
  // The second argument is a regex for the error message, but we can leave it
  // empty to just check for any non-graceful exit.
  ASSERT_DEATH(
      {
        pop_channel.Find(unused_val, [](const int& val) { return val == 10; });
      },
      "");
}

struct TimedData
{
  std::chrono::steady_clock::time_point timestamp;
  int                                   id;

  // Add a comparison operator for easy assertion
  bool operator==(const TimedData& other) const
  {
    return timestamp == other.timestamp && id == other.id;
  }
};

TEST(NeChannelTest, FindClosestPair)
{
  NeChannel<TimedData> channel(
      "closest_pair_channel", NeChannelType_e::KEEP_ON_READ, 10);
  auto start_time = std::chrono::steady_clock::now();

  // 1. A single element is returned as both sides of the bracket.
  TimedData data1 = {start_time + 100ms, 1};
  channel.Transmit(data1, data1.timestamp);
  std::pair<TimedData, TimedData> result_pair;
  EXPECT_TRUE(channel.FindClosestPair(
      start_time, result_pair, [](const TimedData& data) {
        return data.timestamp;
      }));
  EXPECT_EQ(result_pair.first.id, 1);
  EXPECT_EQ(result_pair.second.id, 1);

  // 2. Populate the channel
  TimedData data2 = {start_time + 200ms, 2};
  TimedData data3 = {start_time + 300ms, 3};
  TimedData data4 = {start_time + 400ms, 4};
  TimedData data5 = {start_time + 500ms, 5};
  channel.Transmit(data2, data2.timestamp);
  channel.Transmit(data3, data3.timestamp);
  channel.Transmit(data4, data4.timestamp);
  channel.Transmit(data5, data5.timestamp);
  EXPECT_EQ(channel.Size(), 5);

  // 3. Test a time between two points
  auto search_time1 = start_time + 250ms;
  EXPECT_TRUE(channel.FindClosestPair(
      search_time1, result_pair, [](const TimedData& data) {
        return data.timestamp;
      }));
  EXPECT_EQ(result_pair.first.id, 2);
  EXPECT_EQ(result_pair.second.id, 3);

  // 4. Test a time before all points
  auto search_time2 = start_time;
  EXPECT_TRUE(channel.FindClosestPair(
      search_time2, result_pair, [](const TimedData& data) {
        return data.timestamp;
      }));
  EXPECT_EQ(result_pair.first.id, 1);
  EXPECT_EQ(result_pair.second.id, 1);

  // 5. Test a time after all points
  auto search_time3 = start_time + 600ms;
  EXPECT_TRUE(channel.FindClosestPair(
      search_time3, result_pair, [](const TimedData& data) {
        return data.timestamp;
      }));
  EXPECT_EQ(result_pair.first.id, 5);
  EXPECT_EQ(result_pair.second.id, 5);

  // 6. Test a time that exactly matches a point
  auto search_time4 = start_time + 400ms;
  EXPECT_TRUE(channel.FindClosestPair(
      search_time4, result_pair, [](const TimedData& data) {
        return data.timestamp;
      }));
  EXPECT_EQ(result_pair.first.id, 4);
  EXPECT_EQ(result_pair.second.id, 4);
}

TEST(NeChannelTest, OrderedQueriesWithOutOfOrderTransmit)
{
  NeChannel<TimedData> channel(
      "out_of_order_channel", NeChannelType_e::KEEP_ON_READ, 10);
  auto start_time = std::chrono::steady_clock::now();

  TimedData data1 = {start_time + 100ms, 1};
  TimedData data2 = {start_time + 200ms, 2};
  TimedData data3 = {start_time + 300ms, 3};

  channel.Transmit(data1, data1.timestamp);
  channel.Transmit(data3, data3.timestamp);
  channel.Transmit(data2, data2.timestamp);

  std::pair<TimedData, TimedData> result_pair;
  EXPECT_TRUE(channel.FindClosestPair(
      start_time + 250ms,
      result_pair,
      [](const TimedData& data) { return data.timestamp; }));
  EXPECT_EQ(result_pair.first.id, 2);
  EXPECT_EQ(result_pair.second.id, 3);

  auto recent = channel.GetDataSince(
      start_time + 150ms,
      [](const TimedData& data) { return data.timestamp; });
  ASSERT_EQ(recent.size(), 2);
  EXPECT_EQ(recent[0].id, 2);
  EXPECT_EQ(recent[1].id, 3);

  TimedData found;
  EXPECT_TRUE(channel.Find(found, [](const TimedData& data) {
    return data.id == 2;
  }));
  EXPECT_EQ(found.id, 2);
}

// -----------------------------------------------------------------------------
// Channel Synchronizer Tests
// -----------------------------------------------------------------------------

struct InterpolatableData
{
  double value = 0;

  static InterpolatableData
  Interpolate(const InterpolatableData& lhs,
              const InterpolatableData& rhs,
              const NeChannelStamp_t&   lhs_stamp,
              const NeChannelStamp_t&   rhs_stamp,
              const NeChannelStamp_t&   target_stamp)
  {
    const auto ratio =
        std::chrono::duration<double>(target_stamp - lhs_stamp).count() /
        std::chrono::duration<double>(rhs_stamp - lhs_stamp).count();
    return {lhs.value + (rhs.value - lhs.value) * ratio};
  }
};

TEST(NeChannelSynchronizerTest, SelectsNearestData)
{
  const auto start = std::chrono::steady_clock::now();

  // 目标距离前一个数据更近。
  {
    auto target = std::make_shared<NeChannel<int>>(
        "target_before", NeChannelType_e::KEEP_ON_READ, 4);
    auto samples = std::make_shared<NeChannel<int>>(
        "samples_before", NeChannelType_e::KEEP_ON_READ, 4);
    target->Transmit(100, start + 100ms);
    samples->Transmit(80, start + 80ms);
    samples->Transmit(130, start + 130ms);

    NeChannelSynchronizer synchronizer(target, samples);
    ASSERT_EQ(synchronizer.Sync(), NeChannelSyncResult_e::SUCCESS);

    int result = 0;
    EXPECT_EQ(synchronizer.GetResultData(samples, result),
              NeChannelSyncMatchStatus_e::SUCCESS);
    EXPECT_EQ(result, 80);

    NeChannelSyncSlot<int> slot{samples};
    EXPECT_EQ(synchronizer.GetResultSlot(samples, slot),
              NeChannelSyncMatchStatus_e::SUCCESS);
    EXPECT_DOUBLE_EQ(slot.time_diff_ms, -20.0);
  }

  // 目标距离后一个数据更近。
  {
    auto target = std::make_shared<NeChannel<int>>(
        "target_after", NeChannelType_e::KEEP_ON_READ, 4);
    auto samples = std::make_shared<NeChannel<int>>(
        "samples_after", NeChannelType_e::KEEP_ON_READ, 4);
    target->Transmit(120, start + 120ms);
    samples->Transmit(80, start + 80ms);
    samples->Transmit(130, start + 130ms);

    NeChannelSynchronizer synchronizer(target, samples);
    ASSERT_EQ(synchronizer.Sync(), NeChannelSyncResult_e::SUCCESS);

    int result = 0;
    EXPECT_EQ(synchronizer.GetResultData(samples, result),
              NeChannelSyncMatchStatus_e::SUCCESS);
    EXPECT_EQ(result, 130);
  }

  // 距离相同时固定选择时间更早的数据。
  {
    auto target = std::make_shared<NeChannel<int>>(
        "target_equal", NeChannelType_e::KEEP_ON_READ, 4);
    auto samples = std::make_shared<NeChannel<int>>(
        "samples_equal", NeChannelType_e::KEEP_ON_READ, 4);
    target->Transmit(105, start + 105ms);
    samples->Transmit(80, start + 80ms);
    samples->Transmit(130, start + 130ms);

    NeChannelSynchronizer synchronizer(target, samples);
    ASSERT_EQ(synchronizer.Sync(), NeChannelSyncResult_e::SUCCESS);

    int result = 0;
    EXPECT_EQ(synchronizer.GetResultData(samples, result),
              NeChannelSyncMatchStatus_e::SUCCESS);
    EXPECT_EQ(result, 80);
  }
}

TEST(NeChannelSynchronizerTest, UsesWatermarkAndInterpolation)
{
  const auto start = std::chrono::steady_clock::now();
  auto target = std::make_shared<NeChannel<int>>(
      "target", NeChannelType_e::KEEP_ON_READ, 4);
  auto samples = std::make_shared<NeChannel<InterpolatableData>>(
      "interpolatable", NeChannelType_e::KEEP_ON_READ, 4);

  target->Transmit(100, start + 100ms);
  samples->Transmit({8.0}, start + 80ms);
  samples->Transmit({12.0}, start + 120ms);

  NeChannelSynchronizer synchronizer(target, samples);
  ASSERT_EQ(synchronizer.Sync(), NeChannelSyncResult_e::SUCCESS);
  EXPECT_EQ(synchronizer.GetTargetStamp(), start + 100ms);

  InterpolatableData result;
  EXPECT_EQ(synchronizer.GetResultData(samples, result),
            NeChannelSyncMatchStatus_e::SUCCESS);
  EXPECT_DOUBLE_EQ(result.value, 10.0);

  NeChannelSyncSlot<InterpolatableData> slot{samples};
  ASSERT_EQ(synchronizer.GetResultSlot(samples, slot),
            NeChannelSyncMatchStatus_e::SUCCESS);
  EXPECT_DOUBLE_EQ(slot.time_diff_ms, 0.0);
  EXPECT_FALSE(slot.is_target);
}

TEST(NeChannelSynchronizerTest, ReturnsWarningForTargetBeforeOldest)
{
  const auto start = std::chrono::steady_clock::now();
  auto target = std::make_shared<NeChannel<int>>(
      "target", NeChannelType_e::KEEP_ON_READ, 4);
  auto samples = std::make_shared<NeChannel<int>>(
      "samples", NeChannelType_e::KEEP_ON_READ, 4);

  target->Transmit(100, start + 100ms);
  samples->Transmit(130, start + 130ms);

  NeChannelSynchronizer synchronizer(target, samples);
  ASSERT_EQ(synchronizer.Sync(),
            NeChannelSyncResult_e::SUCCESS_WITH_WARNING);

  int result = 0;
  EXPECT_EQ(synchronizer.GetResultData(samples, result),
            NeChannelSyncMatchStatus_e::TARGET_BEFORE_OLDEST);
  EXPECT_EQ(result, 130);
}

TEST(NeChannelSynchronizerTest, RejectsReadsWhileNotReady)
{
  auto first = std::make_shared<NeChannel<int>>(
      "first", NeChannelType_e::KEEP_ON_READ, 4);
  auto empty = std::make_shared<NeChannel<int>>(
      "empty", NeChannelType_e::KEEP_ON_READ, 4);
  first->Transmit(1);

  NeChannelSynchronizer synchronizer(first, empty);

  EXPECT_DEATH((void)synchronizer.GetTargetStamp(), "Synchronizer is not ready");

  int result = 0;
  EXPECT_DEATH((void)synchronizer.GetResultData(first, result),
               "Synchronizer is not ready");

  ASSERT_EQ(synchronizer.Sync(), NeChannelSyncResult_e::NOT_READY);
  EXPECT_DEATH((void)synchronizer.GetResultData(first, result),
               "Synchronizer is not ready");
}

// -----------------------------------------------------------------------------
// Task Tests
// -----------------------------------------------------------------------------

class Worker
{
public:
  void Run()
  {
    // Heavy work simulation
    // std::this_thread::sleep_for(1ms);
    counter++;
  }

  void Consume(std::shared_ptr<NeChannel<int>> chan)
  {
    int val;
    while (chan->Receive(val))
    {
      counter++;
    }
  }

  std::atomic<int> counter{0};
};

TEST(NeTaskTest, WaitForInterval)
{
  Worker worker;
  NeTask task("interval_task",
              NeTaskType_e::WAIT_FOR_INTERVAL,
              50ms,
              &worker,
              &Worker::Run);

  task.Start();
  std::this_thread::sleep_for(220ms); // Should execute approx 4 times
  task.Stop();

  // Allow variance due to scheduler
  EXPECT_GE(worker.counter, 3);
  EXPECT_LE(worker.counter, 6);
}

TEST(NeTaskTest, WaitForChannelData)
{
  auto channel = std::make_shared<NeChannel<int>>(
      "task_channel", NeChannelType_e::POP_ON_READ, 100);
  Worker worker;
  NeTask task("channel_task",
              NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
              channel,
              &worker,
              &Worker::Consume, // Use Consume
              channel);         // Pass channel as arg

  task.Start();

  // 1. Single transmit
  channel->Transmit(1);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(worker.counter, 1);

  // 2. Burst transmit
  for (int i = 0; i < 5; ++i)
  {
    channel->Transmit(i);
  }
  // Give enough time for task to process 5 items
  std::this_thread::sleep_for(100ms);

  task.Stop();
  EXPECT_EQ(worker.counter, 6);
}

TEST(NeTaskTest, TaskLifecycleRobustness)
{
  Worker worker;
  // Create, Start, Stop multiple times rapidly
  for (int i = 0; i < 10; ++i)
  {
    NeTask task("stress_task",
                NeTaskType_e::WAIT_FOR_INTERVAL,
                1ms,
                &worker,
                &Worker::Run);
    task.Start();
    std::this_thread::sleep_for(2ms);
    task.Stop();
  }
  // Just ensure no crash
  EXPECT_GT(worker.counter, 0);
}

TEST(NeTaskTest, MultipleTasksOneChannel)
{
  auto channel = std::make_shared<NeChannel<int>>(
      "shared_channel", NeChannelType_e::POP_ON_READ, 100);
  Worker worker1, worker2;

  // Bind channel to both workers so they consume from it
  NeTask task1("t1",
               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
               channel,
               &worker1,
               &Worker::Consume,
               channel);
  NeTask task2("t2",
               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
               channel,
               &worker2,
               &Worker::Consume,
               channel);

  task1.Start();
  task2.Start();

  // Transmit 20 items
  for (int i = 0; i < 20; ++i)
  {
    channel->Transmit(i);
  }
  std::this_thread::sleep_for(200ms);

  task1.Stop();
  task2.Stop();

  // Both workers should have processed some, total should be 20
  EXPECT_EQ(worker1.counter + worker2.counter, 20);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
