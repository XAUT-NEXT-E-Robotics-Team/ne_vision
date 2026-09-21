#include "ne_vision/utils/ne_task.hpp"

#include <gtest/gtest.h>
#include <atomic>
#include <future>

using namespace ne_vision;
using namespace std::chrono_literals;

TEST(ChannelStop, StopRequestedBeforeWait)
{
  NeChannel<int> channel("test", NeChannelType_e::KEEP_ON_READ, 1);
  auto signal = std::make_shared<CvBracket_t>();
  channel.RegisterCv(signal);
  std::stop_source source;
  source.request_stop();
  auto token = source.get_token();
  channel.WaitForData(signal, token);
  EXPECT_FALSE(signal->second);
}

TEST(ChannelStop, DataNotificationThenStopWithoutNotification)
{
  NeChannel<int> channel("test", NeChannelType_e::KEEP_ON_READ, 1);
  auto signal = std::make_shared<CvBracket_t>();
  channel.RegisterCv(signal);
  channel.Transmit(42); // Notification before entering wait must be retained.
  std::stop_source source;
  auto token = source.get_token();
  channel.WaitForData(signal, token);
  EXPECT_FALSE(signal->second);
  int data = 0;
  ASSERT_TRUE(channel.Receive(data));
  EXPECT_EQ(data, 42);

  std::promise<void> entered, done;
  auto completed = done.get_future();
  std::jthread worker([&](std::stop_token stop) {
    entered.set_value();
    channel.WaitForData(signal, stop);
    done.set_value();
  });
  entered.get_future().wait();
  EXPECT_EQ(completed.wait_for(20ms), std::future_status::timeout);
  worker.request_stop(); // No notify_all(): stop-aware wait must wake itself.
  EXPECT_EQ(completed.wait_for(2s), std::future_status::ready);
}

struct Counter
{
  std::atomic<int> calls{0};
  void Run() { ++calls; }
};

TEST(TaskStop, ImmediateStopAndRestart)
{
  auto channel = std::make_shared<NeChannel<int>>("test", NeChannelType_e::KEEP_ON_READ, 1);
  Counter counter;
  NeTask task("event", NeTaskType_e::WAIT_FOR_CHANNEL_DATA, channel, &counter, &Counter::Run);
  for (int i = 0; i < 100; ++i)
  {
    task.Start();
    task.Stop(); // Exercise stop before/during entry into waiting.
  }
  task.Stop(); // Idempotent.
  EXPECT_EQ(counter.calls.load(), 0);
}

struct Once
{
  std::promise<void> called;
  void Run() { called.set_value(); }
};

TEST(TaskStop, InterruptLongInterval)
{
  Once callback;
  auto called = callback.called.get_future();
  NeTask task("interval", NeTaskType_e::WAIT_FOR_INTERVAL, 60s, &callback, &Once::Run);
  task.Start();
  ASSERT_EQ(called.wait_for(2s), std::future_status::ready);
  auto stopped = std::async(std::launch::async, [&] { task.Stop(); });
  EXPECT_EQ(stopped.wait_for(2s), std::future_status::ready);
}

struct BlockingCallback
{
  std::promise<void> entered;
  std::shared_future<void> release;
  std::atomic<int> calls{0};
  void Run()
  {
    if (++calls == 1)
    {
      entered.set_value();
      release.wait();
    }
  }
};

TEST(TaskStop, StopDuringCallbackWaitsForCallbackReturn)
{
  auto channel = std::make_shared<NeChannel<int>>("test", NeChannelType_e::KEEP_ON_READ, 1);
  std::promise<void> release;
  BlockingCallback callback;
  callback.release = release.get_future().share();
  auto entered = callback.entered.get_future();
  NeTask task("running", NeTaskType_e::WAIT_FOR_CHANNEL_DATA, channel, &callback, &BlockingCallback::Run);
  task.Start();
  channel->Transmit(1);
  ASSERT_EQ(entered.wait_for(2s), std::future_status::ready);
  auto stopped = std::async(std::launch::async, [&] { task.Stop(); });
  EXPECT_EQ(stopped.wait_for(20ms), std::future_status::timeout);
  release.set_value();
  EXPECT_EQ(stopped.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(callback.calls.load(), 1);
}
