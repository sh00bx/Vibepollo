/**
 * @file tests/unit/test_thread_safe.cpp
 * @brief Test src/thread_safe.h
 */

#include <gtest/gtest.h>

#include "src/thread_safe.h"

TEST(MailRegistryTests, QueueLookupReplacesExpiredPost) {
  constexpr auto id = "stale_queue";
  auto mail = std::make_shared<safe::mail_raw_t>();
  auto original = mail->queue<int>(id);
  std::weak_ptr<void> stale = original;

  original.reset();
  ASSERT_TRUE(stale.expired());
  mail->id_to_post.emplace(id, stale);

  auto replacement = mail->queue<int>(id);

  ASSERT_NE(replacement, nullptr);
  EXPECT_FALSE(std::weak_ptr<void> {replacement}.expired());
}

TEST(MailRegistryTests, EventLookupReplacesExpiredPost) {
  constexpr auto id = "stale_event";
  auto mail = std::make_shared<safe::mail_raw_t>();
  auto original = mail->event<bool>(id);
  std::weak_ptr<void> stale = original;

  original.reset();
  ASSERT_TRUE(stale.expired());
  mail->id_to_post.emplace(id, stale);

  auto replacement = mail->event<bool>(id);

  ASSERT_NE(replacement, nullptr);
  EXPECT_FALSE(std::weak_ptr<void> {replacement}.expired());
}

TEST(QueueOverflow, DropOldestKeepsBacklog) {
  safe::queue_t<int> q(4);
  for (int i = 0; i < 4; ++i) {
    q.raise(i);
  }
  EXPECT_FALSE(q.consume_overflow());

  // Overflow drops only the front element
  q.raise(4);
  EXPECT_TRUE(q.consume_overflow());
  EXPECT_FALSE(q.consume_overflow());  // check-and-clear

  auto val = q.pop();
  ASSERT_TRUE(val);
  EXPECT_EQ(*val, 1);
  EXPECT_EQ(q.unsafe().size(), 3u);  // 2, 3, 4 retained
}

TEST(QueueOverflow, DrainToNewestDropsBacklog) {
  safe::queue_t<int> q(4);
  q.set_overflow_policy(safe::queue_t<int>::overflow_e::drain_to_newest);
  for (int i = 0; i < 4; ++i) {
    q.raise(i);
  }

  // Overflow drops the whole backlog, keeping only the new element
  q.raise(4);
  EXPECT_TRUE(q.consume_overflow());
  EXPECT_EQ(q.unsafe().size(), 1u);

  auto val = q.pop();
  ASSERT_TRUE(val);
  EXPECT_EQ(*val, 4);
  EXPECT_TRUE(q.unsafe().empty());
}

TEST(QueueOverflow, NoOverflowNoFlag) {
  safe::queue_t<int> q(4);
  q.set_overflow_policy(safe::queue_t<int>::overflow_e::drain_to_newest);
  q.raise(1);
  q.raise(2);
  EXPECT_FALSE(q.consume_overflow());
}
