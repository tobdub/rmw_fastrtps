// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstring>
#include <functional>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "fastcdr/FastBuffer.h"
#include "fastdds/rtps/common/SerializedPayload.hpp"

#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "rcutils/allocator.h"
#include "rcutils/process.h"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/serialized_message.h"

#include "rmw_fastrtps_cpp/get_subscriber.hpp"
#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"

#include "rosidl_runtime_c/string_functions.h"

#include "test_msgs/msg/strings.h"

class TestTakeSerialized : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rmw_init_options_t options = rmw_get_zero_initialized_init_options();
    rmw_ret_t ret = rmw_init_options_init(&options, allocator);
    ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      rmw_ret_t ret = rmw_init_options_fini(&options);
      EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    });
    ret = rmw_enclave_options_copy("/", &allocator, &options.enclave);
    ASSERT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    options.discovery_options.automatic_discovery_range = RMW_AUTOMATIC_DISCOVERY_RANGE_LOCALHOST;
    ret = rmw_init(&options, &context);
    ASSERT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    node = rmw_create_node(&context, "test_take_serialized", "/");
    ASSERT_NE(nullptr, node) << rmw_get_error_string().str;
  }

  void TearDown() override
  {
    rmw_ret_t ret = rmw_destroy_node(node);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_shutdown(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_context_fini(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  }

  rmw_context_t context{rmw_get_zero_initialized_context()};
  rmw_node_t * node{nullptr};
};

// Take loops reuse one buffer when they skip a sample, so each payload must replace the last
TEST_F(TestTakeSerialized, cdr_buffer_takes_each_payload) {
  rmw_subscription_options_t options = rmw_get_default_subscription_options();
  rmw_subscription_t * sub = rmw_create_subscription(
    node, ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, Strings), "/test_take_serialized",
    &rmw_qos_profile_default, &options);
  ASSERT_NE(nullptr, sub) << rmw_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node, sub));
  });
  auto type_support = rmw_fastrtps_cpp::get_datareader(sub)->type();

  eprosima::fastcdr::FastBuffer buffer;
  rmw_fastrtps_shared_cpp::SerializedData data{
    rmw_fastrtps_shared_cpp::FASTDDS_SERIALIZED_DATA_TYPE_CDR_BUFFER, &buffer, nullptr};
  eprosima::fastdds::rtps::SerializedPayload_t skipped(8);
  skipped.length = skipped.max_size;
  std::memset(skipped.data, 'a', skipped.length);
  ASSERT_TRUE(type_support.deserialize(skipped, &data));

  eprosima::fastdds::rtps::SerializedPayload_t next(16);
  next.length = next.max_size;
  std::memset(next.data, 'b', next.length);
  ASSERT_TRUE(type_support.deserialize(next, &data));
  ASSERT_EQ(next.length, buffer.getBufferSize());
  EXPECT_EQ(0, std::memcmp(next.data, buffer.getBuffer(), next.length));
}

namespace
{

// Wait up to 10 s for a condition, polling every 10 ms
bool wait_for(const std::function<bool()> & condition)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

std::string string_value_of(const rmw_serialized_message_t & serialized)
{
  test_msgs__msg__Strings msg;
  EXPECT_TRUE(test_msgs__msg__Strings__init(&msg));
  EXPECT_EQ(
    RMW_RET_OK,
    rmw_deserialize(&serialized, ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, Strings), &msg))
    << rmw_get_error_string().str;
  std::string value(msg.string_value.data, msg.string_value.size);
  test_msgs__msg__Strings__fini(&msg);
  return value;
}

}  // namespace

class TestTakeSerializedPubSub : public TestTakeSerialized
{
protected:
  void SetUp() override
  {
    TestTakeSerialized::SetUp();
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&taken_msg, 0u, &allocator));
  }

  void TearDown() override
  {
    if (pub) {
      EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node, pub));
    }
    if (sub) {
      EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node, sub));
    }
    EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&taken_msg));
    TestTakeSerialized::TearDown();
  }

  // Create a publisher and a subscription on a topic of their own, and wait for them to match
  void create_pub_sub(bool ignore_local_publications)
  {
    const auto ts = ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, Strings);
    const std::string topic = "/test_take_serialized_" + std::to_string(rcutils_get_pid()) + "_" +
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
    rmw_publisher_options_t pub_options = rmw_get_default_publisher_options();
    pub = rmw_create_publisher(node, ts, topic.c_str(), &rmw_qos_profile_default, &pub_options);
    ASSERT_NE(nullptr, pub) << rmw_get_error_string().str;
    rmw_subscription_options_t sub_options = rmw_get_default_subscription_options();
    sub_options.ignore_local_publications = ignore_local_publications;
    sub = rmw_create_subscription(node, ts, topic.c_str(), &rmw_qos_profile_default, &sub_options);
    ASSERT_NE(nullptr, sub) << rmw_get_error_string().str;
    ASSERT_TRUE(
      wait_for(
        [this]() {
          size_t pubs = 0;
          size_t subs = 0;
          rmw_subscription_count_matched_publishers(sub, &pubs);
          rmw_publisher_count_matched_subscriptions(pub, &subs);
          return pubs == 1 && subs == 1;
        }));
  }

  void publish(const std::string & value)
  {
    test_msgs__msg__Strings msg;
    ASSERT_TRUE(test_msgs__msg__Strings__init(&msg));
    ASSERT_TRUE(rosidl_runtime_c__String__assign(&msg.string_value, value.c_str()));
    EXPECT_EQ(RMW_RET_OK, rmw_publish(pub, &msg, nullptr)) << rmw_get_error_string().str;
    test_msgs__msg__Strings__fini(&msg);
  }

  // Take into msg, waiting for a sample to arrive
  rmw_ret_t take(rmw_serialized_message_t * msg)
  {
    rmw_ret_t ret = RMW_RET_OK;
    bool taken = false;
    wait_for(
      [&]() {
        ret = rmw_take_serialized_message(sub, msg, &taken, nullptr);
        return ret != RMW_RET_OK || taken;
      });
    EXPECT_TRUE(ret != RMW_RET_OK || taken) << "no sample arrived";
    return ret;
  }

  rmw_publisher_t * pub{nullptr};
  rmw_subscription_t * sub{nullptr};
  rmw_serialized_message_t taken_msg = rmw_get_zero_initialized_serialized_message();
};

// rmw requires the message to be left unchanged when nothing is taken
TEST_F(TestTakeSerializedPubSub, ignored_local_sample_leaves_message_unchanged) {
  create_pub_sub(true);
  const char previous[] = "previous";
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_resize(&taken_msg, sizeof(previous)));
  std::memcpy(taken_msg.buffer, previous, sizeof(previous));
  taken_msg.buffer_length = sizeof(previous);
  const uint8_t * buffer = taken_msg.buffer;

  publish(std::string(1000, 'z'));
  auto reader = rmw_fastrtps_cpp::get_datareader(sub);
  ASSERT_TRUE(wait_for([&]() {return reader->get_unread_count() == 1;}));

  bool taken = true;
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_take_serialized_message(sub, &taken_msg, &taken, nullptr)) << rmw_get_error_string().str;
  EXPECT_FALSE(taken);
  EXPECT_EQ(buffer, taken_msg.buffer);
  EXPECT_EQ(sizeof(previous), taken_msg.buffer_capacity);
  ASSERT_EQ(sizeof(previous), taken_msg.buffer_length);
  EXPECT_EQ(0, std::memcmp(previous, taken_msg.buffer, sizeof(previous)));
}

TEST_F(TestTakeSerializedPubSub, failure_to_grow_message_is_reported) {
  create_pub_sub(false);
  rcutils_allocator_t failing = rcutils_get_default_allocator();
  failing.reallocate = [](void *, size_t, void *) -> void * {return nullptr;};
  rmw_serialized_message_t unsized = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&unsized, 0u, &failing));
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&unsized));
  });

  publish("lost");
  EXPECT_EQ(RMW_RET_BAD_ALLOC, take(&unsized));
  rmw_reset_error();

  // The failed sample was consumed, and the next one can be taken
  publish("next");
  ASSERT_EQ(RMW_RET_OK, take(&taken_msg)) << rmw_get_error_string().str;
  EXPECT_EQ("next", string_value_of(taken_msg));
}
