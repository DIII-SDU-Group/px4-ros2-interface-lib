/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "registration.hpp"
#include "px4_ros2/diagnostics/hil_trace.hpp"

#include <cassert>
#include <random>
#include <unistd.h>
#include <px4_ros2/utils/message_version.hpp>

static constexpr uint16_t kLatestPX4ROS2ApiVersion = 1;

using namespace std::chrono_literals;

Registration::Registration(rclcpp::Node & node, const std::string & topic_namespace_prefix)
: _node(node)
{
  auto created = px4_ros2::diagnostics::HilTrace::event("registration_object_constructed");
  created.number("registration_object_address", reinterpret_cast<uintptr_t>(this));
  created.text("node", node.get_fully_qualified_name());
  created.commit();

  _register_ext_component_reply_sub =
    node.create_subscription<px4_msgs::msg::RegisterExtComponentReply>(
    topic_namespace_prefix + "fmu/out/register_ext_component_reply" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RegisterExtComponentReply>(),
    // PX4's uXRCE-DDS bridge offers this reply stream as best-effort.  A
    // reliable subscriber is incompatible with that endpoint and leaves the
    // external-mode registration loop waiting forever for replies.
    rclcpp::QoS(1).best_effort(),
    [](px4_msgs::msg::RegisterExtComponentReply::UniquePtr msg) {
    });

  _register_ext_component_request_pub =
    node.create_publisher<px4_msgs::msg::RegisterExtComponentRequest>(
    topic_namespace_prefix + "fmu/in/register_ext_component_request" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RegisterExtComponentRequest>(),
    1);

  _unregister_ext_component_pub = node.create_publisher<px4_msgs::msg::UnregisterExtComponent>(
    topic_namespace_prefix + "fmu/in/unregister_ext_component" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::UnregisterExtComponent>(),
    1);

  _unregister_ext_component.mode_id = px4_ros2::ModeBase::kModeIDInvalid;
}

bool Registration::doRegister(const RegistrationSettings & settings)
{
  assert(!_registered);
  px4_msgs::msg::RegisterExtComponentRequest request{};

  if (settings.name.length() >= request.name.size() ||
    settings.name.length() >= _unregister_ext_component.name.size())
  {
    RCLCPP_ERROR(
      _node.get_logger(), "Name too long (%i >= %i)",
      (int)settings.name.length(), (int)request.name.size());
    return false;
  }

  RCLCPP_DEBUG(
    _node.get_logger(), "Registering '%s' (arming check: %i, mode: %i, mode executor: %i)",
    settings.name.c_str(),
    settings.register_arming_check, settings.register_mode, settings.register_mode_executor);

  strcpy(reinterpret_cast<char *>(request.name.data()), settings.name.c_str());
  request.register_arming_check = settings.register_arming_check;
  request.register_mode = settings.register_mode;
  request.register_mode_executor = settings.register_mode_executor;
  request.enable_replace_internal_mode = settings.enable_replace_internal_mode;
  request.replace_internal_mode = settings.replace_internal_mode;
  request.activate_mode_immediately = settings.activate_mode_immediately;
  request.px4_ros2_api_version = kLatestPX4ROS2ApiVersion;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint64_t> distrib{};
  request.request_id = distrib(gen);
  _last_request_id = request.request_id;

  auto begin = px4_ros2::diagnostics::HilTrace::event("registration_begin");
  begin.number("registration_object_address", reinterpret_cast<uintptr_t>(this));
  begin.text("node", _node.get_fully_qualified_name());
  begin.text("registration_name", settings.name);
  begin.number("request_id", request.request_id);
  begin.number("registration_generation", _diagnostic_generation + 1);
  begin.boolean("register_arming_check", settings.register_arming_check);
  begin.boolean("register_mode", settings.register_mode);
  begin.boolean("register_mode_executor", settings.register_mode_executor);
  begin.commit();

  // wait for subscription, it might take a while initially...
  for (int i = 0; i < 100; ++i) {
    if (_register_ext_component_request_pub->get_subscription_count() > 0) {
      RCLCPP_DEBUG(_node.get_logger(), "Subscriber found, continuing");
      break;
    }

    usleep(100000);
  }

  // send request and wait for response
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(_register_ext_component_reply_sub);

  bool got_reply = false;

  for (int retries = 0; retries < 5 && !got_reply; ++retries) {
    request.timestamp = 0; // Let PX4 set the timestamp
    auto published = px4_ros2::diagnostics::HilTrace::event("registration_request_published");
    published.number("registration_object_address", reinterpret_cast<uintptr_t>(this));
    published.text("registration_name", settings.name);
    published.number("request_id", request.request_id);
    published.number("attempt", retries + 1);
    published.commit();
    _register_ext_component_request_pub->publish(request);

    // wait for publisher, it might take a while initially...
    for (int i = 0; i < 100 && retries == 0; ++i) {
      if (_register_ext_component_reply_sub->get_publisher_count() > 0) {
        RCLCPP_DEBUG(_node.get_logger(), "Publisher found, continuing");
        break;
      }

      usleep(100000);
    }

    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = 1000ms; // CI simulation tests require this to be quite high

    while (!got_reply) {
      auto now = std::chrono::steady_clock::now();

      if (now >= start_time + timeout) {
        break;
      }

      auto wait_ret = wait_set.wait(timeout - (now - start_time));

      if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
        px4_msgs::msg::RegisterExtComponentReply reply;
        rclcpp::MessageInfo info;

        if (_register_ext_component_reply_sub->take(reply, info)) {
          reply.name.back() = '\0';

          const auto &rmw_info = info.get_rmw_message_info();
          auto observed = px4_ros2::diagnostics::HilTrace::event("registration_reply_observed");
          observed.number("registration_object_address", reinterpret_cast<uintptr_t>(this));
          observed.text("registration_name", settings.name);
          observed.number("request_id", request.request_id);
          observed.number("reply_request_id", reply.request_id);
          observed.number("reply_arming_check_id", reply.arming_check_id);
          observed.number("reply_mode_id", static_cast<uint64_t>(reply.mode_id));
          observed.boolean("reply_success", reply.success);
          observed.number("source_timestamp", static_cast<uint64_t>(rmw_info.source_timestamp));
          observed.number("received_timestamp", static_cast<uint64_t>(rmw_info.received_timestamp));
          observed.text("publisher_gid", px4_ros2::diagnostics::HilTrace::gid(rmw_info.publisher_gid));
          observed.commit();

          if (strcmp(
              reinterpret_cast<const char *>(reply.name.data()),
              settings.name.c_str()) == 0 &&
            request.request_id == reply.request_id)
          {
            RCLCPP_DEBUG(_node.get_logger(), "Got RegisterExtComponentReply");

            if (reply.success) {
              if (reply.px4_ros2_api_version == kLatestPX4ROS2ApiVersion) {
                _unregister_ext_component.arming_check_id = reply.arming_check_id;
                _unregister_ext_component.mode_id = reply.mode_id;
                _unregister_ext_component.mode_executor_id = reply.mode_executor_id;
                strcpy(
                  reinterpret_cast<char *>(_unregister_ext_component.name.data()),
                  settings.name.c_str());
                _registered = true;
                ++_diagnostic_generation;
                auto completed = px4_ros2::diagnostics::HilTrace::event("registration_completed");
                completed.number("registration_object_address", reinterpret_cast<uintptr_t>(this));
                completed.text("registration_name", settings.name);
                completed.number("request_id", request.request_id);
                completed.number("registration_generation", _diagnostic_generation);
                completed.number("arming_check_id", reply.arming_check_id);
                completed.number("mode_id", static_cast<uint64_t>(reply.mode_id));
                completed.number("mode_executor_id", static_cast<uint64_t>(reply.mode_executor_id));
                completed.commit();
              } else {
                RCLCPP_FATAL(
                  _node.get_logger(), "Incompatible ROS2 library API version: got %i, expected %i",
                  reply.px4_ros2_api_version, kLatestPX4ROS2ApiVersion);
              }

            } else {
              RCLCPP_ERROR(_node.get_logger(), "Registration failed");
            }

            got_reply = true;
          }

        } else {
          RCLCPP_INFO(_node.get_logger(), "no RegisterExtComponentReply message received");
        }

      } else {
        RCLCPP_INFO(_node.get_logger(), "timeout while registering external component");
      }
    }
  }

  wait_set.remove_subscription(_register_ext_component_reply_sub);

  auto end = px4_ros2::diagnostics::HilTrace::event("registration_end");
  end.number("registration_object_address", reinterpret_cast<uintptr_t>(this));
  end.text("registration_name", settings.name);
  end.number("request_id", request.request_id);
  end.number("registration_generation", _diagnostic_generation);
  end.boolean("registered", _registered);
  end.number("arming_check_id", static_cast<uint64_t>(_unregister_ext_component.arming_check_id));
  end.number("mode_id", static_cast<uint64_t>(_unregister_ext_component.mode_id));
  end.commit();

  return _registered;
}

void Registration::doUnregister()
{
  if (_registered) {
    auto begin = px4_ros2::diagnostics::HilTrace::event("registration_unregister_begin");
    begin.number("registration_object_address", reinterpret_cast<uintptr_t>(this));
    begin.text("registration_name", name());
    begin.number("registration_generation", _diagnostic_generation);
    begin.number("request_id", _last_request_id);
    begin.number("arming_check_id", static_cast<uint64_t>(_unregister_ext_component.arming_check_id));
    begin.number("mode_id", static_cast<uint64_t>(_unregister_ext_component.mode_id));
    begin.commit();
    RCLCPP_DEBUG(_node.get_logger(), "Unregistering");
    _unregister_ext_component.timestamp = 0; // Let PX4 set the timestamp
    _unregister_ext_component_pub->publish(_unregister_ext_component);
    _registered = false;
    auto end = px4_ros2::diagnostics::HilTrace::event("registration_unregister_end");
    end.number("registration_object_address", reinterpret_cast<uintptr_t>(this));
    end.text("registration_name", name());
    end.number("registration_generation", _diagnostic_generation);
    end.commit();
  }
}

void Registration::setRegistrationDetails(
  int arming_check_id, px4_ros2::ModeBase::ModeID mode_id,
  int mode_executor_id)
{
  _unregister_ext_component.arming_check_id = arming_check_id;
  _unregister_ext_component.mode_id = mode_id;
  _unregister_ext_component.mode_executor_id = mode_executor_id;
  _registered = true;
  ++_diagnostic_generation;
}
