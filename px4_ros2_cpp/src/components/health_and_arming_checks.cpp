/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "registration.hpp"
#include "px4_ros2/components/health_and_arming_checks.hpp"
#include "px4_ros2/diagnostics/hil_trace.hpp"
#include "px4_ros2/utils/message_version.hpp"

#include <cassert>
#include <atomic>
#include <chrono>
#include <utility>

using namespace std::chrono_literals;

namespace px4_ros2
{

namespace {
std::atomic<uint64_t> next_health_instance_sequence{0};
}

HealthAndArmingChecks::HealthAndArmingChecks(
  rclcpp::Node & node, CheckCallback check_callback,
  const std::string & topic_namespace_prefix)
: _node(node), _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
  _check_callback(std::move(check_callback)),
  _diagnostic_instance_sequence(next_health_instance_sequence.fetch_add(1, std::memory_order_relaxed) + 1)
{
  auto constructed = diagnostics::HilTrace::event("health_object_constructed");
  constructed.number("object_address", reinterpret_cast<uintptr_t>(this));
  constructed.number("instance_sequence", _diagnostic_instance_sequence);
  constructed.number("registration_object_address", reinterpret_cast<uintptr_t>(_registration.get()));
  constructed.text("node", _node.get_fully_qualified_name());
  constructed.number("registration_generation", _registration->diagnosticGeneration());
  constructed.commit();

  _arming_check_reply_pub = _node.create_publisher<px4_msgs::msg::ArmingCheckReply>(
    topic_namespace_prefix + "fmu/in/arming_check_reply" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ArmingCheckReply>(),
    // PX4's uXRCE-DDS subscriber accepts a reliable writer.  Buffer replies
    // from all concurrent external modes so one split-host packet loss does
    // not remove a mode from PX4's registration table.
    rclcpp::QoS(10).reliable());

  _arming_check_request_sub = _node.create_subscription<px4_msgs::msg::ArmingCheckRequest>(
    topic_namespace_prefix + "fmu/out/arming_check_request" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ArmingCheckRequest>(),
    rclcpp::QoS(1).best_effort().transient_local(),
    [this](px4_msgs::msg::ArmingCheckRequest::UniquePtr msg, const rclcpp::MessageInfo & info) {

      const auto callback_start = std::chrono::steady_clock::now();
      const auto &rmw_info = info.get_rmw_message_info();
      auto entry = diagnostics::HilTrace::event("health_callback_entry");
      entry.number("object_address", reinterpret_cast<uintptr_t>(this));
      entry.number("instance_sequence", _diagnostic_instance_sequence);
      entry.number("registration_object_address", reinterpret_cast<uintptr_t>(_registration.get()));
      entry.number("request_id", msg->request_id);
      entry.number("request_source_timestamp", static_cast<uint64_t>(rmw_info.source_timestamp));
      entry.number("request_received_timestamp", static_cast<uint64_t>(rmw_info.received_timestamp));
      entry.text("request_publisher_gid", diagnostics::HilTrace::gid(rmw_info.publisher_gid));
      entry.number("callback_thread_id", diagnostics::HilTrace::currentThreadId());
      entry.text("node", _node.get_fully_qualified_name());
      entry.text("callback_group", "px4_mode_default_mutually_exclusive");
      entry.text("callback_group_type", "MutuallyExclusive");
      entry.number("registration_generation", _registration->diagnosticGeneration());
      entry.boolean("registered", _registration->registered());
      if (_registration->registered()) {
        entry.number("registration_id", _registration->armingCheckId());
        entry.number("mode_id", static_cast<uint64_t>(_registration->modeId()));
        entry.text("registration_name", _registration->name());
      }
      entry.commit();

      RCLCPP_DEBUG_ONCE(
        _node.get_logger(), "Arming check request (id=%i, only printed once)",
        msg->request_id);

      if (_registration->registered()) {
        px4_msgs::msg::ArmingCheckReply reply{};
        reply.registration_id = _registration->armingCheckId();
        reply.request_id = msg->request_id;
        reply.can_arm_and_run = true;

        HealthAndArmingCheckReporter reporter(reply);
        _check_callback(reporter);

        _mode_requirements.fillArmingCheckReply(reply);

        reply.timestamp = 0; // Let PX4 set the timestamp
        auto publish_start = diagnostics::HilTrace::event("health_reply_publication_start");
        publish_start.number("object_address", reinterpret_cast<uintptr_t>(this));
        publish_start.number("instance_sequence", _diagnostic_instance_sequence);
        publish_start.number("registration_object_address", reinterpret_cast<uintptr_t>(_registration.get()));
        publish_start.number("registration_generation", _registration->diagnosticGeneration());
        publish_start.number("request_id", msg->request_id);
        publish_start.text("callback_group", "px4_mode_default_mutually_exclusive");
        publish_start.text("callback_group_type", "MutuallyExclusive");
        publish_start.number("registration_id", reply.registration_id);
        publish_start.number("mode_id", static_cast<uint64_t>(_registration->modeId()));
        publish_start.text("registration_name", _registration->name());
        publish_start.boolean("can_arm_and_run", reply.can_arm_and_run);
        publish_start.number("health_component_index", reply.health_component_index);
        publish_start.boolean("health_component_is_present", reply.health_component_is_present);
        publish_start.boolean("health_component_warning", reply.health_component_warning);
        publish_start.boolean("health_component_error", reply.health_component_error);
        publish_start.number("num_events", reply.num_events);
        publish_start.number("reply_timestamp", reply.timestamp);
        publish_start.commit();
        _arming_check_reply_pub->publish(reply);
        const auto publish_return = std::chrono::steady_clock::now();
        auto publish_end = diagnostics::HilTrace::event("health_reply_publication_return");
        publish_end.number("object_address", reinterpret_cast<uintptr_t>(this));
        publish_end.number("instance_sequence", _diagnostic_instance_sequence);
        publish_end.number("registration_object_address", reinterpret_cast<uintptr_t>(_registration.get()));
        publish_end.number("registration_generation", _registration->diagnosticGeneration());
        publish_end.number("request_id", msg->request_id);
        publish_end.text("callback_group", "px4_mode_default_mutually_exclusive");
        publish_end.text("callback_group_type", "MutuallyExclusive");
        publish_end.number("registration_id", reply.registration_id);
        publish_end.number("mode_id", static_cast<uint64_t>(_registration->modeId()));
        publish_end.text("registration_name", _registration->name());
        publish_end.boolean("can_arm_and_run", reply.can_arm_and_run);
        publish_end.number("health_component_index", reply.health_component_index);
        publish_end.boolean("health_component_is_present", reply.health_component_is_present);
        publish_end.boolean("health_component_warning", reply.health_component_warning);
        publish_end.boolean("health_component_error", reply.health_component_error);
        publish_end.number("num_events", reply.num_events);
        publish_end.number("reply_timestamp", reply.timestamp);
        publish_end.number(
          "request_to_reply_ns",
          static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            publish_return - callback_start).count()));
        publish_end.commit();
        _check_triggered = true;

      } else {
        RCLCPP_DEBUG(_node.get_logger(), "...not registered yet");
      }

      const auto callback_end = std::chrono::steady_clock::now();
      auto exit = diagnostics::HilTrace::event("health_callback_exit");
      exit.number("object_address", reinterpret_cast<uintptr_t>(this));
      exit.number("instance_sequence", _diagnostic_instance_sequence);
      exit.number("registration_object_address", reinterpret_cast<uintptr_t>(_registration.get()));
      exit.number("registration_generation", _registration->diagnosticGeneration());
      exit.number("request_id", msg->request_id);
      exit.text("callback_group", "px4_mode_default_mutually_exclusive");
      exit.text("callback_group_type", "MutuallyExclusive");
      exit.number(
        "callback_duration_ns",
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
          callback_end - callback_start).count()));
      exit.commit();
    });

  _watchdog_timer =
    _node.create_wall_timer(4s, [this] {
      const auto callback_start = std::chrono::steady_clock::now();
      auto callback_entry = diagnostics::HilTrace::event("callback_group_callback_entry");
      callback_entry.text("callback", "external_mode_health_watchdog");
      callback_entry.text("callback_group", "px4_mode_default_mutually_exclusive");
      callback_entry.text("callback_group_type", "MutuallyExclusive");
      callback_entry.text("node", _node.get_fully_qualified_name());
      callback_entry.commit();
      watchdogTimerUpdate();
      const auto callback_end = std::chrono::steady_clock::now();
      auto callback_exit = diagnostics::HilTrace::event("callback_group_callback_exit");
      callback_exit.text("callback", "external_mode_health_watchdog");
      callback_exit.text("callback_group", "px4_mode_default_mutually_exclusive");
      callback_exit.text("callback_group_type", "MutuallyExclusive");
      callback_exit.text("node", _node.get_fully_qualified_name());
      callback_exit.number(
        "duration_ns",
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
          callback_end - callback_start).count()));
      callback_exit.commit();
    });
}

HealthAndArmingChecks::~HealthAndArmingChecks()
{
  auto destroyed = diagnostics::HilTrace::event("health_object_destroyed");
  destroyed.number("object_address", reinterpret_cast<uintptr_t>(this));
  destroyed.number("instance_sequence", _diagnostic_instance_sequence);
  destroyed.number("registration_object_address", reinterpret_cast<uintptr_t>(_registration.get()));
  destroyed.text("node", _node.get_fully_qualified_name());
  destroyed.number("registration_generation", _registration->diagnosticGeneration());
  destroyed.boolean("registered", _registration->registered());
  destroyed.commit();
}

void HealthAndArmingChecks::overrideRegistration(const std::shared_ptr<Registration> & registration)
{
  assert(!_registration->registered());
  auto overridden = diagnostics::HilTrace::event("health_registration_overridden");
  overridden.number("object_address", reinterpret_cast<uintptr_t>(this));
  overridden.number("instance_sequence", _diagnostic_instance_sequence);
  overridden.number("previous_registration_object_address", reinterpret_cast<uintptr_t>(_registration.get()));
  overridden.number("registration_object_address", reinterpret_cast<uintptr_t>(registration.get()));
  overridden.text("node", _node.get_fully_qualified_name());
  overridden.number("registration_generation", registration->diagnosticGeneration());
  overridden.commit();
  _registration = registration;
}

bool HealthAndArmingChecks::doRegister(const std::string & name)
{
  assert(!_registration->registered());
  RegistrationSettings settings{};
  settings.name = name;
  settings.register_arming_check = true;
  auto begin = diagnostics::HilTrace::event("health_registration_begin");
  begin.number("object_address", reinterpret_cast<uintptr_t>(this));
  begin.number("instance_sequence", _diagnostic_instance_sequence);
  begin.number("registration_object_address", reinterpret_cast<uintptr_t>(_registration.get()));
  begin.text("registration_name", name);
  begin.number("registration_generation", _registration->diagnosticGeneration() + 1);
  begin.commit();
  const bool registered = _registration->doRegister(settings);
  auto end = diagnostics::HilTrace::event("health_registration_end");
  end.number("object_address", reinterpret_cast<uintptr_t>(this));
  end.number("instance_sequence", _diagnostic_instance_sequence);
  end.number("registration_object_address", reinterpret_cast<uintptr_t>(_registration.get()));
  end.text("registration_name", name);
  end.number("registration_generation", _registration->diagnosticGeneration());
  end.number("request_id", _registration->lastRequestId());
  end.number("arming_check_id", static_cast<uint64_t>(_registration->armingCheckId()));
  end.number("mode_id", static_cast<uint64_t>(_registration->modeId()));
  end.boolean("registered", registered);
  end.commit();
  return registered;
}

void HealthAndArmingChecks::watchdogTimerUpdate()
{
  if (_registration->registered()) {
    if (!_check_triggered && _shutdown_on_timeout) {
      rclcpp::shutdown();
      throw std::runtime_error(
              "Timeout, no request received from FMU, exiting (this can happen on FMU reboots)");
    }

    _check_triggered = false;

  } else {
    // avoid false positives while unregistered
    _check_triggered = true;
  }
}

} // namespace px4_ros2
