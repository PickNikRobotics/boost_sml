#pragma once

// [boost].SML
#include <boost_sml/sml.hpp>
#include <boost_sml/logger.h>

namespace sml_example {
namespace sml = boost::sml;

// Events
struct Spin
{
};

// Actions
const auto do_sense = []() { ROS_INFO("do_sense"); };
const auto do_plan = []() { ROS_INFO("do_plan"); };
const auto do_execute = []() { ROS_INFO("do_execute"); };

struct StateMachineLogic
{
  auto operator()() const
  {
    using sml::event;
    using sml::operator""_s;
    using sml::X;

    // clang-format off
    return sml::make_transition_table(
      *"idle"_s     + event<Spin>              = "sensing"_s,
      "sensing"_s   + event<Spin> / do_sense   = "planning"_s,
      "planning"_s  + event<Spin> / do_plan    = "executing"_s,
      "executing"_s + event<Spin> / do_execute = X);
  }
};

using StateMachine = sml::sm<StateMachineLogic, sml::logger<SmlRosLogger>>;

} // namespace sml_example
