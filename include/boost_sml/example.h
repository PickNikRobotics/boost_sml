// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

// [boost].SML
#include <boost_sml/logger.h>

#include <boost_sml/sml.hpp>

namespace sml_example {
namespace sml = boost::sml;

// Events
struct Spin {};

// Actions
const auto do_sense = []() { ROS_INFO("do_sense"); };
const auto do_plan = []() { ROS_INFO("do_plan"); };
const auto do_execute = []() { ROS_INFO("do_execute"); };

struct StateMachineLogic {
  auto operator()() const {
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
