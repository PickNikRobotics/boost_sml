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

/* Author: Tyler Weaver
   Desc: [boost].SML ros logger
*/

#pragma once

// C++
#include <string>

// ROS
#include <ros/ros.h>

// [boost].SML
#include <boost_sml/sml.hpp>

// logger struct
struct SmlRosLogger {
  SmlRosLogger(const std::string& name) : name_(name) {}

  template <class SM, class TEvent>
  void log_process_event(const TEvent&) {
    ROS_DEBUG_NAMED(name_ + ".sm.process_event", "[%s][process_event] %s",
                    boost::sml::aux::get_type_name<SM>(),
                    boost::sml::aux::get_type_name<TEvent>());
  }

  template <class SM, class TGuard, class TEvent>
  void log_guard(const TGuard&, const TEvent&, bool result) {
    ROS_DEBUG_NAMED(name_ + ".sm.guard", "[%s][guard] %s %s %s",
                    boost::sml::aux::get_type_name<SM>(),
                    boost::sml::aux::get_type_name<TGuard>(),
                    boost::sml::aux::get_type_name<TEvent>(),
                    (result ? "[OK]" : "[Reject]"));
  }

  template <class SM, class TAction, class TEvent>
  void log_action(const TAction&, const TEvent&) {
    ROS_DEBUG_NAMED(name_ + ".sm.action", "[%s][action] %s %s",
                    boost::sml::aux::get_type_name<SM>(),
                    boost::sml::aux::get_type_name<TAction>(),
                    boost::sml::aux::get_type_name<TEvent>());
  }

  template <class SM, class TSrcState, class TDstState>
  void log_state_change(const TSrcState& src, const TDstState& dst) {
    ROS_DEBUG_NAMED(name_ + ".sm.state_change", "[%s][transition] %s -> %s",
                    boost::sml::aux::get_type_name<SM>(), src.c_str(),
                    dst.c_str());
  }

  const std::string name_;
};  // struct SmlRosLogger
