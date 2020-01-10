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
struct SmlRosLogger
{
  SmlRosLogger(const std::string& name) : name_(name)
  {
  }

  template <class SM, class TEvent>
  void log_process_event(const TEvent&)
  {
    ROS_DEBUG_NAMED(name_ + ".sm.process_event", "[%s][process_event] %s", boost::sml::aux::get_type_name<SM>(),
                    boost::sml::aux::get_type_name<TEvent>());
  }

  template <class SM, class TGuard, class TEvent>
  void log_guard(const TGuard&, const TEvent&, bool result)
  {
    ROS_DEBUG_NAMED(name_ + ".sm.guard", "[%s][guard] %s %s %s", boost::sml::aux::get_type_name<SM>(),
                    boost::sml::aux::get_type_name<TGuard>(), boost::sml::aux::get_type_name<TEvent>(),
                    (result ? "[OK]" : "[Reject]"));
  }

  template <class SM, class TAction, class TEvent>
  void log_action(const TAction&, const TEvent&)
  {
    ROS_DEBUG_NAMED(name_ + ".sm.action", "[%s][action] %s %s", boost::sml::aux::get_type_name<SM>(),
                    boost::sml::aux::get_type_name<TAction>(), boost::sml::aux::get_type_name<TEvent>());
  }

  template <class SM, class TSrcState, class TDstState>
  void log_state_change(const TSrcState& src, const TDstState& dst)
  {
    ROS_DEBUG_NAMED(name_ + ".sm.state_change", "[%s][transition] %s -> %s", boost::sml::aux::get_type_name<SM>(),
                    src.c_str(), dst.c_str());
  }

  const std::string name_;
};  // struct SmlRosLogger
