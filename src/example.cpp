/* Author: Tyler Weaver
   Desc: Simple example using the [boost].SML library
*/

// C++
#include <string>

// ROS
#include <ros/ros.h>

// [boost].SML
#include <boost/sml.hpp>
#include <boost_sml/logger.h>

namespace
{
namespace sml = boost::sml;

// Events
struct Spin{};

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

}  // anonymous namespace

int example_main(int argc, char** argv)
{
  const std::string node_name = "sml_example";

  // Initialize ROS
  ros::init(argc, argv, node_name);
  ROS_INFO_STREAM_NAMED(node_name, "Starting");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  SmlRosLogger logger(node_name);
  StateMachine state_machine{ logger };
  Spin spin{};

  ros::Rate loop_rate(1);
  while(ros::ok() && !state_machine.is(boost::sml::X))
  {
    state_machine.process_event(spin);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Shutdown
  ROS_INFO_STREAM_NAMED(node_name, "Shutting down.");
  spinner.stop();
  ros::shutdown();

  return 0;
}
