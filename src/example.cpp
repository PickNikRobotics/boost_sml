/* Author: Tyler Weaver
   Desc: Simple example using the [boost].SML library
*/

// C++
#include <string>

// ROS
#include <ros/ros.h>

#include <boost_sml/example.h>
using namespace sml_example;

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
  while (ros::ok() && !state_machine.is(boost::sml::X))
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
